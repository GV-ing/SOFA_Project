"""
main_scene.py

Scena SOFA del Progetto 5: tubo in Nitinol con tendine (CableConstraint), oggetto di contatto
rigido o soffice e controller della molla in serie.
SOFA v26.06, SofaPython3, Python 3.12.

Uso:
    ./run_gui.sh [--object soft|rigid|none] [--spring fixed|switched|direct]
"""
import os
import sys
import numpy as np
import Sofa
import Sofa.Core

# cartella del progetto nel path per importare modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from modules.config import NOMINAL_PARAMS, SPRING_CONFIGS
from modules.geometry_gen import (
    box_hexa_mesh,
    box_surface,
    build_tube_meshes,
    check_tet_mesh,
    contact_object_axes,
)
from modules.controller import TendonSpringController
from modules.contact_force import read_contact_force


# nomi usati in config.py -> componenti SOFA
_COLLISION_MODELS = {
    "triangle": "TriangleCollisionModel",
    "line": "LineCollisionModel",
    "point": "PointCollisionModel",
}


# converte i nomi dei modelli di collisione di config.py nei componenti SOFA
def _collision_model_names(names):
    if isinstance(names, str):
        names = (names,)
    try:
        models = [_COLLISION_MODELS[str(n).lower()] for n in names]
    except KeyError as exc:
        raise ValueError(f"Modello di collisione sconosciuto: {exc}; usare Triangle/Line/Point")
    if not models:
        raise ValueError("Serve almeno un modello di collisione")
    return models


# costruisce la scena; senza custom_params (avvio da runSofa) vale il profilo GUI
def createScene(rootNode, custom_params=None):
    params = NOMINAL_PARAMS.copy()
    gui = custom_params is None
    if gui and params.get("gui_fast", True):
        # profilo GUI: valori di gui_overrides
        params.update(params["gui_overrides"])
        print(f"[GUI] profilo interattivo: {params['gui_overrides']}")
    if gui:
        # oggetto, molla e ordine dei livelli scelti da run_gui.sh con variabili d'ambiente
        obj = os.environ.get("P5_OBJECT")
        if obj:
            params["contact_object"] = obj
        spring = os.environ.get("P5_SPRING")
        if spring:
            if spring not in SPRING_CONFIGS:
                raise ValueError("P5_SPRING deve essere 'fixed', 'switched' oppure 'direct'")
            params.update(SPRING_CONFIGS[spring])
        order = os.environ.get("P5_K_FREE")
        if order:
            if order not in ("high", "low"):
                raise ValueError("P5_K_FREE deve essere 'high' (4.8 libero) o 'low' (0.6 libero)")
            params["k_free_level"] = order
        if params["switched_mode"]:
            k_f, k_c = ((params["k_high"], params["k_low"]) if params["k_free_level"] == "high"
                        else (params["k_low"], params["k_high"]))
            print(f"[GUI] molla a due livelli: {k_f} N/mm in moto libero, {k_c} N/mm in contatto")
        print(f"[GUI] oggetto: {params['contact_object']} | molla: "
              f"{'due livelli 0.6/4.8 N/mm' if params['switched_mode'] else str(params['k_spring']) + ' N/mm'}"
              f" | forza in contatto {params['force_ref'] * 1e3:g} mN +/-{params['force_band'] * 100:g} %")
    if custom_params is not None:
        params.update(custom_params)
    # con la vista amplificata il modello visivo reale diventa semitrasparente
    ghost_alpha = 0.25 if gui and params.get("visual_amplification", 1.0) > 1.0 else 1.0
    if params["contact_object"] not in {"soft", "rigid", "none"}:
        raise ValueError("contact_object deve essere 'soft', 'rigid' oppure 'none'")

    # 1. nodo radice (unità: mm, N, MPa, s), senza gravità
    rootNode.dt = params["dt"]
    rootNode.gravity = [0.0, 0.0, 0.0]

    # 2. plugin
    plugins = [
        "Sofa.Component.AnimationLoop",
        "Sofa.Component.Collision.Detection.Algorithm",
        "Sofa.Component.Collision.Detection.Intersection",
        "Sofa.Component.Collision.Geometry",
        "Sofa.Component.Collision.Response.Contact",
        "Sofa.Component.Constraint.Lagrangian.Correction",
        "Sofa.Component.Constraint.Lagrangian.Solver",
        "Sofa.Component.Constraint.Projective",
        "Sofa.Component.Engine.Select",
        "Sofa.Component.IO.Mesh",
        "Sofa.Component.LinearSolver.Direct",
        "Sofa.Component.Mapping.Linear",
        "Sofa.Component.Mass",
        "Sofa.Component.ODESolver.Backward",
        "Sofa.Component.Setting",
        "Sofa.Component.SolidMechanics.FEM.Elastic",
        "Sofa.Component.StateContainer",
        "Sofa.Component.Topology.Container.Constant",
        "Sofa.Component.Topology.Container.Dynamic",
        "Sofa.Component.Topology.Container.Grid",
        "Sofa.Component.Topology.Mapping",
        "Sofa.Component.Visual",
        "Sofa.GL.Component.Rendering3D",
        "SoftRobots",
    ]
    # con parallel_components si usano le versioni Parallel dei componenti (plugin MultiThreading)
    par = "Parallel" if params.get("parallel_components", False) else ""
    if par:
        plugins.append("MultiThreading")
    for p in plugins:
        rootNode.addObject("RequiredPlugin", name="req_" + p.replace(".", "_"), pluginName=p)

    # 3. visualizzazione
    rootNode.addObject("DefaultVisualManagerLoop")
    rootNode.addObject("VisualStyle", name="visualStyle",
                       displayFlags=params.get("display_flags", "showVisualModels"))

    # 4. animation loop e solver dei vincoli
    rootNode.addObject("FreeMotionAnimationLoop", name="FreeMotionLoop")
    rootNode.addObject("BlockGaussSeidelConstraintSolver", name="ConstraintSolver",
                       maxIterations=int(params.get("cs_max_iterations", 500)),
                       tolerance=float(params.get("cs_tolerance", 1e-7)),
                       computeConstraintForces=True)

    # 5. collisioni: pipeline, LocalMinDistance e risposta con attrito
    rootNode.addObject("CollisionPipeline", name="CollisionPipeline", depth=6)
    rootNode.addObject(par + "BruteForceBroadPhase", name="BroadPhase")
    rootNode.addObject(par + "BVHNarrowPhase", name="NarrowPhase")
    rootNode.addObject(
        "LocalMinDistance",
        name="LocalMinDistance",
        alarmDistance=params["alarm_distance"],
        contactDistance=params["contact_distance"],
        angleCone=0.01,
    )
    rootNode.addObject("CollisionResponse", name="Response",
                       response="FrictionContactConstraint",
                       responseParams="mu=" + str(params["mu_friction"]))

    # 6. mesh del tubo: FEM tetraedrica graduata e superfici visiva e di collisione,
    #    rigenerate a ogni avvio in meshes/scena/
    mesh_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes", "scena")
    fem_nodes, tetrahedra, mesh_files = build_tube_meshes(params, mesh_dir)
    obj_path = mesh_files["visual"]
    collision_obj_path = mesh_files["collision"]
    bad, vmin, tube_volume = check_tet_mesh(fem_nodes, tetrahedra)
    if bad > 0:
        raise RuntimeError(f"Mesh tubo non valida: {bad} tetraedri invertiti/degeneri (vol min {vmin:.4g})")

    # nodi della sezione di punta (x = length_total)
    nodes_np = np.array(fem_nodes)
    tip_node_indices = np.where(np.isclose(nodes_np[:, 0], params["length_total"], atol=1e-3))[0].tolist()

    # 7. tubo: solver, topologia, massa e FEM
    tubeNode = rootNode.addChild("NitinolTube")
    tubeNode.addObject("EulerImplicitSolver", name="odesolver", rayleighStiffness=0.1, rayleighMass=0.1)
    tubeNode.addObject("SparseLDLSolver", name="linearsolver", template="CompressedRowSparseMatrixMat3x3d")

    tubeNode.addObject("MeshTopology", name="topology", position=fem_nodes, tetrahedra=tetrahedra)
    tubeMeca = tubeNode.addObject("MechanicalObject", name="dofs", template="Vec3d")
    # massa totale = densità * volume della mesh
    tubeNode.addObject("UniformMass", name="mass", totalMass=params["rho_tube"] * tube_volume)

    femFF = tubeNode.addObject(
        par + "TetrahedronFEMForceField",
        name="FEM",
        youngModulus=params["E_load"],
        poissonRatio=params["poisson"],
        method="large",
    )

    # albero prossimale bloccato (primi length_shaft mm)
    tubeNode.addObject("BoxROI", name="shaftROI",
                       box=[-0.5, -2.0, -2.0, params["length_shaft"] + 0.01, 2.0, 2.0],
                       drawBoxes=True)
    tubeNode.addObject("FixedProjectiveConstraint", name="FixedShaft", indices="@shaftROI.indices")
    tubeNode.addObject("LinearSolverConstraintCorrection", name="LinearConstraintCorrection")

    # 8. tendine: punti a y = yt_offset dalla fine dell'albero alla punta, tirati dal pullPoint
    tendonNode = tubeNode.addChild("Tendon")
    num_cable_pts = 15
    cable_x = np.linspace(params["length_shaft"], params["length_total"], num_cable_pts)
    cable_pts = [[float(x), float(params["yt_offset"]), 0.0] for x in cable_x]

    tendonNode.addObject("MechanicalObject", name="cablePts", position=cable_pts)
    cableConstraint = tendonNode.addObject(
        "CableConstraint",
        name="tendonCable",
        indices=list(range(num_cable_pts)),
        pullPoint=[params["length_shaft"], params["yt_offset"], 0.0],
        valueType="force",
        value=0.0,
        hasPullPoint=True,
    )
    tendonNode.addObject("BarycentricMapping", name="tendonMapping")

    # 9. modelli visivo e di collisione del tubo, mappati sulla mesh FEM
    visualNode = tubeNode.addChild("VisualModel")
    visualNode.addObject("MeshOBJLoader", name="objLoader", filename=obj_path)
    visualNode.addObject("OglModel", name="Visual", src="@objLoader", color=[0.75, 0.78, 0.82, ghost_alpha])
    visualNode.addObject("BarycentricMapping", name="visualMapping")

    tubeCollisionNode = tubeNode.addChild("CollisionModel")
    tubeCollisionNode.addObject("MeshOBJLoader", name="collisionLoader", filename=collision_obj_path)
    tubeCollisionNode.addObject("MeshTopology", src="@collisionLoader")
    tubeCollisionNode.addObject("MechanicalObject", name="dofs", src="@collisionLoader", template="Vec3d")
    for model in _collision_model_names(params["tube_collision_models"]):
        tubeCollisionNode.addObject(model, group=0, simulated=True, moving=True)
    tubeCollisionNode.addObject("BarycentricMapping")

    # 10. oggetto di contatto davanti alla punta
    L = params["length_total"]
    contact_object = params["contact_object"]
    if contact_object != "none":
        cube_size = float(params["obj_size"])
        grid_n = int(params["obj_grid_n"])
        overlap_x = float(params["obj_overlap_x"])
        if cube_size <= 0 or overlap_x < 0 or grid_n < 2:
            raise ValueError("obj_size > 0, obj_overlap_x >= 0 e obj_grid_n >= 2 richiesti")

        xs, ys, zs, bounds, graded = contact_object_axes(params)
        x_min, x_max, y_min, y_max, z_min, z_max = bounds
        collision_positions, collision_triangles = box_surface(xs, ys, zs)

        contactNode = rootNode.addChild(
            "SoftContactObject" if contact_object == "soft" else "RigidContactObject"
        )
        # oggetto soffice: FEM esaedrica con la faccia posteriore bloccata
        if contact_object == "soft":
            contactNode.addObject("EulerImplicitSolver", name="odesolver",
                                  rayleighStiffness=0.1, rayleighMass=0.1)
            contactNode.addObject("SparseLDLSolver", name="linearsolver",
                                  template="CompressedRowSparseMatrixMat3x3d")
            # griglia graduata (più fitta al contatto) oppure griglia regolare
            if graded:
                soft_pos, soft_hexa = box_hexa_mesh(xs, ys, zs)
                contactNode.addObject("HexahedronSetTopologyContainer", name="grid",
                                      position=soft_pos, hexahedra=soft_hexa)
                contactNode.addObject("MechanicalObject", name="state", template="Vec3d",
                                      position=soft_pos)
            else:
                contactNode.addObject(
                    "RegularGridTopology",
                    name="grid",
                    n=[grid_n, grid_n, grid_n],
                    min=[x_min, y_min, z_min],
                    max=[x_max, y_max, z_max],
                )
                contactNode.addObject("MechanicalObject", name="state", template="Vec3d")
            soft_volume = (x_max - x_min) * (y_max - y_min) * (z_max - z_min)
            contactNode.addObject("UniformMass", name="mass",
                                  totalMass=params["rho_soft"] * soft_volume)
            contactNode.addObject(
                par + "HexahedronFEMForceField",
                name="SoftFEM",
                topology="@grid",
                youngModulus=params["E_soft"],
                poissonRatio=params["poisson_soft"],
                method="large",
            )
            # faccia posteriore (y = y_max)
            contactNode.addObject(
                "BoxROI",
                name="backROI",
                box=[
                    x_min - 0.1, y_max - 0.1, z_min - 0.1,
                    x_max + 0.1, y_max + 0.1, z_max + 0.1,
                ],
            )
            contactNode.addObject(
                "FixedProjectiveConstraint",
                name="FixedBack",
                indices="@backROI.indices",
            )
            contactNode.addObject(
                "LinearSolverConstraintCorrection",
                name="LinearConstraintCorrection",
            )

        # superficie di collisione dell'oggetto
        collisionNode = contactNode.addChild("CollisionModel")
        collisionNode.addObject(
            "TriangleSetTopologyContainer",
            name="topology",
            position=collision_positions,
            triangles=collision_triangles,
        )
        collisionNode.addObject("TriangleSetTopologyModifier", name="modifier")
        collisionNode.addObject(
            "TriangleSetGeometryAlgorithms",
            name="geometry",
            template="Vec3d",
        )
        collisionNode.addObject(
            "MechanicalObject",
            name="dofs",
            template="Vec3d",
            position=collision_positions,
        )
        for model in _collision_model_names(params["object_collision_models"]):
            collisionNode.addObject(
                model,
                group=1,
                simulated=True,
                moving=(contact_object == "soft"),
            )
        # soffice: superficie mappata sulla FEM; rigido: tutti i punti bloccati
        if contact_object == "soft":
            collisionNode.addObject("BarycentricMapping")
        else:
            collisionNode.addObject(
                "FixedProjectiveConstraint",
                name="FixedRigidSurface",
                indices=list(range(len(collision_positions))),
            )

        visualNode = contactNode.addChild("VisualModel")
        visualNode.addObject(
            "OglModel",
            name="Visual",
            src="@../CollisionModel/topology",
            color=[0.85, 0.35, 0.25, ghost_alpha if contact_object == "soft" else 1.0],
        )
        if contact_object == "soft":
            visualNode.addObject("BarycentricMapping")

    # 11. controller della molla in serie e del ciclo di prova
    controller = rootNode.addObject(
        TendonSpringController(
            name="ActuationController",
            root=rootNode,
            tube_meca=tubeMeca,
            fem_ff=femFF,
            cable=cableConstraint,
            params=params,
            tip_node_indices=tip_node_indices,
            contact_force_provider=params.get(
                "contact_force_provider",
                read_contact_force,
            ),
        )
    )

    # 12. solo GUI: vista amplificata, registrazione CSV e grafici in tempo reale
    if custom_params is None:
        if params.get("visual_amplification", 1.0) > 1.0:
            from modules.amplified_view import add_amplified_view
            soft_surface = ((collision_positions, collision_triangles)
                            if contact_object == "soft" else None)
            add_amplified_view(rootNode, params["visual_amplification"], obj_path, soft_surface)
        from modules.controller import HISTORY_FIELDS
        from modules.live_plot import CsvRecorder, LivePlotListener
        if params.get("record_csv", True):
            controller.listeners.append(CsvRecorder(HISTORY_FIELDS))
        if params.get("live_plot", True):
            controller.listeners.append(LivePlotListener(every=params["plot_every"],
                                                         python=params.get("live_plot_python")))

    return rootNode
