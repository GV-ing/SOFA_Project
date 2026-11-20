"""
modules/contact_force.py

Lettura della forza normale di contatto dalle constraintForces del constraint solver.

Con FreeMotionAnimationLoop i valori dei vincoli sono impulsi [N*s] (forza = impulso / dt).
Il CableConstraint occupa una riga, con valore uguale all'impulso imposto T*dt; ogni punto di
contatto con FrictionContactConstraint occupa tre righe (n, t1, t2) e la normale è la prima.
"""

import numpy as np

# nomi dei componenti in main_scene.py
SOLVER_NAME = "ConstraintSolver"
CABLE_PATH = ("NitinolTube", "Tendon", "tendonCable")


# CableConstraint della scena (None se non c'è)
def _find_cable(root):
    node = root
    for name in CABLE_PATH[:-1]:
        node = node.getChild(name) if node is not None else None
    return node.getObject(CABLE_PATH[-1]) if node is not None else None


# indice della riga del cavo: quella uguale all'impulso imposto, altrimenti l'ultima
def _cable_row(forces, cable_impulse):
    if cable_impulse is not None and abs(cable_impulse) > 0.0:
        diff = np.abs(forces - cable_impulse)
        idx = int(np.argmin(diff))
        if diff[idx] <= 1e-9 + 1e-6 * abs(cable_impulse):
            return idx
    return forces.size - 1


# impulsi normali di contatto [N*s], uno per punto di contatto
def contact_normal_impulses(root):
    solver = root.getObject(SOLVER_NAME)
    if solver is None:
        raise RuntimeError(f"Constraint solver '{SOLVER_NAME}' non trovato")
    forces = np.asarray(solver.constraintForces.value, dtype=float).ravel()
    if forces.size == 0:
        return forces
    if not np.all(np.isfinite(forces)):
        raise ValueError("constraintForces contiene valori non finiti")

    # tolta la riga del cavo, restano blocchi di tre righe per punto di contatto
    cable = _find_cable(root)
    has_cable = cable is not None
    if has_cable:
        impulse = float(np.ravel(cable.value.value)[0])
        forces = np.delete(forces, _cable_row(forces, impulse))
    if forces.size % 3 != 0:
        raise RuntimeError(
            f"Righe di contatto non multiple di 3 ({forces.size}, cavo "
            f"{'escluso' if has_cable else 'assente'}): verificare il tipo di risposta"
        )
    return forces[0::3]


# forza normale di contatto totale [N]: somma degli impulsi normali / dt
def read_contact_force(root):
    dt = float(root.dt.value)
    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError(f"dt non valido: {dt}")
    normals = contact_normal_impulses(root)
    if normals.size == 0:
        return 0.0
    return float(abs(np.sum(normals)) / dt)
