"""
modules/geometry_gen.py

Generazione delle mesh: tubo cavo (tetraedri per il FEM e superfici .obj per i modelli visivo
e di collisione), oggetto di contatto (griglia di esaedri e superficie triangolata) ed export
in formato VTK.
"""
import os
import itertools
import numpy as np


# mesh tetraedrica del tubo cavo (nodi e tetraedri) e superficie in .obj
def create_hollow_tube_meshes(
    obj_filepath,
    od=1.93,          # diametro esterno [mm]
    thickness=0.22,   # spessore di parete [mm]
    length=100.0,     # lunghezza totale [mm]
    n_x=30,           # suddivisioni assiali (se x_vals non è dato)
    n_theta=12,       # suddivisioni lungo la circonferenza
    x_vals=None,      # coordinate assiali degli anelli [mm], se date prevalgono su n_x e length
):
    if od <= 0 or thickness <= 0 or thickness >= od / 2:
        raise ValueError("Richiesti od > 0 e 0 < thickness < od/2")
    if x_vals is None:
        if length <= 0 or n_x < 1:
            raise ValueError("Richiesti length > 0 e n_x >= 1")
        x_vals = np.linspace(0.0, length, n_x + 1)
    else:
        x_vals = np.asarray(x_vals, dtype=float)
        if x_vals.ndim != 1 or x_vals.size < 2 or np.any(np.diff(x_vals) <= 0):
            raise ValueError("x_vals deve essere un vettore strettamente crescente (>= 2 valori)")
        n_x = x_vals.size - 1
        length = float(x_vals[-1] - x_vals[0])
    if n_theta < 3:
        raise ValueError("Richiesto n_theta >= 3")

    r_out = od / 2.0
    r_in = r_out - thickness
    theta_vals = np.linspace(0.0, 2.0 * np.pi, n_theta, endpoint=False)

    # per ogni anello e ogni angolo: nodo interno e nodo esterno
    fem_nodes = []
    for ix, x in enumerate(x_vals):
        for ith, th in enumerate(theta_vals):
            fem_nodes.append([float(x), float(r_in * np.cos(th)), float(r_in * np.sin(th))])
            fem_nodes.append([float(x), float(r_out * np.cos(th)), float(r_out * np.sin(th))])

    # indice del nodo (anello ix, angolo ith, interno/esterno)
    def node_idx(ix, ith, is_outer):
        return ix * (2 * n_theta) + (ith % n_theta) * 2 + (1 if is_outer else 0)

    nodes_np = np.array(fem_nodes)

    # ogni cella tra due anelli e due angoli è un esaedro con coordinate locali (a, b, r):
    # a = asse x, b = theta, r = interno/esterno. Viene divisa in 6 tetraedri con lo schema di
    # Freudenthal, uno per ogni permutazione degli assi lungo i cammini da (0,0,0) a (1,1,1)
    tetrahedra = []
    for ix in range(n_x):
        for ith in range(n_theta):
            def c(a, b, r):
                return node_idx(ix + a, ith + b, bool(r))

            for perm in itertools.permutations(range(3)):
                p = [0, 0, 0]
                path = [c(*p)]
                for ax in perm:
                    p[ax] = 1
                    path.append(c(*p))
                P = nodes_np[path]
                vol6 = np.dot(P[1] - P[0], np.cross(P[2] - P[0], P[3] - P[0]))
                if vol6 < 0:
                    path[1], path[2] = path[2], path[1]   # orientamento positivo
                tetrahedra.append([int(i) for i in path])

    output_dir = os.path.dirname(os.path.abspath(obj_filepath))
    os.makedirs(output_dir, exist_ok=True)
    # facce della superficie esterna e interna (indici .obj da 1)
    faces = []
    for ix in range(n_x):
        for ith in range(n_theta):
            i_next = (ith + 1) % n_theta
            v_in_1  = node_idx(ix,     ith,    False) + 1
            v_in_2  = node_idx(ix,     i_next, False) + 1
            v_in_3  = node_idx(ix + 1, i_next, False) + 1
            v_in_4  = node_idx(ix + 1, ith,    False) + 1

            v_out_1 = node_idx(ix,     ith,    True) + 1
            v_out_2 = node_idx(ix,     i_next, True) + 1
            v_out_3 = node_idx(ix + 1, i_next, True) + 1
            v_out_4 = node_idx(ix + 1, ith,    True) + 1

            faces.append((v_out_1, v_out_2, v_out_3))
            faces.append((v_out_1, v_out_3, v_out_4))
            faces.append((v_in_1, v_in_3, v_in_2))
            faces.append((v_in_1, v_in_4, v_in_3))

    # chiusura delle due estremità (corona circolare, il lume resta aperto)
    for ith in range(n_theta):
        i_next = (ith + 1) % n_theta
        start_inner = node_idx(0, ith, False) + 1
        start_inner_next = node_idx(0, i_next, False) + 1
        start_outer = node_idx(0, ith, True) + 1
        start_outer_next = node_idx(0, i_next, True) + 1
        faces.extend([
            (start_inner, start_inner_next, start_outer_next),
            (start_inner, start_outer_next, start_outer),
        ])

        end_inner = node_idx(n_x, ith, False) + 1
        end_inner_next = node_idx(n_x, i_next, False) + 1
        end_outer = node_idx(n_x, ith, True) + 1
        end_outer_next = node_idx(n_x, i_next, True) + 1
        faces.extend([
            (end_inner, end_outer_next, end_inner_next),
            (end_inner, end_outer, end_outer_next),
        ])

    with open(obj_filepath, "w") as f:
        f.write(f"# Hollow Tube Surface Mesh (Tetra) - OD={od}mm, t={thickness}mm, L={length}mm\n")
        for pt in fem_nodes:
            f.write(f"v {pt[0]:.6f} {pt[1]:.6f} {pt[2]:.6f}\n")
        for fc in faces:
            f.write(f"f {fc[0]} {fc[1]} {fc[2]}\n")

    return fem_nodes, tetrahedra


# coordinate assiali degli anelli: n_shaft elementi sull'albero, passo ~h_flex nel resto
def graded_x_vals(length_total, length_shaft, h_flex, n_shaft=4):
    if not 0 < length_shaft < length_total:
        raise ValueError("Richiesto 0 < length_shaft < length_total")
    if h_flex <= 0 or n_shaft < 1:
        raise ValueError("Richiesti h_flex > 0 e n_shaft >= 1")
    n_flex = max(1, int(round((length_total - length_shaft) / h_flex)))
    shaft = np.linspace(0.0, length_shaft, n_shaft + 1)
    flex = np.linspace(length_shaft, length_total, n_flex + 1)[1:]
    return np.concatenate((shaft, flex))


# superficie chiusa del tubo tra x_start e x_end con passo ~h, con i vertici sulla superficie
# della mesh FEM (stessi angoli); restituisce il numero di vertici
def write_surface_obj(obj_filepath, od, thickness, x_start, x_end, h, n_theta=12):
    n = max(1, int(round((x_end - x_start) / h)))
    nodes, _ = create_hollow_tube_meshes(
        obj_filepath, od=od, thickness=thickness, n_theta=n_theta,
        x_vals=np.linspace(x_start, x_end, n + 1))
    return len(nodes)


# coordinate in [lo, hi] con passo h_min vicino a focus, che cresce di un fattore growth fino
# a h_max; contiene sempre lo, hi e focus
def graded_axis(lo, hi, focus, h_min, h_max, growth=1.4):
    if not lo < hi or not 0 < h_min <= h_max or growth < 1.0:
        raise ValueError("Richiesti lo < hi, 0 < h_min <= h_max, growth >= 1")
    focus = min(max(focus, lo), hi)

    def side(length):
        pts, x, h = [], 0.0, h_min
        while length - x > 1e-12:
            step = min(h, h_max)
            if length - x - step < 0.5 * step:   # l'ultimo intervallo assorbe il residuo
                step = length - x
            x += step
            pts.append(x)
            h *= growth
        return pts

    right = [focus + d for d in side(hi - focus)]
    left = [focus - d for d in side(focus - lo)]
    xs = sorted(set([lo, hi, focus] + right + left))
    out = [xs[0]]
    for a, b in zip(xs[:-1], xs[1:]):          # nessun intervallo più largo di h_max
        n = int(np.ceil((b - a) / h_max - 1e-9))
        out.extend(np.linspace(a, b, n + 1)[1:].tolist())
    return np.clip(np.array(out, dtype=float), lo, hi)


# griglia strutturata di esaedri (anche con passo non uniforme); indice del nodo
# i + nx*(j + ny*k), vertici dell'esaedro nello stesso ordine di RegularGridTopology
def box_hexa_mesh(xs, ys, zs):
    nx, ny, nz = len(xs), len(ys), len(zs)
    X, Y, Z = np.meshgrid(xs, ys, zs, indexing="ij")
    pos = np.stack([X.ravel(order="F"), Y.ravel(order="F"), Z.ravel(order="F")], axis=1)

    def idx(i, j, k):
        return i + nx * (j + ny * k)

    hexa = []
    for k in range(nz - 1):
        for j in range(ny - 1):
            for i in range(nx - 1):
                hexa.append([idx(i, j, k), idx(i + 1, j, k), idx(i + 1, j + 1, k), idx(i, j + 1, k),
                             idx(i, j, k + 1), idx(i + 1, j, k + 1), idx(i + 1, j + 1, k + 1),
                             idx(i, j + 1, k + 1)])
    return pos.tolist(), hexa


# superficie triangolata delle 6 facce di una griglia strutturata (normali verso l'esterno);
# i vertici sono i nodi di bordo della griglia
def box_surface(xs, ys, zs):
    nx, ny, nz = len(xs), len(ys), len(zs)
    index, positions = {}, []
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                if i in (0, nx - 1) or j in (0, ny - 1) or k in (0, nz - 1):
                    index[(i, j, k)] = len(positions)
                    positions.append([float(xs[i]), float(ys[j]), float(zs[k])])
    tris = []

    def quad(c):
        a, b, cc, d = (index[x] for x in c)
        tris.extend([[a, b, cc], [a, cc, d]])

    for j in range(ny - 1):
        for k in range(nz - 1):
            quad([(0, j, k), (0, j, k + 1), (0, j + 1, k + 1), (0, j + 1, k)])
            quad([(nx - 1, j, k), (nx - 1, j + 1, k), (nx - 1, j + 1, k + 1), (nx - 1, j, k + 1)])
    for i in range(nx - 1):
        for k in range(nz - 1):
            quad([(i, 0, k), (i + 1, 0, k), (i + 1, 0, k + 1), (i, 0, k + 1)])
            quad([(i, ny - 1, k), (i, ny - 1, k + 1), (i + 1, ny - 1, k + 1), (i + 1, ny - 1, k)])
    for i in range(nx - 1):
        for j in range(ny - 1):
            quad([(i, j, 0), (i, j + 1, 0), (i + 1, j + 1, 0), (i + 1, j, 0)])
            quad([(i, j, nz - 1), (i + 1, j, nz - 1), (i + 1, j + 1, nz - 1), (i, j + 1, nz - 1)])
    return positions, tris


# numero di tetraedri invertiti o degeneri, volume minimo e volume totale [mm^3]
def check_tet_mesh(nodes, tetrahedra):
    n = np.array(nodes)
    t = np.array(tetrahedra)
    v = np.einsum("ij,ij->i", n[t[:, 1]] - n[t[:, 0]],
                  np.cross(n[t[:, 2]] - n[t[:, 0]], n[t[:, 3]] - n[t[:, 0]])) / 6.0
    return int((v <= 1e-12).sum()), float(v.min()), float(v.sum())



# mesh del tubo usate dalla scena:
#   tube_fem_surface.obj   superficie della mesh FEM graduata
#   tube_surface.obj       modello visivo (passo visual_h)
#   tube_collision.obj     modello di collisione (solo il tratto vicino alla punta)
# restituisce nodi FEM, tetraedri e percorsi dei file
def build_tube_meshes(params, mesh_dir):
    fem_obj_path = os.path.join(mesh_dir, "tube_fem_surface.obj")
    obj_path = os.path.join(mesh_dir, "tube_surface.obj")
    collision_obj_path = os.path.join(mesh_dir, "tube_collision.obj")
    n_theta = int(params["mesh_n_theta"])
    fem_nodes, tetrahedra = create_hollow_tube_meshes(
        obj_filepath=fem_obj_path,
        od=params["od"],
        thickness=params["thickness"],
        n_theta=n_theta,
        x_vals=graded_x_vals(params["length_total"], params["length_shaft"],
                             params["mesh_h_flex"], int(params["mesh_n_shaft"])),
    )
    write_surface_obj(obj_path, params["od"], params["thickness"],
                      0.0, params["length_total"], params["visual_h"], n_theta)
    collision_x_start = max(
        params["length_shaft"],
        params["length_total"] - params["obj_size"] - params["collision_margin"],
    )
    write_surface_obj(collision_obj_path, params["od"], params["thickness"],
                      collision_x_start, params["length_total"], params["collision_h"], n_theta)
    return fem_nodes, tetrahedra, {"fem_surface": fem_obj_path, "visual": obj_path,
                                   "collision": collision_obj_path}


# coordinate dei nodi dell'oggetto di contatto (cubo di lato obj_size davanti alla punta):
# soffice "graded" con passo soft_h_min al contatto che cresce fino a soft_h_max, altrimenti
# griglia uniforme di obj_grid_n nodi per lato
def contact_object_axes(params):
    L = params["length_total"]
    cube_size = float(params["obj_size"])
    grid_n = int(params["obj_grid_n"])
    x_min, x_max = L - cube_size, L + float(params["obj_overlap_x"])
    y_min = params["od"] / 2.0 + float(params["obj_gap"])
    y_max = y_min + cube_size
    z_min, z_max = -cube_size / 2.0, cube_size / 2.0
    graded = params["contact_object"] == "soft" and params["soft_mesh"] == "graded"
    if graded:
        h_args = (params["soft_h_min"], params["soft_h_max"], params["soft_growth"])
        xs = graded_axis(x_min, x_max, L, *h_args)
        ys = graded_axis(y_min, y_max, y_min, *h_args)
        zs = graded_axis(z_min, z_max, 0.0, *h_args)
    else:
        xs = np.linspace(x_min, x_max, grid_n)
        ys = np.linspace(y_min, y_max, grid_n)
        zs = np.linspace(z_min, z_max, grid_n)
    return xs, ys, zs, (x_min, x_max, y_min, y_max, z_min, z_max), graded


# mesh volumetrica di tetraedri in formato VTK legacy
def write_vtk_tetra(path, nodes, tetrahedra):
    _write_vtk(path, nodes, tetrahedra, cell_type=10)


# mesh volumetrica di esaedri in formato VTK legacy
def write_vtk_hexa(path, nodes, hexahedra):
    _write_vtk(path, nodes, hexahedra, cell_type=12)


# scrittura del file VTK (cell_type 10 = tetraedro, 12 = esaedro)
def _write_vtk(path, nodes, cells, cell_type):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    nodes = np.asarray(nodes, dtype=float)
    cells = np.asarray(cells, dtype=int)
    n_per = cells.shape[1]
    with open(path, "w", encoding="ascii") as f:
        f.write("# vtk DataFile Version 3.0\nProgetto 5 - mesh volumetrica\nASCII\n"
                "DATASET UNSTRUCTURED_GRID\n")
        f.write(f"POINTS {len(nodes)} double\n")
        for p in nodes:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
        f.write(f"CELLS {len(cells)} {len(cells) * (n_per + 1)}\n")
        for c in cells:
            f.write(f"{n_per} " + " ".join(str(int(i)) for i in c) + "\n")
        f.write(f"CELL_TYPES {len(cells)}\n" + f"{cell_type}\n" * len(cells))

if __name__ == "__main__":
    print("Per generare le mesh del progetto usare: python3 tools/generate_meshes.py")
