"""
modules/protocols.py

Protocolli di prova eseguiti in SOFA senza GUI, usati da tools/run_cycle.py e dal Monte Carlo
(modules/mc_worker.py).

    free_cycle      ciclo libero senza oggetto: carico a auto_rate fino a T = auto_T_reverse
                    o u = u_max, poi scarico fino a u = 0. Metrica: H [%]
    contact_test    avvicinamento a contact_rate, rilevamento a 5 mN (k commuta nella
                    config. b), regolazione a 5 mN +/-10 %, contact_settle_s in banda a
                    motore fermo, finestra di hold_duration s con regolatore attivo.
                    Metriche: stabilità e permanenza nella banda
"""

import csv
import os
import time

import numpy as np

from modules.config import NOMINAL_PARAMS, SPRING_CONFIGS
from modules.metrics import contact_stability, hysteresis_H

CONFIGS = SPRING_CONFIGS


# salva la cronologia del controller in CSV (una riga per passo)
def save_series(path, history):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    keys = list(history)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(keys)
        w.writerows(zip(*(history[k] for k in keys)))
    return path


# costruisce la scena, esegue un ciclo autonomo e restituisce
# (root, controller, passi, tempo di calcolo, timeout)
def simulate(overrides, stop_after_hold, max_time=120.0):
    import Sofa.Core
    import Sofa.Simulation

    from main_scene import createScene

    p = NOMINAL_PARAMS.copy()
    p.update(live_plot=False)
    p.update(overrides)
    root = Sofa.Core.Node("root")
    createScene(root, p)
    Sofa.Simulation.init(root)
    ctrl = root.getObject("ActuationController")
    dt = float(root.dt.value)
    ctrl.start_autonomous_cycle(repeat=False)
    seen_hold, n = False, 0
    t0 = time.perf_counter()
    while ctrl.phase != "done" and n * dt < max_time:
        Sofa.Simulation.animate(root, dt)
        n += 1
        seen_hold |= ctrl.phase == "hold"
        if stop_after_hold and seen_hold and ctrl.phase == "unload":
            break
    timed_out = n * dt >= max_time
    return root, ctrl, n, time.perf_counter() - t0, timed_out


# ciclo libero: restituisce (metriche, cronologia)
def free_cycle(overrides):
    root, ctrl, n, elapsed, timed_out = simulate(
        {"contact_object": "none", **overrides}, stop_after_hold=False)
    h = {k: list(v) for k, v in ctrl.history.items()}
    out = hysteresis_H(h["u"], h["dY"], h["phase"])
    out.update(T_peak_N=float(max(h["T"])), steps=n, elapsed_s=elapsed, timed_out=timed_out)
    return out, h


# prova di contatto su obj ("rigid" o "soft"): restituisce (metriche, cronologia)
def contact_test(obj, overrides):
    P = {**NOMINAL_PARAMS, **overrides}
    o = {**overrides, "contact_object": obj, "auto_rate": P["contact_rate"]}
    root, ctrl, n, elapsed, timed_out = simulate(o, stop_after_hold=True)
    h = {k: list(v) for k, v in ctrl.history.items()}
    t, F, T = np.array(h["t"]), np.array(h["F_contact"]), np.array(h["T"])
    phase, contact = np.array(h["phase"]), np.array(h["contact"])
    band = (P["force_ref"] * (1.0 - P["force_band"]), P["force_ref"] * (1.0 + P["force_band"]))

    hold = np.flatnonzero(phase == "hold")
    if hold.size >= 2:
        out = contact_stability(t, F, phase, duration=P["hold_duration"],
                                ptp_max=P["stability_ptp_max"],
                                loss_force=P["contact_loss_force"], band=band)
    else:   # la regolazione non si è assestata entro max_time: prova non superata
        out = {"F_mean_N": float("nan"), "F_min_N": float("nan"), "F_max_N": float("nan"),
               "F_ptp_N": float("nan"), "ptp_over_mean": float("nan"), "contact_lost": False,
               "hold_duration_s": 0.0, "window_complete": False, "in_band": False,
               "stable": False}
    out["settled"] = bool(hold.size >= 2)

    # valori al rilevamento e durante la regolazione
    det = np.flatnonzero(contact == 1)
    if det.size:
        i = int(det[0])
        j = min(i + 1, len(t) - 1)            # primo passo con la k di contatto
        end = int(hold[-1]) + 1 if hold.size else len(t)
        out.update(t_detect_s=float(t[i]), F_detect_N=float(F[i]), T_detect_N=float(T[i]),
                   T_after_switch_N=float(T[j]), k_contact=float(h["k"][j]),
                   F_peak_N=float(F[i:end].max()),
                   regulation_s=float(t[hold[0]] - t[i]) if hold.size else float("nan"))
    else:
        out.update(t_detect_s=float("nan"), F_detect_N=float("nan"), T_detect_N=float("nan"),
                   T_after_switch_N=float("nan"), k_contact=float("nan"),
                   F_peak_N=float("nan"), regulation_s=float("nan"))
    out.update(contact_changes=int(np.count_nonzero(np.diff(contact))), steps=n,
               elapsed_s=elapsed, timed_out=timed_out,
               contact_read_errors=int(ctrl.contact_read_errors))
    return out, h
