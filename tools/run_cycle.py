#!/usr/bin/env python3
# tools/run_cycle.py  -  SOFA v26.06 / SofaPython3 / Python 3.12
"""
Esegue i casi NOMINALI del Progetto 5 (stessi protocolli del Monte Carlo, modules/protocols.py)
e ne stampa le metriche.

Casi:
  free     ciclo libero senza oggetto: carico fino a T = 10 N o u = 15 mm, poi scarico -> H [%]
           molle: fixed (1.2), switched (0.6/4.8 nei due ordini), direct (k = 50, 0.1 mm/s)
  contact  avvicinamento a contact_rate -> rilevamento a 5 mN -> regolazione a 5 mN +/-10 %
           -> 1 s in banda a motore fermo -> finestra di 5 s
           su oggetto rigido e soffice, molle fixed e switched     -> stabilita' e banda

Ordini della molla a due livelli (--kfree): low = 0.6 libero / 4.8 contatto,
high = 4.8 libero / 0.6 contatto. Per ogni caso salva in --outdir la serie temporale
(<caso>.csv, una riga per passo) e le metriche (<caso>.json).

Esecuzione (cartella SOFA_Project, source env.sh):
    python3.12 -u tools/run_cycle.py --free                         # 4 cicli liberi
    python3.12 -u tools/run_cycle.py --contact --objects rigid      # 3 prove sul rigido
    python3.12 -u tools/run_cycle.py --contact --objects soft --configs switched --kfree high
"""

import argparse
import json
import os
import sys

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_DIR)

from modules.protocols import CONFIGS, contact_test, free_cycle, save_series  # noqa: E402


def spring_cases(configs, kfree):
    """(nome del caso, override della molla); la switched si ripete per ogni ordine."""
    for name in configs:
        if name == "switched":
            for level in kfree:
                yield f"switched_{level}", {**CONFIGS[name], "k_free_level": level}
        else:
            yield name, dict(CONFIGS[name])


def save(outdir, case, metrics, history):
    save_series(os.path.join(outdir, f"{case}.csv"), history)
    with open(os.path.join(outdir, f"{case}.json"), "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--free", action="store_true", help="Cicli liberi per H")
    ap.add_argument("--contact", action="store_true", help="Prove di contatto")
    ap.add_argument("--configs", nargs="+", default=list(CONFIGS), choices=list(CONFIGS))
    ap.add_argument("--kfree", nargs="+", default=["low", "high"], choices=["low", "high"],
                    help="Ordini della molla a due livelli da provare")
    ap.add_argument("--objects", nargs="+", default=["rigid", "soft"], choices=["rigid", "soft"])
    ap.add_argument("--outdir", default=os.path.join(PROJECT_DIR, "results", "nominali"))
    a = ap.parse_args()
    if not (a.free or a.contact):
        ap.error("specificare --free e/o --contact")

    if a.free:
        print("\n=== Cicli liberi (H) ===", flush=True)
        H = {}
        for case, spring in spring_cases(a.configs, a.kfree):
            m, h = free_cycle(spring)
            save(a.outdir, f"free_{case}", m, h)
            H[case] = m["H_pct"]
            print(f"  {case:14s}: H = {m['H_pct']:6.2f} %  dY_max = {m['dY_max_mm']:.4f} mm  "
                  f"inversione a u = {m['u_max_mm']:.3f} mm, T = {m['T_peak_N']:.2f} N  "
                  f"scarto max {m['gap_max_mm']:.4f} mm a u = {m['u_at_gap_max_mm']:.3f} mm\n"
                  f"  {'':14s}  {m['steps']} passi, {m['elapsed_s']:.0f} s "
                  f"({m['elapsed_s'] / m['steps'] * 1e3:.0f} ms/passo)", flush=True)
        if "direct" in H:
            for case, value in H.items():
                if case != "direct":
                    print(f"  H_{case} / H_direct = {value / H['direct']:.3f}")

    if a.contact:
        print("\n=== Prove di contatto (5 mN +/-10 %, finestra di 5 s) ===", flush=True)
        for obj in a.objects:
            for case, spring in spring_cases([c for c in a.configs if c != "direct"], a.kfree):
                m, h = contact_test(obj, spring)
                save(a.outdir, f"contact_{obj}_{case}", m, h)
                print(f"  {obj:5s} {case:14s}: rilevamento a t = {m['t_detect_s']:.2f} s, "
                      f"F = {m['F_detect_N'] * 1e3:.2f} mN; T {m['T_detect_N']:.2f} -> "
                      f"{m['T_after_switch_N']:.2f} N (k = {m['k_contact']:.1f}); picco F "
                      f"{m['F_peak_N'] * 1e3:.2f} mN; regolazione {m['regulation_s']:.1f} s\n"
                      f"  {'':5s} {'':14s}  finestra: F media {m['F_mean_N'] * 1e3:.2f} mN "
                      f"[{m['F_min_N'] * 1e3:.2f}, {m['F_max_N'] * 1e3:.2f}], p-p/media "
                      f"{m['ptp_over_mean']:.3f}, perso {m['contact_lost']} -> stabile "
                      f"{m['stable']}, in banda 4.5-5.5 {m['in_band']}  (assestata {m['settled']}, "
                      f"cambi di stato {m['contact_changes']})\n"
                      f"  {'':5s} {'':14s}  {m['steps']} passi, {m['elapsed_s']:.0f} s "
                      f"({m['elapsed_s'] / m['steps'] * 1e3:.0f} ms/passo)", flush=True)
    print(f"\nSerie (.csv) e metriche (.json) in {a.outdir}")


if __name__ == "__main__":
    main()
