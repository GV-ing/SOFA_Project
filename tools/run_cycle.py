#!/usr/bin/env python3
"""
tools/run_cycle.py

Casi nominali del Progetto 5, con gli stessi protocolli del Monte Carlo (modules/protocols.py).
Per ogni caso salva in --outdir la serie temporale (<caso>.csv) e le metriche (<caso>.json).

    --free      cicli liberi per H: fixed, switched (nei due ordini), direct
    --contact   prove di contatto su oggetto rigido e soffice: fixed e switched

Ordini della molla a due livelli (--kfree): low = 0.6 libero / 4.8 contatto,
high = 4.8 libero / 0.6 contatto.

Uso (cartella SOFA_Project, dopo source env.sh):
    python3.12 -u tools/run_cycle.py --free
    python3.12 -u tools/run_cycle.py --contact --objects rigid
    python3.12 -u tools/run_cycle.py --contact --objects soft --configs switched --kfree high
"""

import argparse
import json
import os
import sys

# cartella del progetto nel path per importare modules
PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_DIR)

from modules.protocols import CONFIGS, contact_test, free_cycle, save_series


# coppie (nome del caso, parametri della molla); switched viene ripetuta per ogni ordine
def spring_cases(configs, kfree):
    for name in configs:
        if name == "switched":
            for level in kfree:
                yield f"switched_{level}", {**CONFIGS[name], "k_free_level": level}
        else:
            yield name, dict(CONFIGS[name])


# salva serie (.csv) e metriche (.json) di un caso
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

    # cicli liberi: H di ogni molla e rapporto con l'attuazione diretta
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

    # prove di contatto (l'attuazione diretta è esclusa)
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
