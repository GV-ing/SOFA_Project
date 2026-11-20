#!/usr/bin/env python3
"""
tools/make_figures.py

Figure del Progetto 5 a partire da una cartella di campagna Monte Carlo (results/mc_finale).

    fig_tornado.png             coefficienti della regressione di H (punti di H per +20 %)
    fig_H_realizzazioni.png     H per realizzazione, nominale e soglia 1.3 x nominale
    fig_forze_contatto.png      forza nella finestra di 5 s (media, min-max) e banda 4.5-5.5 mN
    fig_isteresi_nominale.png   cicli dY(u) nominali
    fig_contatto_nominale.png   F(t) e k(t) nominali

Le serie temporali si prendono da <campagna>/series/<caso>.csv; se mancano, da quelle dei casi
nominali di tools/run_cycle.py (results/nominali/). Da lì viene anche l'ordine della molla (b)
non usato nel Monte Carlo, mostrato solo nel ciclo nominale.

Uso (senza SOFA):
    env -u PYTHONPATH python3 tools/make_figures.py --campaign results/mc_finale --out results/figures
"""

import argparse
import csv
import json
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NOMINALI = os.path.join(PROJECT_DIR, "results", "nominali")
sys.path.insert(0, PROJECT_DIR)
from modules.config import NOMINAL_PARAMS

# colori delle configurazioni e degli elementi dei grafici
C_FIXED, C_SWITCH, C_OTHER, C_DIRECT = "#2a78d6", "#eb6834", "#1baf7a", "#52514e"
INK, INK2, GRID, BAND = "#0b0b0b", "#52514e", "#e4e3df", "#9c9b95"
# etichette delle configurazioni e dei parametri incerti
ORDER = {"low": "0.6 libero / 4.8 in contatto", "high": "4.8 libero / 0.6 in contatto"}
LABEL = {"fixed": "(a) molla fissa k = 1.2 N/mm", "switched": "(b) due livelli",
         "other": "(b) ordine non scelto", "direct": "attuazione diretta k = 50 N/mm"}
COLOR = {"fixed": C_FIXED, "switched": C_SWITCH, "other": C_OTHER, "direct": C_DIRECT}
PARAM = {"E": "E (carico/scarico)", "Fh_mat": "F_h materiale", "Fh_spring": "F_h molla",
         "c": "smorzamento c", "k": "rigidezza k", "yt": "offset tendine y_t"}


# stile comune delle figure
def style():
    plt.rcParams.update({
        "figure.dpi": 110, "savefig.dpi": 200, "font.size": 9.5,
        "axes.edgecolor": INK2, "axes.labelcolor": INK, "axes.titlesize": 10.5,
        "axes.titleweight": "bold", "axes.spines.top": False, "axes.spines.right": False,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
        "xtick.color": INK2, "ytick.color": INK2, "legend.frameon": False,
        "lines.linewidth": 2.0,
    })


# legge summary, realizzazioni e risultati dei casi della campagna
def load(campaign):
    with open(os.path.join(campaign, "summary.json"), encoding="utf-8") as f:
        summary = json.load(f)
    with open(os.path.join(campaign, "realizations.json"), encoding="utf-8") as f:
        reals = json.load(f)["realizations"]
    latest = {}
    with open(os.path.join(campaign, "cases.jsonl"), encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    r = json.loads(line)
                except json.JSONDecodeError:
                    continue
                latest[r["case_id"]] = r          # se un caso compare più volte vale l'ultima riga
    return summary, reals, latest


# metrica di un caso (None se il caso manca o è fallito)
def metric(latest, real, cfg, test, key):
    r = latest.get(f"{real}_{cfg}_{test}")
    return r["metrics"].get(key) if r and "metrics" in r else None


# diagramma tornado delle due configurazioni
def fig_tornado(summary, out):
    fig, axes = plt.subplots(1, 2, figsize=(9.2, 3.4), sharex=True)
    for ax, cfg in zip(axes, ("fixed", "switched")):
        t = summary["configs"][cfg]["tornado"]
        rows = sorted(t["coefficients"], key=lambda r: abs(r["coef_pct_per_20pct"]))
        names = [PARAM[r["parameter"]] for r in rows]
        vals = [r["coef_pct_per_20pct"] for r in rows]
        ax.barh(names, vals, color=COLOR[cfg], height=0.55, edgecolor="white", linewidth=1)
        ax.axvline(0, color=INK2, linewidth=1)
        for i, v in enumerate(vals):
            if abs(v) >= 0.3:
                ax.text(v + (0.06 if v > 0 else -0.06), i, f"{v:+.2f}", va="center",
                        ha="left" if v > 0 else "right", color=INK, fontsize=8.5)
        ax.set_title(f"{LABEL[cfg]}\nR² = {t['r2']:.3f}", loc="left")
        ax.grid(axis="y", visible=False)
        ax.set_xlabel("variazione di H [punti %] per +20% del parametro")
    lim = max(abs(r["coef_pct_per_20pct"]) for c in ("fixed", "switched")
              for r in summary["configs"][c]["tornado"]["coefficients"]) * 1.35
    axes[0].set_xlim(-lim, lim)
    fig.suptitle("Diagramma tornado: regressione lineare di H sulle perturbazioni normalizzate",
                 x=0.01, ha="left", fontsize=11, fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_tornado.png"))
    plt.close(fig)


# H per realizzazione, con nominale, soglia 1.3 x nominale e attuazione diretta
def fig_H(summary, reals, latest, out):
    ids = [r["realization"] for r in reals]
    x = np.arange(len(ids))
    fig, ax = plt.subplots(figsize=(9.2, 4.2))
    for cfg, dx in (("fixed", -0.12), ("switched", 0.12)):
        c = summary["configs"][cfg]
        H = [metric(latest, i, cfg, "free", "H_pct") for i in ids]
        nom = c["H_nominal_pct"]
        ax.axhline(nom, color=COLOR[cfg], linewidth=1.2, alpha=0.9)
        ax.axhline(1.3 * nom, color=COLOR[cfg], linewidth=1.2, linestyle=(0, (4, 3)), alpha=0.9)
        ax.scatter(x + dx, H, s=34, color=COLOR[cfg], edgecolor="white", linewidth=1, zorder=3,
                   label=f"{LABEL[cfg]}: media {c['H_mean_pct']:.1f} ± {c['H_std_pct']:.1f} %, "
                         f"robustezza {c['robustness_index']:.2f}")
        ax.text(len(ids) - 0.4, 1.3 * nom, f"1.3 × nominale {'(a)' if cfg == 'fixed' else '(b)'}", color=INK2,
                fontsize=8, va="bottom", ha="right")
    Hd = summary.get("H_direct_nominal_pct")
    if Hd:
        ax.axhline(Hd, color=C_DIRECT, linewidth=1.2, linestyle=(0, (1, 2)))
        ax.text(len(ids) - 0.4, Hd, f"attuazione diretta (nominale) {Hd:.1f} %", color=INK2,
                fontsize=8, va="bottom", ha="right")
    ax.set_xticks(x, ids, rotation=0, fontsize=8)
    ax.set_xlim(-0.6, len(ids) - 0.4)
    ax.set_ylabel("H [%]")
    ax.set_title("Isteresi H nelle 20 realizzazioni (linea continua: nominale; tratteggio: target)",
                 loc="left")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=1, fontsize=8.5)
    ax.grid(axis="x", visible=False)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_H_realizzazioni.png"), bbox_inches="tight")
    plt.close(fig)


# forza di contatto nella finestra (media e min-max) per realizzazione; cerchiate le prove
# instabili o fuori banda
def fig_forces(reals, latest, out):
    ids = [r["realization"] for r in reals]
    x = np.arange(len(ids))
    ref, b = NOMINAL_PARAMS["force_ref"] * 1e3, NOMINAL_PARAMS["force_band"]
    lo_b, hi_b = ref * (1 - b), ref * (1 + b)
    fig, axes = plt.subplots(1, 2, figsize=(9.2, 3.6), sharey=True)
    lo, hi = lo_b - 0.4, hi_b + 0.4
    for ax, obj, title in ((axes[0], "rigid", "Oggetto rigido"),
                           (axes[1], "soft", "Oggetto soffice (E = 5 kPa)")):
        ax.axhspan(lo_b, hi_b, color=BAND, alpha=0.18, linewidth=0)
        for cfg, dx in (("fixed", -0.12), ("switched", 0.12)):
            get = lambda key: np.array([1e3 * (metric(latest, i, cfg, obj, key) or np.nan)
                                        for i in ids])
            Fm, Fmin, Fmax = get("F_mean_N"), get("F_min_N"), get("F_max_N")
            ax.errorbar(x + dx, Fm, yerr=[Fm - Fmin, Fmax - Fm], fmt="o", ms=5,
                        color=COLOR[cfg], elinewidth=1.4, capsize=0, label=LABEL[cfg], zorder=3)
            bad = [j for j, i in enumerate(ids)
                   if metric(latest, i, cfg, obj, "stable") is False
                   or metric(latest, i, cfg, obj, "in_band") is False]
            if bad:
                ax.scatter(x[bad] + dx, Fm[bad], s=90, facecolor="none", edgecolor=INK,
                           linewidth=1.2, zorder=4)
            if np.isfinite(Fmin).any():
                lo, hi = min(lo, np.nanmin(Fmin) - 0.2), max(hi, np.nanmax(Fmax) + 0.2)
        ax.text(0.98, hi_b, f"banda {lo_b:g}-{hi_b:g} mN ", transform=ax.get_yaxis_transform(),
                color=INK2, fontsize=8, va="bottom", ha="right")
        ax.set_xticks(x[::2], ids[::2], fontsize=8)
        ax.set_title(title, loc="left")
        ax.grid(axis="x", visible=False)
    axes[0].set_ylim(lo, hi)
    axes[0].set_ylabel("F nella finestra di 5 s [mN]\n(punto: media, barra: min-max)")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.suptitle(f"Prova di contatto: forza mantenuta a {ref:g} mN +/-{b * 100:g} % "
                 "(cerchio: prova instabile o fuori banda)", x=0.01, ha="left", fontsize=10.5,
                 fontweight="bold")
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    fig.legend(handles, labels, loc="lower center", ncol=2, fontsize=8.5)
    fig.savefig(os.path.join(out, "fig_forze_contatto.png"), bbox_inches="tight")
    plt.close(fig)


# legge una serie temporale CSV (phase come testo, il resto come numeri)
def read_series(path):
    with open(path, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    out = {}
    for k in rows[0]:
        if k == "phase":
            out[k] = np.array([r[k] for r in rows])
        else:
            out[k] = np.array([float(r[k]) for r in rows])
    return out


# serie della campagna (case_id) o, se manca, quella di fallback
def find_series(campaign, case_id, fallback=None):
    p = os.path.join(campaign, "series", f"{case_id}.csv") if case_id else None
    if p and os.path.exists(p):
        return read_series(p)
    if fallback and os.path.exists(fallback):
        return read_series(fallback)
    return None


# H del ciclo libero nominale da results/nominali/free_<case>.json, se esiste
def _nominal_H(case):
    p = os.path.join(NOMINALI, f"free_{case}.json")
    if not os.path.exists(p):
        return None
    with open(p, encoding="utf-8") as f:
        return json.load(f).get("H_pct")


# cicli dY(u) nominali: molla fissa, due livelli nei due ordini, attuazione diretta
def fig_hysteresis(summary, campaign, out, level):
    other = "low" if level == "high" else "high"
    panels = (("fixed", "nominal_fixed_free", "fixed", summary["configs"]["fixed"]["H_nominal_pct"]),
              ("switched", "nominal_switched_free", f"switched_{level}",
               summary["configs"]["switched"]["H_nominal_pct"]),
              ("other", None, f"switched_{other}", _nominal_H(f"switched_{other}")),
              ("direct", "nominal_direct_free", "direct", summary["H_direct_nominal_pct"]))
    fig, axes = plt.subplots(1, 4, figsize=(13, 3.4))
    found = False
    for ax, (key, case_id, nominal_case, H) in zip(axes, panels):
        s = find_series(campaign, case_id, os.path.join(NOMINALI, f"free_{nominal_case}.csv"))
        if s is None or H is None:
            ax.set_visible(False)
            continue
        found = True
        load_ = s["phase"] == "load"
        unl = s["phase"] == "unload"
        ax.plot(s["u"][load_], 1e3 * s["dY"][load_], color=COLOR[key], label="carico")
        ax.plot(s["u"][unl], 1e3 * s["dY"][unl], color=COLOR[key], linestyle=(0, (4, 2)),
                label="scarico")
        name = LABEL[key]
        if key in ("switched", "other"):
            name = f"(b) {ORDER[level if key == 'switched' else other]}"
        ax.set_title(f"{name}\nH = {H:.1f} %", loc="left")
        ax.set_xlabel("comando motore u [mm]")
        ax.set_ylabel("deflessione punta ΔY [µm]")
        ax.legend(loc="upper left", fontsize=8)
    if found:
        fig.suptitle("Cicli di carico e scarico nominali in moto libero (inversione a T = 10 N "
                     "o u = 15 mm)", x=0.01, ha="left", fontsize=11, fontweight="bold")
        fig.tight_layout()
        fig.savefig(os.path.join(out, "fig_isteresi_nominale.png"))
    plt.close(fig)
    return found


# F(t) e k(t) delle prove di contatto nominali su rigido e soffice
def fig_contact_time(campaign, out, level):
    ref, b = NOMINAL_PARAMS["force_ref"] * 1e3, NOMINAL_PARAMS["force_band"]
    fig, axes = plt.subplots(2, 2, figsize=(9.2, 5.2), sharex="col")
    found = False
    for col, obj in enumerate(("rigid", "soft")):
        for cfg, case in (("fixed", "fixed"), ("switched", f"switched_{level}")):
            s = find_series(campaign, f"nominal_{cfg}_{obj}",
                            os.path.join(NOMINALI, f"contact_{obj}_{case}.csv"))
            if s is None:
                continue
            found = True
            axes[0, col].plot(s["t"], 1e3 * s["F_contact"], color=COLOR[cfg], label=LABEL[cfg])
            axes[1, col].plot(s["t"], s["k"], color=COLOR[cfg], drawstyle="steps-post")
        axes[0, col].axhspan(ref * (1 - b), ref * (1 + b), color=BAND, alpha=0.18, linewidth=0)
        axes[0, col].set_title("Oggetto rigido" if obj == "rigid" else "Oggetto soffice",
                               loc="left")
        axes[0, col].set_ylabel("forza di contatto [mN]")
        axes[1, col].set_ylabel("rigidezza molla k [N/mm]")
        axes[1, col].set_xlabel("tempo [s]")
    if found:
        axes[0, 0].legend(loc="upper right", fontsize=8)
        fig.suptitle(f"Prove di contatto nominali: rilevamento a 5 mN, k commuta, forza regolata "
                     f"a {ref:g} mN +/-{b * 100:g} % (fascia grigia)", x=0.01, ha="left",
                     fontsize=10.5, fontweight="bold")
        fig.tight_layout()
        fig.savefig(os.path.join(out, "fig_contatto_nominale.png"))
    plt.close(fig)
    return found


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--campaign", default=os.path.join(PROJECT_DIR, "results", "mc_finale"))
    ap.add_argument("--out", default=os.path.join(PROJECT_DIR, "results", "figures"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    style()
    summary, reals, latest = load(a.campaign)
    level = summary.get("k_free_level", NOMINAL_PARAMS["k_free_level"])
    LABEL["switched"] = f"(b) due livelli, {ORDER[level]}"
    fig_tornado(summary, a.out)
    fig_H(summary, reals, latest, a.out)
    fig_forces(reals, latest, a.out)
    h = fig_hysteresis(summary, a.campaign, a.out, level)
    c = fig_contact_time(a.campaign, a.out, level)
    print(f"Figure in {a.out}" + ("" if h else "  (cicli di isteresi: serie mancanti)")
          + ("" if c else "  (contatto nel tempo: serie mancanti)"))


if __name__ == "__main__":
    main()