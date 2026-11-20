"""
modules/mc_worker.py

Esecuzione di un singolo caso del Monte Carlo. Ogni caso gira in un processo nuovo, così SOFA
parte pulito e la memoria viene liberata alla fine.
"""

import os
import traceback


# task: dizionario con case_id, realization, config, test, overrides, series_dir
def run_task(task):
    from modules.protocols import CONFIGS, contact_test, free_cycle, save_series

    result = {k: task[k] for k in ("case_id", "realization", "config", "test")}
    if task.get("tag"):
        result["tag"] = task["tag"]
    if task.get("dry_run"):  # metriche finte per provare la pipeline senza SOFA
        import math
        seed = sum(map(ord, task["case_id"]))
        if task["test"] == "free":
            yt = task["overrides"].get("yt_offset", 0.6) / 0.6 - 1
            result["metrics"] = {"H_pct": 15.0 * (1 + 0.8 * yt) + 0.1 * math.sin(seed)}
        else:
            result["metrics"] = {"stable": True, "in_band": task["test"] == "soft"}
        return result
    try:
        overrides = {**CONFIGS[task["config"]], **task["overrides"]}
        if task["test"] == "free":
            metrics, history = free_cycle(overrides)
        else:
            metrics, history = contact_test(task["test"], overrides)
        save_series(os.path.join(task["series_dir"], f"{task['case_id']}.csv"), history)
        result["metrics"] = metrics
    except Exception as exc:  # l'errore viene salvato nel risultato e la campagna continua
        result["error"] = f"{type(exc).__name__}: {exc}"
        result["traceback"] = traceback.format_exc()
    return result
