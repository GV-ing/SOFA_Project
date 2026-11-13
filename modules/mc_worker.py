# modules/mc_worker.py
"""Esecuzione di un singolo caso Monte Carlo in un processo separato (spawn).

Ogni caso gira in un processo nuovo (max_tasks_per_child = 1): SOFA parte pulito e la
memoria viene liberata alla fine del caso.
"""

import os
import traceback


def run_task(task):
    """task: dict con case_id, realization, config, test, overrides, series_dir."""
    from modules.protocols import CONFIGS, contact_test, free_cycle, save_series

    result = {k: task[k] for k in ("case_id", "realization", "config", "test")}
    if task.get("tag"):
        result["tag"] = task["tag"]
    if task.get("dry_run"):  # verifica della pipeline senza SOFA (metriche sintetiche)
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
    except Exception as exc:  # il caso fallito viene registrato, la campagna prosegue
        result["error"] = f"{type(exc).__name__}: {exc}"
        result["traceback"] = traceback.format_exc()
    return result
