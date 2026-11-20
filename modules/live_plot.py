"""
modules/live_plot.py

Listener del controller usati solo nella GUI.

    LivePlotListener    avvia tools/live_plot_viewer.py in un processo separato (Python di
                        sistema con matplotlib e tkinter) e gli invia un campione ogni
                        plot_every passi; se la finestra non parte o viene chiusa si disattiva
                        senza fermare la simulazione
    CsvRecorder         scrive ogni passo in results/gui_<data_ora>.csv
"""

import atexit
import csv
import datetime as dt
import json
import os
import shutil
import subprocess

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VIEWER = os.path.join(PROJECT_DIR, "tools", "live_plot_viewer.py")


# ambiente del viewer senza le variabili di SOFA (librerie e numpy compilati per 3.12)
def _viewer_env():
    env = dict(os.environ)
    for var in ("PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH"):
        env.pop(var, None)
    env["PYTHONPATH"] = PROJECT_DIR          # per importare modules.config e modules.metrics
    return env


# primo interprete che importa matplotlib e tkinter: quello indicato in config
# (live_plot_python), poi il Python di sistema, poi python3.12
def find_viewer_python(preferred=None):
    candidates = [preferred, "/usr/bin/python3", shutil.which("python3"), shutil.which("python3.12")]
    seen = set()
    for exe in candidates:
        if not exe or exe in seen or not os.path.exists(exe):
            continue
        seen.add(exe)
        try:
            ok = subprocess.run([exe, "-c", "import matplotlib, tkinter"], env=_viewer_env(),
                                capture_output=True, timeout=20).returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            ok = False
        if ok:
            return exe
    return None


class LivePlotListener:
    def __init__(self, every=5, python=None):
        self.every = max(1, int(every))
        self.count = 0
        self.proc = None
        exe = find_viewer_python(python)
        if exe is None:
            print("[LIVE PLOT] disattivati: nessun Python con matplotlib e tkinter. "
                  "Installare con: sudo apt install python3-matplotlib python3-tk")
            return
        try:
            self.proc = subprocess.Popen([exe, "-u", VIEWER], stdin=subprocess.PIPE,
                                         text=True, env=_viewer_env())
            print(f"[LIVE PLOT] finestra avviata con {exe}")
        except OSError as exc:
            print(f"[LIVE PLOT] disattivati: {exc}")

    # invia il campione al viewer come riga JSON
    def __call__(self, sample):
        if self.proc is None:
            return
        self.count += 1
        if self.count % self.every:
            return
        if self.proc.poll() is not None:     # finestra chiusa o viewer terminato
            print("[LIVE PLOT] finestra chiusa: grafici disattivati (la simulazione continua)")
            self.proc = None
            return
        try:
            self.proc.stdin.write(json.dumps(sample) + "\n")
            self.proc.stdin.flush()
        except (BrokenPipeError, OSError):
            self.proc = None


class CsvRecorder:
    def __init__(self, fields, outdir=None):
        outdir = outdir or os.path.join(PROJECT_DIR, "results")
        os.makedirs(outdir, exist_ok=True)
        self.path = os.path.join(outdir, "gui_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S")
                                 + ".csv")
        self.fields = list(fields)
        self.file = open(self.path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.fields)
        self.rows = 0
        atexit.register(self.file.close)     # chiude il file all'uscita di runSofa
        print(f"[CSV] registrazione in {self.path}")

    # scrive una riga per passo, svuota il buffer ogni 100 righe
    def __call__(self, sample):
        self.writer.writerow([sample[f] for f in self.fields])
        self.rows += 1
        if self.rows % 100 == 0:
            self.file.flush()
