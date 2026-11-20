"""
modules/contact_control.py

Stato di contatto e regolazione della forza di contatto (solo Python, senza SOFA).

ContactState:
    LIBERO   -> CONTATTO   quando F >= F_contact_th
    CONTATTO -> LIBERO     quando F <  contact_loss_force
    Durante il contatto k resta fisso; si torna in LIBERO solo quando il contatto si perde.

ForceRegulator:
    regolatore a banda morta sul motore. Si attiva se |F - F_ref| > start e si ferma se
    |F - F_ref| < stop. Quando è attivo impone dT/dt = -gain * (F - F_ref); la velocità del
    motore è dT/dt diviso la rigidezza locale della molla (SeriesSpring.stiffness), limitata
    a v_max.
"""


class ContactState:
    def __init__(self, params):
        self.F_detect = float(params["F_contact_th"])
        self.F_loss = float(params["contact_loss_force"])
        if not 0.0 <= self.F_loss < self.F_detect:
            raise ValueError("Serve 0 <= contact_loss_force < F_contact_th")
        self.reset()

    def reset(self):
        self.in_contact = False

    # aggiorna lo stato con la forza misurata [N]; restituisce True se lo stato cambia
    def update(self, F):
        before = self.in_contact
        if self.in_contact:
            self.in_contact = F >= self.F_loss
        else:
            self.in_contact = F >= self.F_detect
        return self.in_contact != before


class ForceRegulator:
    def __init__(self, params):
        self.F_ref = float(params["force_ref"])
        self.start = float(params["force_reg_start"]) * self.F_ref
        self.stop = float(params["force_reg_stop"]) * self.F_ref
        self.gain = float(params["force_gain"])
        self.v_max = float(params["contact_rate"])
        band = float(params["force_band"]) * self.F_ref
        if not 0.0 < self.stop < self.start < band:
            raise ValueError("Serve 0 < force_reg_stop < force_reg_start < force_band")
        if self.gain <= 0.0 or self.v_max <= 0.0:
            raise ValueError("force_gain e contact_rate devono essere positivi")
        self.reset()

    def reset(self):
        self.active = False

    # spostamento del motore [mm] in questo passo, data la forza misurata F [N]
    def step(self, F, spring, dt):
        e = F - self.F_ref
        if self.active and abs(e) < self.stop:
            self.active = False
        elif not self.active and abs(e) > self.start:
            self.active = True
        if not self.active:
            return 0.0
        dT = -self.gain * e * dt                     # variazione di T richiesta nel passo [N]
        du = dT / spring.stiffness(1.0 if dT > 0.0 else -1.0)
        lim = self.v_max * dt
        return max(-lim, min(lim, du))
