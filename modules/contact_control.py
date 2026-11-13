# modules/contact_control.py
"""Stato di contatto e regolazione della forza di contatto. Puro Python (testabile).

Indicazioni dell'assistente del corso per il Progetto 5: la forza di contatto va mantenuta
attivamente a 5 mN per tutta la durata del contatto, con margine +/-10 % (4.5-5.5 mN).

ContactState: macchina a stati del rilevamento (unico punto da cambiare se cambia la regola)

    LIBERO   --[F >= F_contact_th (5 mN, traccia)]-->      CONTATTO   (k commuta)
    CONTATTO --[F <  contact_loss_force (0.1 mN)]-->        LIBERO     (k torna al valore libero)

  5 mN e' sia la soglia di rilevamento sia la forza da mantenere: confrontare F con 5 mN a ogni
  passo farebbe commutare k di continuo. Per questo durante il contatto k resta fisso e si torna
  in LIBERO solo quando il contatto si perde.

ForceRegulator: regolazione a banda morta sul motore.
  * |F - F_ref| > start: il regolatore interviene e impone alla tensione dT/dt = -gain*(F - F_ref);
    la velocita' del motore e' dT/dt divisa per la rigidezza locale della molla
    (SeriesSpring.stiffness, che include l'isteresi di Dahl), limitata a v_max;
  * |F - F_ref| < stop: il motore si ferma. A u fermo la forza resta costante, quindi non si
    innesca il ciclo limite che un PI continuo avrebbe con l'isteresi di Dahl (circa 1.7 N di T a
    ogni inversione del motore).
  Lavorare su dT/dt rende il guadagno indipendente da k e dallo stato di Dahl: resta solo la
  rigidezza del contatto (dF/dT, circa 10 volte maggiore sul rigido che sul soffice). Guadagni
  e velocita' sono unici per tutte le configurazioni, realizzazioni e oggetti.
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

    def update(self, F):
        """Aggiorna lo stato con la forza misurata [N]. Restituisce True se lo stato cambia."""
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

    def step(self, F, spring, dt):
        """Spostamento del motore [mm] per questo passo, data la forza misurata F [N]."""
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
