# modules/spring_model.py
"""Modello della molla in serie tra carrello motore e tendine. Puro Python (testabile).

Traccia (requisiti comuni): T = k*dLs +/- isteresi, con smorzamento e isteresi della molla.

    dLs = max(0, u - u1)                                    allungamento della molla [mm]
    z   : stato di Dahl, dz/du = sigma0 * (1 - z*sgn(du))   (z in [-1, +1])
    T   = max(0, k*dLs + z*(Fh_molla + Fh_mat) - c*du1/dt)  tensione sul tendine [N]

Il Progetto 5 non pone vincoli su T: nessuna saturazione superiore, nessun limite su dT/dt.

Rigidezza:
  * configurazione (a): k = k_spring (1.2 N/mm);
  * configurazione (b): k_free in moto libero, k_contact durante il contatto, scelti con
    k_free_level ("low": 0.6 libero / 4.8 contatto; "high": 4.8 libero / 0.6 contatto).
    La commutazione e' istantanea, a pari allungamento dLs. Lo stato di contatto arriva da
    modules/contact_control.py (ContactState) tramite set_contact().

Scelte di progetto:
  * isteresi di Dahl sullo spostamento del motore u, sigma0 = 10 mm^-1, ampiezza
    Fh_molla + Fh_mat (le due isteresi della tabella agiscono sullo stesso cavo);
  * smorzamento c sul moto del tendine u1, non sull'allungamento della molla.
"""

import math


class SeriesSpring:
    def __init__(self, params):
        self.switched = bool(params.get("switched_mode", False))
        self.k_fixed = float(params["k_spring"])
        self.k_low = float(params["k_low"])
        self.k_high = float(params["k_high"])
        level = params.get("k_free_level", "low")
        if level not in ("low", "high"):
            raise ValueError("k_free_level deve essere 'low' oppure 'high'")
        self.k_free = self.k_high if level == "high" else self.k_low
        self.k_contact = self.k_low if level == "high" else self.k_high
        self.F_hyst_max = float(params["Fh_spring"]) + float(params["Fh_mat"])
        self.sigma0 = float(params["dahl_sigma0"])
        self.c = float(params["c_spring"])
        for name in ("k_fixed", "k_low", "k_high", "sigma0"):
            if not getattr(self, name) > 0:
                raise ValueError(f"{name} deve essere positivo")
        if self.F_hyst_max < 0 or self.c < 0:
            raise ValueError("Isteresi e smorzamento devono essere non negativi")
        self.reset()

    # ------------------------------------------------------------------ stato
    def reset(self):
        self.z = 0.0
        self.in_contact = False
        self.k = self.k_free if self.switched else self.k_fixed
        self.T = 0.0
        self.dL_s = 0.0
        self.u_prev = 0.0
        self.u1_prev = 0.0
        self.direction = 1.0

    def set_contact(self, in_contact):
        """Configurazione (b): k_contact durante il contatto, k_free altrimenti."""
        self.in_contact = bool(in_contact)
        if self.switched:
            self.k = self.k_contact if self.in_contact else self.k_free

    # ------------------------------------------------------------------ Dahl
    def update_dahl(self, du):
        """Aggiornamento esatto di Dahl per uno spostamento du a segno costante."""
        if du > 0.0:
            self.z = 1.0 - (1.0 - self.z) * math.exp(-self.sigma0 * du)
            self.direction = 1.0
        elif du < 0.0:
            self.z = -1.0 + (1.0 + self.z) * math.exp(self.sigma0 * du)
            self.direction = -1.0
        self.z = min(1.0, max(-1.0, self.z))

    def stiffness(self, direction):
        """dT/du locale [N/mm] per un piccolo spostamento del motore nel verso indicato.

        Vale k + sigma0*F_h*(1 - z*verso): subito dopo un'inversione l'isteresi di Dahl rende
        la molla molto piu' rigida di k (fino a k + 2*sigma0*F_h = k + 17 N/mm).
        """
        s = 1.0 if direction >= 0 else -1.0
        return self.k + self.sigma0 * self.F_hyst_max * (1.0 - self.z * s)

    # ------------------------------------------------------------------ passo
    def step(self, u, u1, dt):
        """Avanza di dt con comando motore u e spostamento del tendine u1. Restituisce T [N]."""
        if dt <= 0:
            raise ValueError("dt deve essere positivo")
        self.update_dahl(u - self.u_prev)
        self.dL_s = max(0.0, u - u1)
        u1_dot = (u1 - self.u1_prev) / dt
        if self.dL_s <= 0.0:
            self.T = 0.0  # molla scarica: il cavo non trasmette forza
        else:
            self.T = max(0.0, self.k * self.dL_s + self.z * self.F_hyst_max - self.c * u1_dot)
        self.u_prev = u
        self.u1_prev = u1
        return self.T
