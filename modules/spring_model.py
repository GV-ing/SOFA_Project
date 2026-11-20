"""
modules/spring_model.py

Molla in serie tra il carrello del motore e il tendine (solo Python, senza SOFA).

    dLs = max(0, u - u1)                                    allungamento della molla [mm]
    z   : stato di Dahl, dz/du = sigma0 * (1 - z*sgn(du))   z in [-1, +1]
    T   = max(0, k*dLs + z*(Fh_molla + Fh_mat) - c*du1/dt)  tensione sul tendine [N]

L'isteresi di Dahl segue lo spostamento del motore u, lo smorzamento c il moto del tendine u1.
Rigidezza: k_spring nella config. (a); nella config. (b) k_free in moto libero e k_contact
durante il contatto, con commutazione istantanea quando cambia lo stato di contatto.
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

    # stato iniziale: molla scarica, moto libero
    def reset(self):
        self.z = 0.0
        self.in_contact = False
        self.k = self.k_free if self.switched else self.k_fixed
        self.T = 0.0
        self.dL_s = 0.0
        self.u_prev = 0.0
        self.u1_prev = 0.0
        self.direction = 1.0

    # config. (b): k_contact durante il contatto, k_free altrimenti
    def set_contact(self, in_contact):
        self.in_contact = bool(in_contact)
        if self.switched:
            self.k = self.k_contact if self.in_contact else self.k_free

    # aggiornamento esatto dello stato di Dahl per uno spostamento du del motore
    def update_dahl(self, du):
        if du > 0.0:
            self.z = 1.0 - (1.0 - self.z) * math.exp(-self.sigma0 * du)
            self.direction = 1.0
        elif du < 0.0:
            self.z = -1.0 + (1.0 + self.z) * math.exp(self.sigma0 * du)
            self.direction = -1.0
        self.z = min(1.0, max(-1.0, self.z))

    # rigidezza locale dT/du [N/mm] per un piccolo spostamento del motore nel verso indicato:
    # k + sigma0*F_h*(1 - z*verso), quindi isteresi di Dahl compresa
    def stiffness(self, direction):
        s = 1.0 if direction >= 0 else -1.0
        return self.k + self.sigma0 * self.F_hyst_max * (1.0 - self.z * s)

    # avanza di dt con comando del motore u e spostamento del tendine u1; restituisce T [N]
    def step(self, u, u1, dt):
        if dt <= 0:
            raise ValueError("dt deve essere positivo")
        self.update_dahl(u - self.u_prev)
        self.dL_s = max(0.0, u - u1)
        u1_dot = (u1 - self.u1_prev) / dt
        if self.dL_s <= 0.0:
            self.T = 0.0  # molla scarica: nessuna forza sul cavo
        else:
            self.T = max(0.0, self.k * self.dL_s + self.z * self.F_hyst_max - self.c * u1_dot)
        self.u_prev = u
        self.u1_prev = u1
        return self.T
