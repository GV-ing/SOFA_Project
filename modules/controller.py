# modules/controller.py
"""Controller da tastiera e autonomo del tubo a tendine con molla in serie (SofaPython3).

Comandi (traccia: carico/scarico, reset, manuale/autonomo; a ogni comando stampa u, T, dLs, punta):
    Freccia SU / U / +   carico   (+u_step)          Freccia GIU' / D / -   scarico (-u_step)
    R                    reset                        M / A                  manuale <-> autonomo

Contatto (modules/contact_control.py): a F >= 5 mN lo stato passa a CONTATTO, k commuta
(configurazione b) e il regolatore di forza tiene F a 5 mN +/-10 % finche' il contatto dura.
  * Modo manuale: in contatto la freccia SU e' ignorata (la forza e' regolata); la freccia GIU'
    rilascia: il regolatore si disarma e il carrello arretra di u_step. La freccia SU lo riarma.
    Il reset lo disarma (la forza decade senza che il regolatore riporti la punta sull'oggetto).
  * Ciclo autonomo (macchina a stati):
      'load'     u cresce a auto_rate; se si rileva il contatto -> 'regulate'; in moto libero si
                 inverte a T >= auto_T_reverse o a u = u_max -> 'hold' (auto_hold_s) o 'unload'
      'regulate' regolazione di forza; quando F resta in banda a motore fermo per
                 contact_settle_s -> 'hold' (se il contatto si perde torna a 'load')
      'hold'     finestra di hold_duration s in contatto (regolatore attivo) o auto_hold_s in
                 moto libero -> 'unload'
      'unload'   u scende a auto_rate fino a 0 -> 'done' (o 'load' se auto_repeat); un nuovo
                 rilevamento durante lo scarico non riattiva la regolazione
    La fase e' registrata per separare carico e scarico nel calcolo di H.

Convenzioni SOFA 26.06 verificate durante lo sviluppo: al CableConstraint si impone l'impulso T*dt;
cambiare youngModulus richiede reinit() del TetrahedronFEMForceField.
"""

import logging

import numpy as np
import Sofa
import Sofa.Core

from modules.contact_control import ContactState, ForceRegulator
from modules.spring_model import SeriesSpring

HISTORY_FIELDS = ("t", "phase", "contact", "u", "u_target", "u1", "dLs", "T", "k", "z",
                  "E", "tip_x", "tip_y", "tip_z", "dY", "F_contact")


class TendonSpringController(Sofa.Core.Controller):

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.root = kwargs.get("root")
    self.tube_meca = kwargs.get("tube_meca")
    self.fem_ff = kwargs.get("fem_ff")
    self.cable = kwargs.get("cable")
    self.params = kwargs.get("params")
    self.tip_node_indices = kwargs.get("tip_node_indices")
    self.contact_force_provider = kwargs.get("contact_force_provider")

    self.force_scale = float(self.params["force_scale"])
    if not np.isfinite(self.force_scale) or self.force_scale <= 0.0:
      raise ValueError("force_scale deve essere un fattore positivo e finito")

    self.spring = SeriesSpring(self.params)
    self.contact = ContactState(self.params)
    self.regulator = ForceRegulator(self.params)
    self.u = 0.0
    self.u_target = 0.0
    self.u1 = 0.0
    self.T = 0.0
    self.delta_Ls = 0.0
    self.E_current = float(self.params["E_load"])
    self.contact_force_N = 0.0
    self.contact_read_errors = 0
    self.time = 0.0

    self.autonomous_mode = False
    self.phase = "manual"
    self.hold_timer = 0.0
    self.hold_s = 0.0
    self.settle_timer = 0.0
    self.manual_force_armed = True   # modo manuale: regolazione attiva in contatto (SU / GIU')

    self.initial_tip = None
    self.history = {name: [] for name in HISTORY_FIELDS}
    self.listeners = []   # callable(sample: dict) chiamati a ogni passo (grafici live)
    self._warned_missing_cable_displacement = False

  # ------------------------------------------------------------------ misure
  def get_tip_position(self):
    """Posizione media dei nodi di punta [mm]."""
    pos = self.tube_meca.position.value
    return np.mean(pos[self.tip_node_indices], axis=0)

  def get_tendon_displacement(self):
    """Spostamento del tendine a valle della molla u1 [mm] (CableConstraint.displacement)."""
    try:
      # In SoftRobots 26.06 'displacement' e' uno scalare: np.ravel gestisce scalare e vettore
      return max(0.0, float(np.ravel(self.cable.displacement.value)[0]))
    except (AttributeError, IndexError, TypeError, ValueError):
      if not self._warned_missing_cable_displacement:
        logging.warning("CableConstraint.displacement non accessibile; uso u1 = 0.")
        self._warned_missing_cable_displacement = True
      return 0.0

  def read_contact_force(self):
    """Forza normale di contatto [N] dal provider (modules/contact_force.py)."""
    if self.contact_force_provider is None:
      return 0.0
    try:
      force = float(self.contact_force_provider(self.root))
    except Exception as exc:  # errore di lettura: segnalato, non confuso con "nessun contatto"
      self.contact_read_errors += 1
      if self.contact_read_errors == 1:
        logging.error("Lettura forza di contatto fallita (%s); uso 0 N. "
                      "Errori contati in contact_read_errors.", exc)
      return 0.0
    if not np.isfinite(force) or force < 0.0:
      raise ValueError(f"Forza di contatto non valida: {force}")
    return force * self.force_scale

  # ------------------------------------------------------------------ stampa
  def print_state_feedback(self):
    tip = self.get_tip_position()
    dY = float(tip[1]) - float(self.initial_tip[1]) if self.initial_tip is not None else 0.0
    print(
        f"[CONTROLLER] u: {self.u:.3f} mm (rif {self.u_target:.3f}) | T: {self.T:.3f} N | "
        f"dLs: {self.delta_Ls:.3f} mm | k: {self.spring.k:.2f} N/mm | "
        f"Punta: X={tip[0]:.3f}, Y={tip[1]:.4f} (dY={dY:.4f}), Z={tip[2]:.4f} mm | "
        f"F_contatto: {self.contact_force_N * 1e3:.2f} mN"
        f"{' | CONTATTO' if self.contact.in_contact else ''}"
    )

  # ------------------------------------------------------------------ fisica
  def _set_young_modulus(self, E):
    if abs(E - self.E_current) > 1e-9:
      self.fem_ff.youngModulus.value = [E]
      # TetrahedronFEMForceField NON ricalcola la rigidezza al solo cambio del Data:
      # serve reinit() (verificato durante lo sviluppo: rapporto 1.000 -> 1.185).
      self.fem_ff.reinit()
      self.E_current = E

  def update_actuation_physics(self, dt):
    """Molla in serie -> tensione T sul cavo; E di carico o scarico secondo il verso di u."""
    self.u1 = self.get_tendon_displacement()
    self.T = self.spring.step(self.u, self.u1, dt)
    self.delta_Ls = self.spring.dL_s
    self._set_young_modulus(
        self.params["E_load"] if self.spring.direction >= 0 else self.params["E_unload"])
    # Con FreeMotionAnimationLoop il valore del vincolo e' un IMPULSO: si impone T*dt.
    self.cable.value.value = [self.T * dt]

  # ------------------------------------------------------------------ logica
  def _regulating(self):
    """True se in questo passo il motore e' comandato dal regolatore di forza."""
    if self.autonomous_mode:
      return self.phase == "regulate" or (self.phase == "hold" and self.contact.in_contact)
    return self.contact.in_contact and self.manual_force_armed

  def _in_band(self):
    return (abs(self.contact_force_N - self.regulator.F_ref)
            <= self.params["force_band"] * self.regulator.F_ref)

  def _autonomous_step(self, dt):
    rate = self.params["auto_rate"]
    if self.phase == "load":
      if self.contact.in_contact:
        self._enter_regulation()
      else:
        self.u_target = min(self.params["u_max"], self.u_target + rate * dt)
        if (self.T >= self.params["auto_T_reverse"]
            or self.u >= self.params["u_max"] - 1e-9):
          self.u_target = self.u
          self._enter_hold(self.params["auto_hold_s"])
    elif self.phase == "regulate":
      if not self.contact.in_contact:
        self.phase = "load"                      # contatto perso: nuovo avvicinamento
      else:
        idle = not self.regulator.active and self._in_band()
        self.settle_timer = self.settle_timer + dt if idle else 0.0
        if self.settle_timer >= self.params["contact_settle_s"] - 1e-9:
          self._enter_hold(self.params["hold_duration"])
    elif self.phase == "hold":
      self.hold_timer += dt
      if self.hold_timer >= self.hold_s - 1e-9:
        self.u_target = self.u
        self.phase = "unload"
    elif self.phase == "unload":
      self.u_target = max(0.0, self.u_target - rate * dt)
      if self.u <= 1e-9 and self.u_target <= 0.0:
        self.phase = "load" if self.params["auto_repeat"] else "done"

  def _enter_hold(self, duration):
    """Pausa a fine carico (moto libero) o finestra di stabilita' (contatto)."""
    self.hold_s = float(duration)
    self.hold_timer = 0.0
    self.phase = "hold" if self.hold_s > 0 else "unload"

  def _enter_regulation(self):
    self.u_target = self.u
    self.regulator.reset()
    self.settle_timer = 0.0
    if self.autonomous_mode:
      self.phase = "regulate"

  def onAnimateBeginEvent(self, event):
    dt = float(self.root.dt.value)
    if self.initial_tip is None:
      self.initial_tip = np.array(self.get_tip_position(), dtype=float)
    if self.autonomous_mode:
      self._autonomous_step(dt)
    if self._regulating():
      du = self.regulator.step(self.contact_force_N, self.spring, dt)
      self.u = float(np.clip(self.u + du, 0.0, self.params["u_max"]))
      self.u_target = self.u
    else:
      rate = self.params["auto_rate"] if self.autonomous_mode else self.params["manual_rate"]
      max_step = rate * dt
      self.u += float(np.clip(self.u_target - self.u, -max_step, max_step))
    self.update_actuation_physics(dt)

  def onAnimateEndEvent(self, event):
    dt = float(self.root.dt.value)
    self.time += dt
    self.contact_force_N = self.read_contact_force()
    if self.contact.update(self.contact_force_N):
      self._on_contact_change()
    tip = self.get_tip_position()
    sample = {
        "t": self.time, "phase": self.phase, "contact": int(self.contact.in_contact),
        "u": self.u, "u_target": self.u_target,
        "u1": self.u1, "dLs": self.delta_Ls, "T": self.T, "k": self.spring.k,
        "z": self.spring.z, "E": self.E_current,
        "tip_x": float(tip[0]), "tip_y": float(tip[1]), "tip_z": float(tip[2]),
        "dY": float(tip[1] - self.initial_tip[1]), "F_contact": self.contact_force_N,
    }
    for name in HISTORY_FIELDS:
      self.history[name].append(sample[name])
    for listener in list(self.listeners):
      try:
        listener(sample)
      except Exception as exc:  # un grafico che fallisce non deve fermare la fisica
        logging.warning("Listener disattivato dopo errore: %s", exc)
        self.listeners.remove(listener)

  def _on_contact_change(self):
    """Transizione LIBERO <-> CONTATTO: k commuta (config. b) e si arma il regolatore."""
    k_old = self.spring.k
    self.spring.set_contact(self.contact.in_contact)
    if self.contact.in_contact:
      regulate = ((self.autonomous_mode and self.phase in ("load", "regulate", "hold"))
                  or (not self.autonomous_mode and self.manual_force_armed))
      print(f"[CONTROLLER] t = {self.time:.2f} s: contatto rilevato (F = "
            f"{self.contact_force_N * 1e3:.2f} mN), k {k_old:.2f} -> {self.spring.k:.2f} N/mm"
            + (f", regolazione a {self.regulator.F_ref * 1e3:.1f} mN" if regulate else ""))
      if regulate:
        self._enter_regulation()
    else:
      print(f"[CONTROLLER] t = {self.time:.2f} s: contatto perso, k {k_old:.2f} -> "
            f"{self.spring.k:.2f} N/mm")
      self.regulator.reset()

  # ------------------------------------------------------------------ comandi
  def start_autonomous_cycle(self, hold_s=None, repeat=None):
    if hold_s is not None:
      self.params["auto_hold_s"] = float(hold_s)
    if repeat is not None:
      self.params["auto_repeat"] = bool(repeat)
    self.autonomous_mode = True
    self.phase = "load"
    self.u_target = self.u
    if self.contact.in_contact:
      self._enter_regulation()

  def stop_autonomous_cycle(self):
    self.autonomous_mode = False
    self.phase = "manual"
    self.u_target = self.u
    self.regulator.reset()

  def reset(self):
    self.autonomous_mode = False
    self.phase = "manual"
    self.u = self.u_target = 0.0
    self.spring.reset()
    # Lo stato di contatto non si azzera: la forza decade in pochi passi e la perdita del
    # contatto viene rilevata normalmente (niente falso "contatto rilevato" durante il decadimento).
    # Il regolatore resta disarmato fino alla prossima freccia SU, altrimenti riporterebbe la
    # punta sull'oggetto mentre la forza decade.
    self.spring.set_contact(self.contact.in_contact)
    self.regulator.reset()
    self.manual_force_armed = False
    self.T = 0.0
    self.cable.value.value = [0.0]
    self._set_young_modulus(self.params["E_load"])

  def onKeypressedEvent(self, event):
    key = str(event.get("key", ""))
    step = self.params["u_step"]
    is_up = key in ["\x13", "Up", "U", "u", "+"]
    is_down = key in ["\x15", "Down", "D", "d", "-"]
    is_reset = key in ["R", "r"]
    is_auto = key in ["M", "m", "A", "a"]

    if (is_up or is_down) and self.autonomous_mode:
      self.stop_autonomous_cycle()
    if is_up:
      if self._regulating():
        print("[CONTROLLER] In contatto la forza e' regolata a "
              f"{self.regulator.F_ref * 1e3:.1f} mN: carico ignorato (freccia giu' per rilasciare)")
      elif self.contact.in_contact:              # dopo un rilascio: la regolazione riparte
        self.manual_force_armed = True
        self._enter_regulation()
        print("[CONTROLLER] Regolazione di forza riattivata")
      else:
        self.manual_force_armed = True
        self.u_target = min(self.params["u_max"], self.u_target + step)
      self.print_state_feedback()
    elif is_down:
      if self._regulating():
        print("[CONTROLLER] Rilascio: regolazione di forza disattivata (freccia su per riattivarla)")
      self.manual_force_armed = False
      self.regulator.reset()
      self.u_target = max(0.0, self.u_target - step)
      self.print_state_feedback()
    elif is_reset:
      self.reset()
      print("[CONTROLLER] Reset eseguito: u = 0.0 mm")
      self.print_state_feedback()
    elif is_auto:
      if self.autonomous_mode:
        self.stop_autonomous_cycle()
      else:
        self.start_autonomous_cycle()
      state = "ATTIVATA" if self.autonomous_mode else "DISATTIVATA"
      print(f"[CONTROLLER] Modalita' autonoma: {state}")
      self.print_state_feedback()
