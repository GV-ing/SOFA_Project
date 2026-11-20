"""
modules/controller.py

Controller del tubo a tendine (Sofa.Core.Controller): modo manuale da tastiera, ciclo
autonomo, molla in serie, commutazione di k al contatto e regolazione della forza di contatto.
A ogni comando stampa u, T, dLs e la posizione della punta.

Tasti:
    freccia su / U / +      carico (+u_step)
    freccia giù / D / -     scarico (-u_step)
    R                       reset
    M / A                   manuale <-> autonomo
In modo manuale, durante il contatto la freccia su è ignorata finché il regolatore è attivo;
la freccia giù disattiva il regolatore e arretra il carrello, la freccia su lo riattiva.

Fasi del ciclo autonomo:
    load        u cresce a auto_rate; al contatto passa a regulate; in moto libero si ferma
                a T = auto_T_reverse o a u = u_max e passa a hold
    regulate    regolazione della forza; dopo contact_settle_s con F in banda a motore fermo
                passa a hold (se il contatto si perde torna a load)
    hold        finestra di hold_duration s in contatto (regolatore attivo) o pausa di
                auto_hold_s in moto libero, poi unload
    unload      u scende a auto_rate fino a 0, poi done (o load se auto_repeat)
"""

import logging

import numpy as np
import Sofa
import Sofa.Core

from modules.contact_control import ContactState, ForceRegulator
from modules.spring_model import SeriesSpring

# grandezze registrate a ogni passo
HISTORY_FIELDS = ("t", "phase", "contact", "u", "u_target", "u1", "dLs", "T", "k", "z",
                  "E", "tip_x", "tip_y", "tip_z", "dY", "F_contact")


class TendonSpringController(Sofa.Core.Controller):

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    # componenti della scena passati da main_scene.py
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

    # stato dell'attuazione
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

    # stato del ciclo autonomo
    self.autonomous_mode = False
    self.phase = "manual"
    self.hold_timer = 0.0
    self.hold_s = 0.0
    self.settle_timer = 0.0
    self.manual_force_armed = True   # modo manuale: regolazione attiva al contatto

    self.initial_tip = None
    self.history = {name: [] for name in HISTORY_FIELDS}
    self.listeners = []   # funzioni chiamate a ogni passo con il campione (grafici, CSV)
    self._warned_missing_cable_displacement = False

  # posizione media dei nodi di punta [mm]
  def get_tip_position(self):
    pos = self.tube_meca.position.value
    return np.mean(pos[self.tip_node_indices], axis=0)

  # spostamento del tendine a valle della molla u1 [mm], dal displacement del CableConstraint
  def get_tendon_displacement(self):
    try:
      return max(0.0, float(np.ravel(self.cable.displacement.value)[0]))
    except (AttributeError, IndexError, TypeError, ValueError):
      if not self._warned_missing_cable_displacement:
        logging.warning("CableConstraint.displacement non accessibile; uso u1 = 0.")
        self._warned_missing_cable_displacement = True
      return 0.0

  # forza normale di contatto [N]; un errore di lettura viene contato e la forza vale 0
  def read_contact_force(self):
    if self.contact_force_provider is None:
      return 0.0
    try:
      force = float(self.contact_force_provider(self.root))
    except Exception as exc:
      self.contact_read_errors += 1
      if self.contact_read_errors == 1:
        logging.error("Lettura forza di contatto fallita (%s); uso 0 N. "
                      "Errori contati in contact_read_errors.", exc)
      return 0.0
    if not np.isfinite(force) or force < 0.0:
      raise ValueError(f"Forza di contatto non valida: {force}")
    return force * self.force_scale

  # stampa u, T, dLs, k, posizione della punta e forza di contatto
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

  # cambia il modulo di Young del FEM; reinit() ricalcola la rigidezza
  def _set_young_modulus(self, E):
    if abs(E - self.E_current) > 1e-9:
      self.fem_ff.youngModulus.value = [E]
      self.fem_ff.reinit()
      self.E_current = E

  # calcola T con il modello della molla, sceglie E di carico o scarico e applica T al cavo
  def update_actuation_physics(self, dt):
    self.u1 = self.get_tendon_displacement()
    self.T = self.spring.step(self.u, self.u1, dt)
    self.delta_Ls = self.spring.dL_s
    self._set_young_modulus(
        self.params["E_load"] if self.spring.direction >= 0 else self.params["E_unload"])
    # il valore del CableConstraint è un impulso: T*dt
    self.cable.value.value = [self.T * dt]

  # True se in questo passo il motore è comandato dal regolatore di forza
  def _regulating(self):
    if self.autonomous_mode:
      return self.phase == "regulate" or (self.phase == "hold" and self.contact.in_contact)
    return self.contact.in_contact and self.manual_force_armed

  # True se F è dentro la banda force_ref +/- force_band
  def _in_band(self):
    return (abs(self.contact_force_N - self.regulator.F_ref)
            <= self.params["force_band"] * self.regulator.F_ref)

  # un passo della macchina a stati del ciclo autonomo
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

  # pausa a fine carico (moto libero) o finestra di stabilità (contatto)
  def _enter_hold(self, duration):
    self.hold_s = float(duration)
    self.hold_timer = 0.0
    self.phase = "hold" if self.hold_s > 0 else "unload"

  # blocca il riferimento di u e fa ripartire il regolatore
  def _enter_regulation(self):
    self.u_target = self.u
    self.regulator.reset()
    self.settle_timer = 0.0
    if self.autonomous_mode:
      self.phase = "regulate"

  # prima del passo: aggiorna u (regolatore o rampa verso u_target) e applica T al cavo
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

  # dopo il passo: legge la forza, aggiorna lo stato di contatto e registra il campione
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
      except Exception as exc:  # il listener che va in errore viene tolto
        logging.warning("Listener disattivato dopo errore: %s", exc)
        self.listeners.remove(listener)

  # passaggio libero <-> contatto: k commuta (config. b) e si attiva il regolatore
  def _on_contact_change(self):
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

  # avvia il ciclo autonomo dalla posizione attuale
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

  # torna al modo manuale
  def stop_autonomous_cycle(self):
    self.autonomous_mode = False
    self.phase = "manual"
    self.u_target = self.u
    self.regulator.reset()

  # riporta u a 0 e azzera la molla
  def reset(self):
    self.autonomous_mode = False
    self.phase = "manual"
    self.u = self.u_target = 0.0
    self.spring.reset()
    # lo stato di contatto non si azzera: si aggiorna da solo quando la forza scende;
    # il regolatore resta disattivato fino alla prossima freccia su
    self.spring.set_contact(self.contact.in_contact)
    self.regulator.reset()
    self.manual_force_armed = False
    self.T = 0.0
    self.cable.value.value = [0.0]
    self._set_young_modulus(self.params["E_load"])

  # comandi da tastiera ("\x13" e "\x15" sono le frecce su e giù in runSofa)
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
      elif self.contact.in_contact:              # dopo un rilascio: riattiva il regolatore
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
