# modules/config.py
"""
Parametri fisici, geometrici e di incertezza per il Progetto 5:
"Incertezza parametrica: Monte Carlo e analisi di sensibilita'"
Unita' coerenti in SOFA: mm, N, MPa (N/mm^2), s.

Origine dei valori: (traccia) = PDF del Progetto 5 e requisiti comuni; (assistente del corso) =
indicazioni ricevute per il Progetto 5; (scelta) = scelta di progetto. Il glossario del corso
serve solo a quantificare grandezze che la traccia nomina.
"""

NOMINAL_PARAMS = {
    # Geometria del tubo continuo (traccia: tabella di riferimento)
    "od": 1.93,                 # Diametro esterno [mm] (traccia)
    "thickness": 0.22,          # Spessore parete [mm] (traccia, ID = 1.49 mm)
    "length_total": 100.0,      # Lunghezza totale [mm] (traccia)
    "length_shaft": 40.0,       # Albero prossimale vincolato [mm] (traccia)
    "length_flex": 60.0,        # Parte flessibile distale [mm] (traccia)

    # Discretizzazione (studio di convergenza: tools/mesh_convergence.py)
    "mesh_h_flex": 0.40,        # Passo assiale FEM sulla parte flessibile [mm]
    "mesh_n_shaft": 4,          # Elementi sull'albero vincolato (non deformabile)
    "mesh_n_theta": 12,         # Suddivisioni circonferenziali (ininfluenti sulla flessione)
    "visual_h": 2.0,            # Passo assiale della superficie visiva [mm]
    "collision_h": 1.0,         # Passo assiale della superficie di collisione [mm]
    "collision_margin": 5.0,    # La collisione parte da (L - obj_size - margin) [mm]

    # Materiale Nitinol (traccia)
    "E_load": 77000.0,          # Modulo di Young in carico [MPa] (traccia, 77 GPa)
    "E_unload": 65000.0,        # Modulo di Young in scarico [MPa] (traccia, 65 GPa)
    "poisson": 0.33,            # Coefficiente di Poisson [-] (traccia)
    "Fh_mat": 0.40,             # Isteresi del materiale [N] (traccia, +/- 0.40 N)

    # Tendine
    "yt_offset": 0.60,          # Offset eccentrico tendine lungo Y [mm] (traccia)

    # Molla in serie (traccia: tabella di riferimento e Progetto 5)
    "L0_spring": 20.0,          # Lunghezza a riposo [mm] (traccia; non entra nel modello, che usa
                                #   solo l'allungamento dLs = u - u1)
    "k_spring": 1.2,            # Config. (a): rigidezza fissa [N/mm] (traccia)
    "k_low": 0.6,               # Config. (b): livello basso [N/mm] (traccia)
    "k_high": 4.8,              # Config. (b): livello alto [N/mm] (traccia)
    # Config. (b): livello usato in moto libero (traccia: lo scelgono gli studenti).
    # "low" = 0.6 libero / 4.8 in contatto; "high" = 4.8 libero / 0.6 in contatto.
    # Scelto "high" sui casi nominali: H 12.4 % contro 16.0 %, molla cedevole in contatto e
    # nessun salto verso l'alto della forza al rilevamento (con "low" T sale da 5 a 35 N).
    "k_free_level": "high",
    "c_spring": 1.5,            # Smorzamento [N*s/mm] (traccia); agisce sul moto del tendine (-c*du1/dt)
    "Fh_spring": 0.45,          # Isteresi della molla [N] (traccia)
    "dahl_sigma0": 10.0,        # Rigidezza di Dahl dell'isteresi [1/mm] (scelta, range 5-20 del glossario)
    "switched_mode": False,     # False: config. (a) k fissa; True: config. (b) due livelli

    # Attuazione e ciclo autonomo
    "u_step": 0.35,             # Passo del comando per pressione di tasto [mm] (traccia)
    "u_max": 15.0,              # Corsa massima del carrello [mm] (scelta, corsa 0-15 mm del glossario)
    "manual_rate": 1.0,         # Velocita' del carrello nei comandi manuali [mm/s] (scelta)
    "auto_rate": 1.0,           # Velocita' del carrello nel ciclo autonomo [mm/s] (scelta, range
                                #   0.5-2 mm/s del glossario); la diretta usa 0.1 (SPRING_CONFIGS)
    "auto_T_reverse": 10.0,     # Ciclo libero per H: inversione a T = 10 N o a u_max [N]. Scelta di
                                #   protocollo (stesso picco di T per tutte le molle, quindi dY_max
                                #   confrontabile), non una saturazione di T
    "auto_hold_s": 0.0,         # Pausa a fine carico nel ciclo libero [s] (scelta)
    "auto_repeat": False,       # Ciclo autonomo ripetuto (True) o singolo (False)

    # Contatto e controllo di forza (modules/contact_control.py)
    "F_contact_th": 0.005,      # Soglia di rilevamento del contatto: 5 mN (traccia)
    "contact_loss_force": 1e-4, # Contatto perso sotto 0.1 mN (scelta)
    "force_ref": 0.005,         # Forza mantenuta per tutto il contatto: 5 mN (assistente del corso)
    "force_band": 0.10,         # Margine ammesso +/-10 %: 4.5-5.5 mN (assistente del corso)
    "force_reg_start": 0.06,    # Il regolatore interviene se |F - 5 mN| > 6 % = 0.3 mN (scelta)
    "force_reg_stop": 0.02,     # e ferma il motore quando |F - 5 mN| < 2 % = 0.1 mN (scelta)
    "force_gain": 1000.0,       # dT/dt = -force_gain * (F - force_ref) [1/s], cioe' 1 N/s per mN
                                #   (scelta: in SOFA F segue T entro 1-2 passi; sul rigido
                                #   force_gain * dF/dT * dt = 0.15, margine ~2 prima dei cicli limite)
    "contact_rate": 2.0,        # Avvicinamento nella prova di contatto e velocita' massima del
                                #   regolatore [mm/s] (scelta, massimo 2 mm/s del glossario)
    "contact_settle_s": 1.0,    # F in banda a motore fermo per 1 s prima della finestra [s] (scelta)
    "hold_duration": 5.0,       # Finestra di valutazione della stabilita' [s] (traccia)
    "stability_ptp_max": 0.30,  # Oscillazione picco-picco massima / media (traccia: 30 %)

    # Ambiente di Contatto
    "contact_object": "soft",   # "soft" | "rigid" | "none"
    "obj_size": 20.0,           # Dimensione caratteristica oggetto [mm]
    "obj_gap": 0.03,            # Gap punta-oggetto lungo Y [mm], UGUALE per rigido e soffice:
                                #   contatto a T ~ 0.3 N (c = 0.0304 mm/N), stessa geometria per il
                                #   confronto tra oggetti; H si misura comunque in moto libero
    "obj_overlap_x": 5.0,       # Quanto l'oggetto supera la punta lungo X [mm]
    "obj_grid_n": 5,            # Nodi per lato della griglia FEM dell'oggetto soffice
    "soft_mesh": "graded",      # "graded" (fine al contatto) | "uniform" (obj_grid_n per lato)
    "soft_h_min": 1.6,          # Passo minimo al contatto [mm]: compromesso costo/accuratezza (studio di convergenza)
    "soft_h_max": 5.0,          # Passo massimo lontano dal contatto [mm]
    "soft_growth": 1.4,         # Fattore di crescita geometrica del passo
    "E_soft": 0.005,            # Oggetto soffice: 5 kPa = 0.005 MPa (traccia)
    "poisson_soft": 0.45,       # Poisson oggetto soffice [-] (traccia)
    "mu_friction": 0.2,         # Coefficiente di attrito [-] (traccia)
    "alarm_distance": 0.30,     # LocalMinDistance alarmDistance [mm] (> gap, < dimensione elementi)
    "contact_distance": 0.02,   # LocalMinDistance contactDistance [mm]
    "force_scale": 1.0,         # Fattore forza del solver -> N (1: verificato, vedi contact_force.py)
    "random_seed": 2026,        # Seed per riproducibilita' delle realizzazioni
    "mc_workers": 3,            # Processi paralleli del Monte Carlo (VM a 6 CPU e 7.8 GB)

    # Masse da densita' (unita' mm-N-s: massa in tonnellate, densita' in t/mm^3) (scelta)
    "rho_tube": 6.45e-9,        # Nitinol 6450 kg/m^3 (scelta, valore tipico)
    "rho_soft": 1.0e-9,         # Oggetto soffice ~ tessuto/acqua 1000 kg/m^3 (scelta)

    # Modelli di collisione (Triangle | Line | Point). Verificato durante lo sviluppo:
    # i modelli completi sono piu' veloci (432 vs 691 ms/passo) e stabili; i ridotti
    # (solo Point vs Triangle) oscillano a forza elevata (61 mN p-p a T = 10 N).
    "tube_collision_models": ("Triangle", "Line", "Point"),
    "object_collision_models": ("Triangle", "Line", "Point"),

    # Simulazione e output
    "dt": 0.01,                 # Passo di integrazione temporale [s] (di consegna: non va cambiato)
    "cs_max_iterations": 500,   # BlockGaussSeidelConstraintSolver: iterazioni massime
    "cs_tolerance": 1e-7,       # BlockGaussSeidelConstraintSolver: tolleranza
    "live_plot": True,          # Grafici in tempo reale nella GUI (tools/live_plot_viewer.py)
    # Solo GUI (runSofa): profilo interattivo e vista amplificata. Non usati da script e MC.
    "gui_fast": True,
    # dt (0.01 s) e solver dei vincoli (500 iterazioni, tol 1e-7) restano quelli del
    # Monte Carlo anche nella GUI: con tolleranza 1e-5 il contatto andava in chattering.
    "gui_overrides": {
        "mesh_h_flex": 0.8,     # tubo piu' leggero (rigidezza ~85% del valore convergente)
        "soft_mesh": "uniform", "obj_grid_n": 5,   # soffice leggero
        "collision_h": 2.0,     # meno punti di contatto
        "manual_rate": 4.0,     # [mm/s] risposta piu' rapida ai tasti
        "parallel_components": True,   # FEM e collisione multi-core (plugin MultiThreading)
    },
    "visual_amplification": 1.0,   # deformazioni mostrate x50 (1 = disattivata)
    "display_flags": "showVisualModels",
    "live_plot_python": None,   # Interprete del viewer (None = automatico: Python di sistema)
    "record_csv": True,         # Registrazione CSV nella GUI (results/gui_<data_ora>.csv)
    "plot_every": 5,            # Invia un punto ai grafici ogni N passi
}

# Configurazioni della molla: (a) fissa e (b) a due livelli (traccia), piu' l'attuazione diretta
# k = 50 N/mm, riferimento di H (traccia). Nei cicli autonomi la diretta va a 0.1 mm/s (scelta di
# protocollo: 10 N in circa 2 s; a 1 mm/s il carico durerebbe 20 passi e H risentirebbe del
# ritardo dinamico del tubo).
SPRING_CONFIGS = {
    "fixed": {"switched_mode": False},
    "switched": {"switched_mode": True},
    "direct": {"switched_mode": False, "k_spring": 50.0, "auto_rate": 0.1},
}

# Range di perturbazione Monte Carlo (Progetto 5): modulo uniforme tra 10% e 20% con segno casuale
MC_UNCERTAINTY_RANGE = (0.10, 0.20)
MC_NUM_REALIZATIONS = 20

# Parametri incerti della tabella del Progetto 5, raggruppati: ogni gruppo riceve UNA
# perturbazione (modulo uniforme 10-20%, segno casuale) applicata a tutte le sue chiavi.
#   E : E_load ed E_unload con lo stesso fattore (rapporto carico/scarico invariato)
#   k : k fissa e i due livelli 0.6 / 4.8 con lo stesso fattore ("k" e' un solo parametro)
UNCERTAIN_GROUPS = {
    "E": ("E_load", "E_unload"),
    "Fh_mat": ("Fh_mat",),
    "Fh_spring": ("Fh_spring",),
    "c": ("c_spring",),
    "k": ("k_spring", "k_low", "k_high"),
    "yt": ("yt_offset",),
}
