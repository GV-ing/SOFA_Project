"""
modules/config.py

Parametri del Progetto 5: geometria, materiale, molla, contatto, controllo, Monte Carlo e
simulazione. Unità usate in SOFA: mm, N, MPa (N/mm^2), s.

Origine dei valori:
    (*)     traccia del progetto
    (**)    glossario del corso
    (***)   scelta di progetto
"""

NOMINAL_PARAMS = {
    # geometria del tubo
    "od": 1.93,                 # (*)   diametro esterno [mm]
    "thickness": 0.22,          # (*)   spessore di parete [mm], ID = 1.49 mm
    "length_total": 100.0,      # (*)   lunghezza totale [mm]
    "length_shaft": 40.0,       # (*)   albero prossimale vincolato [mm]
    "length_flex": 60.0,        # (*)   parte flessibile [mm]

    # discretizzazione delle mesh
    "mesh_h_flex": 0.40,        # (***) passo assiale della mesh FEM sulla parte flessibile [mm]
    "mesh_n_shaft": 4,          # (***) elementi sull'albero
    "mesh_n_theta": 12,         # (***) suddivisioni lungo la circonferenza
    "visual_h": 2.0,            # (***) passo assiale della superficie visiva [mm]
    "collision_h": 1.0,         # (***) passo assiale della superficie di collisione [mm]
    "collision_margin": 5.0,    # (***) la collisione parte da L - obj_size - collision_margin [mm]

    # materiale del tubo (Nitinol)
    "E_load": 77000.0,          # (*)   modulo di Young in carico [MPa]
    "E_unload": 65000.0,        # (*)   modulo di Young in scarico [MPa]
    "poisson": 0.33,            # (*)   coefficiente di Poisson
    "Fh_mat": 0.40,             # (*)   isteresi del materiale [N]

    # tendine
    "yt_offset": 0.60,          # (*)   offset del tendine lungo Y [mm]

    # molla in serie
    "L0_spring": 20.0,          # (*)   lunghezza a riposo [mm], non usata dal modello
    "k_spring": 1.2,            # (*)   config. (a): rigidezza fissa [N/mm]
    "k_low": 0.6,               # (*)   config. (b): livello basso [N/mm]
    "k_high": 4.8,              # (*)   config. (b): livello alto [N/mm]
    # config. (b): "high" = 4.8 N/mm in moto libero e 0.6 in contatto, "low" = il contrario
    "k_free_level": "high",     # (***) livello di k usato in moto libero
    "c_spring": 1.5,            # (*)   smorzamento [N*s/mm], agisce sul moto del tendine
    "Fh_spring": 0.45,          # (*)   isteresi della molla [N]
    "dahl_sigma0": 10.0,        # (**)  rigidezza di Dahl dell'isteresi [1/mm]
    "switched_mode": False,     # (***) False = config. (a) k fissa, True = config. (b) due livelli

    # attuazione e ciclo autonomo
    "u_step": 0.35,             # (*)   passo del comando per ogni pressione di tasto [mm]
    "u_max": 15.0,              # (**)  corsa massima del carrello [mm]
    "manual_rate": 1.0,         # (**)  velocità del carrello nei comandi manuali [mm/s]
    "auto_rate": 1.0,           # (**)  velocità del carrello nel ciclo autonomo [mm/s]
    "auto_T_reverse": 10.0,     # (***) T di inversione del ciclo libero (o u = u_max) [N]
    "auto_hold_s": 0.0,         # (***) pausa a fine carico nel ciclo libero [s]
    "auto_repeat": False,       # (***) True = ciclo autonomo ripetuto, False = singolo

    # contatto e regolazione della forza
    "F_contact_th": 0.005,      # (*)   soglia di rilevamento del contatto [N]
    "contact_loss_force": 1e-4, # (***) sotto questa forza il contatto è perso [N]
    "force_ref": 0.005,         # (***) forza di contatto da mantenere [N]
    "force_band": 0.10,         # (***) banda ammessa attorno a force_ref (+/-10 %)
    "force_reg_start": 0.06,    # (***) il regolatore parte se |F - force_ref| > 6 % di force_ref
    "force_reg_stop": 0.02,     # (***) il regolatore si ferma se |F - force_ref| < 2 % di force_ref
    "force_gain": 1000.0,       # (***) guadagno: dT/dt = -force_gain * (F - force_ref) [1/s]
    "contact_rate": 2.0,        # (**)  velocità di avvicinamento e massima del regolatore [mm/s]
    "contact_settle_s": 1.0,    # (***) tempo con F in banda a motore fermo prima della finestra [s]
    "hold_duration": 5.0,       # (*)   durata della finestra di stabilità [s]
    "stability_ptp_max": 0.30,  # (*)   oscillazione picco-picco massima rispetto alla media

    # oggetto di contatto
    "contact_object": "soft",   # (***) "soft" | "rigid" | "none"
    "obj_size": 20.0,           # (*)   dimensione caratteristica dell'oggetto [mm]
    "obj_gap": 0.03,            # (***) gap punta-oggetto lungo Y [mm], uguale per i due oggetti
    "obj_overlap_x": 5.0,       # (***) sporgenza dell'oggetto oltre la punta lungo X [mm]
    "obj_grid_n": 5,            # (***) nodi per lato della griglia uniforme dell'oggetto soffice
    "soft_mesh": "graded",      # (***) "graded" (più fitta al contatto) | "uniform"
    "soft_h_min": 1.6,          # (***) passo minimo della mesh graduata, vicino al contatto [mm]
    "soft_h_max": 5.0,          # (***) passo massimo, lontano dal contatto [mm]
    "soft_growth": 1.4,         # (***) fattore di crescita del passo
    "E_soft": 0.005,            # (*)   modulo di Young dell'oggetto soffice [MPa] (5 kPa)
    "poisson_soft": 0.45,       # (*)   coefficiente di Poisson dell'oggetto soffice
    "mu_friction": 0.2,         # (*)   coefficiente di attrito
    "alarm_distance": 0.30,     # (***) alarmDistance di LocalMinDistance [mm]
    "contact_distance": 0.02,   # (***) contactDistance di LocalMinDistance [mm]
    "force_scale": 1.0,         # (***) conversione della forza letta dal solver in N
    "random_seed": 2026,        # (***) seed delle realizzazioni Monte Carlo
    "mc_workers": 3,            # (***) processi paralleli del Monte Carlo

    # masse dalla densità (unità mm-N-s: densità in t/mm^3)
    "rho_tube": 6.45e-9,        # (**)  densità del Nitinol (6450 kg/m^3)
    "rho_soft": 1.0e-9,         # (***) densità dell'oggetto soffice (1000 kg/m^3)

    # modelli di collisione: "Triangle", "Line", "Point"
    "tube_collision_models": ("Triangle", "Line", "Point"),     # (***)
    "object_collision_models": ("Triangle", "Line", "Point"),   # (***)

    # simulazione e output
    "dt": 0.01,                 # (**)  passo di integrazione [s]
    "cs_max_iterations": 500,   # (***) iterazioni massime del BlockGaussSeidelConstraintSolver
    "cs_tolerance": 1e-7,       # (***) tolleranza del BlockGaussSeidelConstraintSolver
    "live_plot": True,          # (***) grafici in tempo reale nella GUI
    "gui_fast": True,           # (***) usa gui_overrides quando la scena parte da runSofa
    # valori che nella GUI sostituiscono quelli nominali
    "gui_overrides": {
        "mesh_h_flex": 0.8,     # (***) mesh del tubo più grossolana
        "soft_mesh": "uniform", "obj_grid_n": 5,   # (***) oggetto soffice a griglia uniforme
        "collision_h": 2.0,     # (***) superficie di collisione più grossolana
        "manual_rate": 4.0,     # (***) velocità del carrello con i tasti [mm/s]
        "parallel_components": True,   # (***) componenti paralleli del plugin MultiThreading
    },
    "visual_amplification": 50.0,   # (***) amplificazione delle deformazioni (1 = disattivata)
    "display_flags": "showVisualModels",   # (***) displayFlags del VisualStyle
    "live_plot_python": None,   # (***) interprete per i grafici live (None = ricerca automatica)
    "record_csv": False,        # (***) salva un CSV della sessione GUI in results/
    "plot_every": 5,            # (***) invia un campione ai grafici ogni N passi
}

# configurazioni della molla: (a) fissa, (b) a due livelli, diretta (riferimento per H)
SPRING_CONFIGS = {
    "fixed": {"switched_mode": False},      # (*)   config. (a)
    "switched": {"switched_mode": True},    # (*)   config. (b)
    # (*) attuazione diretta k = 50 N/mm, (***) velocità del carrello 0.1 mm/s nel ciclo libero
    "direct": {"switched_mode": False, "k_spring": 50.0, "auto_rate": 0.1},
}

MC_UNCERTAINTY_RANGE = (0.10, 0.20)     # (*)   modulo della perturbazione, uniforme tra 10 % e 20 %
MC_NUM_REALIZATIONS = 20                # (*)   numero di realizzazioni

# parametri incerti: ogni gruppo riceve una sola perturbazione (segno casuale), applicata a
# tutte le sue chiavi
UNCERTAIN_GROUPS = {
    "E": ("E_load", "E_unload"),            # (*)   stesso fattore su carico e scarico
    "Fh_mat": ("Fh_mat",),                  # (*)
    "Fh_spring": ("Fh_spring",),            # (*)
    "c": ("c_spring",),                     # (*)
    "k": ("k_spring", "k_low", "k_high"),   # (*)   stesso fattore su k fissa e sui due livelli
    "yt": ("yt_offset",),                   # (*)
}
