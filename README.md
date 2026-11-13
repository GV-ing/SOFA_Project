# Progetto 5 — Incertezza parametrica: Monte Carlo e analisi di sensibilità

Robotica Medica 2026/27 (prof.ssa F. Ficuciello). Tubo in nitinol azionato da un tendine con
molla in serie, simulato in **SOFA v26.06 (SofaPython3, Python 3.12) su Ubuntu 22.04**.

Si confrontano due configurazioni della molla: **(a)** fissa con k = 1.2 N/mm e **(b)** a due
livelli, **4.8 N/mm in moto libero e 0.6 N/mm in contatto** (`k_free_level = "high"` in
`modules/config.py`). Il contatto è rilevato a 5 mN; da quel momento k commuta istantaneamente
e il motore mantiene la forza di contatto a **5 mN ±10 % (4.5–5.5 mN)** per tutta la durata del
contatto. Per ogni configurazione si simulano il caso nominale e 20 realizzazioni con i parametri
perturbati del ±(10–20) % con segno casuale; su ognuna si calcolano l'isteresi H e la stabilità
del contatto, e una regressione lineare dà il diagramma tornado.

Riferimento per i requisiti: la traccia del Progetto 5 (PDF del corso) e le indicazioni
dell'assistente del corso (forza di contatto mantenuta a 5 mN ±10 %, nessun vincolo su T,
commutazione di k istantanea). Il glossario del corso serve solo a quantificare grandezze che la
traccia nomina; in `modules/config.py` ogni valore riporta l'origine: traccia, assistente o scelta.

## Pipeline completa

| Passo | Comando | Durata indicativa | Produce |
|---|---|---|---|
| 0. Ambiente | `source env.sh` | – | variabili di SOFA per `python3.12` |
| 1. Mesh | `python3.12 tools/generate_meshes.py` | secondi | `meshes/*.obj`, `meshes/*.vtk` |
| 2. Verifica della mesh | `python3.12 -u tools/mesh_convergence.py` | decine di minuti | tabella ΔY SOFA / teoria della trave |
| 3. Scena interattiva | `./run_gui.sh` | – | GUI, grafici live, `results/gui_<data>.csv` |
| 4. Prove nominali | `python3.12 -u tools/run_cycle.py --free --contact` | circa 1 ora | `results/nominali/*.csv`, `*.json` |
| 5. Monte Carlo | `python3.12 -u tools/run_monte_carlo.py --outdir results/mc_finale` | diverse ore | `results/mc_finale/` |
| 6. Figure | `env -u PYTHONPATH python3 tools/make_figures.py` | secondi | `results/figures/*.png` |

La scena rigenera le sue mesh a ogni avvio e il Monte Carlo contiene già i casi nominali delle
configurazioni (a) e (b): il minimo per rifare i risultati è 0 → 5 → 6. Il passo 4 serve anche
per l'ordine della molla (b) non scelto (`--kfree low`), simulato solo nel caso nominale.

### 0. Installazione e ambiente

Prerequisiti:
- SOFA v26.06 (binario per Python 3.12) in `~/SOFA_v26.06.00_Linux_Python3.12/`;
- questo progetto in `~/SOFA_v26.06.00_Linux_Python3.12/SOFA_Project`;
- il Python di sistema con matplotlib, usato per i grafici e le figure.

```bash
sudo apt install python3-matplotlib python3-tk
cd ~/SOFA_v26.06.00_Linux_Python3.12/SOFA_Project
chmod +x run_gui.sh
source env.sh                                    # va ripetuto in ogni nuovo terminale
python3.12 -c "import Sofa.Core, Sofa.Simulation; print('SOFA OK')"     # atteso: SOFA OK
```

`env.sh` e `run_gui.sh` danno per scontati i percorsi indicati sopra. Se SOFA si trova altrove,
basta correggere `SOFA_ROOT` in `env.sh`, oppure lanciare `SOFA_ROOT=/percorso ./run_gui.sh`.

### 1. Generazione delle mesh

```bash
python3.12 tools/generate_meshes.py
```

| File | Contenuto |
|---|---|
| `meshes/tube_surface.obj` | superficie del tubo, modello visivo |
| `meshes/tube_collision.obj` | superficie del tubo, modello di collisione (solo il tratto vicino alla punta) |
| `meshes/tube_fem_surface.obj` | superficie esterna della mesh FEM (controllo) |
| `meshes/tube_volume.vtk` | mesh volumetrica del tubo: 3720 nodi, 11088 tetraedri, più fine (0.4 mm) nella parte flessibile |
| `meshes/soft_volume.vtk` | mesh volumetrica dell'oggetto soffice: 432 esaedri, più fini nella zona di contatto |
| `meshes/object_surface.obj` | superficie dell'oggetto di contatto |

- I file `.vtk` si aprono con ParaView.
- Lo script usa le stesse funzioni della scena (`modules/geometry_gen.py`).
- La scena rigenera le sue mesh a ogni avvio in `meshes/scena/` (cartella di lavoro, ignorata da
  git) con i parametri del momento, compreso il profilo GUI più leggero. Per questo la GUI non va
  aperta mentre girano gli script del passo 4 o il Monte Carlo.

### 2. Verifica della mesh (facoltativa)

```bash
python3.12 -u tools/mesh_convergence.py
```

Lo script prova `mesh_h_flex` = 0.8, 0.4, 0.3 e 0.2 mm. Per ciascun valore confronta la
deflessione della punta con la teoria della trave: momento T·yt dal cavo e forza laterale in
punta. Con 0.4 mm, il valore scelto, ΔY a 10 N vale 0.304 mm contro 0.320 mm teorici (−4.8 %).

### 3. Scena interattiva (GUI)

```bash
./run_gui.sh                                                   # soffice, molla fissa
./run_gui.sh --object rigid --spring switched                  # rigido, molla a due livelli
./run_gui.sh --object soft --spring switched --kfree low       # ordine non scelto
./run_gui.sh --object none                                     # moto libero
./run_gui.sh --spring direct                                   # attuazione diretta k = 50 N/mm
```

Il launcher si può lanciare da qualunque cartella e imposta l'ambiente da solo. Opzioni:
- `--object soft|rigid|none`
- `--spring fixed|switched|direct`
- `--kfree high|low` (ordine dei livelli della molla b; predefinito in `config.py`)

Si aprono la finestra di SOFA e quella dei grafici. Premere **Play**.

| Tasto (finestra SOFA) | Moto libero | In contatto |
|---|---|---|
| Freccia su / `U` / `+` | carico: u + 0.35 mm | ignorato (la forza è regolata); dopo un rilascio riattiva la regolazione |
| Freccia giù / `D` / `-` | scarico: u − 0.35 mm | rilascio: regolazione disattivata e u − 0.35 mm |
| `R` | reset di comando e cavo | reset (la regolazione resta disattivata fino alla prossima freccia su) |
| `M` / `A` | modo manuale ↔ ciclo autonomo | come in moto libero |

Nella finestra dei grafici il tasto `R` azzera i loop e l'energia dissipata.

- A ogni comando il terminale stampa u, T, ΔLs, X, Y e ΔY della punta e la forza di contatto;
  rilevamento e perdita del contatto sono annunciati con il nuovo valore di k.
- Ciclo autonomo (`M`): senza oggetto carica a 1 mm/s fino a T = 10 N o u = 15 mm e scarica; con
  l'oggetto avvicina, regola la forza a 5 mN, mantiene la finestra di 5 s e scarica.
- La finestra dei grafici segue la struttura del Modulo 06 del corso:
  - a sinistra, il loop T–ΔLs della molla (legge di Hooke, banda ±0.85 N, energia dissipata) e il loop ΔY–u della punta con H calcolato in tempo reale;
  - a destra, nel tempo: posizione della punta, u e ΔLs, T, forza di contatto (soglia e riferimento 5 mN, banda 4.5–5.5 mN) e k.
- Ogni passo viene salvato in `results/gui_<data>.csv`.

Da dichiarare nel video:
- **Profilo GUI** (`gui_fast`, `gui_overrides`): mesh più leggere (`mesh_h_flex` 0.8 mm, circa l'85 % della rigidezza convergente; soffice con griglia uniforme 5) e componenti `Parallel…`. `dt` = 0.01 s e il solver dei vincoli (500 iterazioni, tolleranza 1e-7) restano quelli di consegna. Con `"gui_fast": False` la GUI usa il modello completo.
- **Deformazione mostrata ×50** (`visual_amplification`): la forma reale resta visibile in trasparenza.
- La simulazione va più lenta del tempo reale, quindi va dichiarato l'eventuale fattore di accelerazione del video.

### 4. Prove nominali

```bash
python3.12 -u tools/run_cycle.py --free                                  # 4 cicli liberi
python3.12 -u tools/run_cycle.py --contact --objects rigid               # 3 prove sul rigido
python3.12 -u tools/run_cycle.py --contact --objects soft --configs switched --kfree high
```

Le prove sono indipendenti: si possono lanciare in parallelo (al massimo 3 processi nella VM),
con `nohup … > results/nominali/<nome>.log 2>&1 &`.

- **Ciclo libero** (molle fissa, due livelli nei due ordini, diretta): carico fino a T = 10 N o
  u = 15 mm, poi scarico. Le molle vanno a 1 mm/s, la diretta a 0.1 mm/s (10 N in circa 2 s).
- **Prova di contatto** (rigido e soffice; fissa e due livelli): avvicinamento a 2 mm/s →
  rilevamento a 5 mN e commutazione di k → regolazione a 5 mN ±10 % → forza in banda a motore
  fermo per 1 s → finestra di 5 s con il regolatore attivo.
- Per ogni caso: serie temporale `results/nominali/<caso>.csv` (una riga per passo) e metriche
  `results/nominali/<caso>.json` (H, oppure stabilità, banda, tempo di regolazione, picco di forza,
  T prima e dopo la commutazione).

### 5. Campagna Monte Carlo (127 casi)

I 127 casi sono (1 nominale + 20 realizzazioni) × 2 molle (fissa e due livelli nell'ordine di
`config.py`) × 3 prove (libera, rigido, soffice), più 1 caso di attuazione diretta. Seed 2026.

```bash
python3.12 -u tools/run_monte_carlo.py --dry-run --outdir /tmp/prova_mc     # prova senza SOFA (secondi)
nohup python3.12 -u tools/run_monte_carlo.py --outdir results/mc_finale > results/mc_finale.log 2>&1 &
python3.12 -u tools/run_monte_carlo.py --outdir results/mc_finale --resume   # dopo un'interruzione
```

- **Perturbazioni:** sei gruppi di parametri, ciascuno con un solo fattore: E (E_load ed E_unload), Fh_mat, Fh_spring, c, k (k_spring, k_low, k_high) e yt. La soglia di 5 mN e i guadagni del regolatore non vengono ritarati.
- **Ripresa:** ogni caso viene aggiunto a `cases.jsonl` appena finisce, quindi `--resume` riparte dal primo caso mancante.
- **Worker:** 3 processi (predefinito, `mc_workers`); con più processi la VM si è bloccata.
- **Output:**
  - `summary.json`: statistiche di H, tornado, contatti instabili, prove fuori banda, durata massima della regolazione e picco di forza dopo il rilevamento, verifica dei target;
  - `cases.jsonl`: metriche di ogni caso;
  - `realizations.json`: parametri estratti;
  - `series/`: serie temporali (i `.csv` non sono in git, le figure del passo 6 sì).

Riepilogo rapido dal terminale:

```bash
python3 -c "import json; s=json.load(open('results/mc_finale/summary.json'))
for c,v in s['configs'].items(): print(c, {k: v[k] for k in ('H_nominal_pct','H_mean_pct','H_std_pct','H_worst_pct','robustness_index','unstable_total','contact_out_of_band','dominant_parameters','meets_target')})
print('H diretta:', s['H_direct_nominal_pct'], '| piu robusta:', s['most_robust_config'])"
```

### 6. Figure

```bash
env -u PYTHONPATH python3 tools/make_figures.py --campaign results/mc_finale --out results/figures
```

Figure prodotte: `fig_tornado`, `fig_H_realizzazioni`, `fig_forze_contatto` (forza nella
finestra e banda 4.5–5.5 mN), `fig_isteresi_nominale` (comprende l'ordine non scelto, se esiste
`results/nominali/`) e `fig_contatto_nominale`. Lo script usa il Python di sistema (matplotlib),
non serve SOFA.

## Risultati

### Isteresi nominale in moto libero (`results/nominali`, 13 novembre 2026)

| | H [% di ΔYmax] | H / H diretta | inversione |
|---|---:|---:|---|
| (b) 4.8 libero / 0.6 in contatto (scelto) | **12.42** | **0.58** | T = 10 N a u = 1.93 mm |
| (a) fissa 1.2 N/mm | 14.39 | 0.68 | T = 10 N a u = 7.65 mm |
| (b) 0.6 libero / 4.8 in contatto | 16.00 | 0.75 | u = 15 mm, T = 9.84 N |
| attuazione diretta 50 N/mm | 21.26 | 1 | T = 10 N a u = 0.20 mm |

**Scelta dell'ordine della molla (b).** L'ordine 4.8 → 0.6 ha la H più bassa, è cedevole in
contatto (1 mm di motore cambia la forza di circa 0.6 mN sul soffice, contro circa 6 mN con
4.8 N/mm) e al rilevamento la forza scende invece di salire: con l'ordine opposto la tensione
passa da circa 5 a circa 35 N in un passo e la forza sul soffice arriva a 44 mN. Il difetto
dell'ordine scelto è allo scarico, quando il contatto si perde (vedi i limiti).

### Monte Carlo

Da aggiornare con la campagna `results/mc_finale` (configurazioni fissa e 4.8 → 0.6).

## Requisiti della traccia

| Requisito | Implementazione |
|---|---|
| Superficie OBJ per i modelli visivo e di collisione, mesh volumetrica per la meccanica | `modules/geometry_gen.py`, `tools/generate_meshes.py` (passo 1) |
| FEM con E diverso in carico e scarico | `TetrahedronFEMForceField`; il controller passa da E_load a E_unload secondo il verso del motore (`reinit()`) |
| `BarycentricMapping`, `CableConstraint` | `main_scene.py`: tendine con yt = 0.6 mm, ancorato alla punta |
| Molla in serie T = k·ΔLs ± isteresi, con smorzamento | `modules/spring_model.py`: isteresi di Dahl (σ0 = 10/mm, ±(Fh_mat + Fh_spring)), smorzamento c sul moto del tendine; solo T ≥ 0, nessun altro vincolo su T |
| Molla a due livelli commutata dal rilevamento del contatto (5 mN) | `modules/contact_control.py: ContactState` (CONTATTO a F ≥ 5 mN, LIBERO a F < 0.1 mN); k fisso per tutto il contatto, commutazione istantanea |
| Forza di contatto mantenuta a 5 mN ±10 % (assistente) | `modules/contact_control.py: ForceRegulator` (banda morta, dT/dt proporzionale all'errore) |
| Controller da tastiera (su, giù, reset, manuale/autonomo, stampa u, T, ΔLs, punta) | `modules/controller.py` (`Sofa.Core.Controller`) |
| Grafici in tempo reale + CSV | `tools/live_plot_viewer.py`, `modules/live_plot.py` (passo 3) |
| Oggetti rigido e soffice (5 kPa, ν 0.45), `FrictionContactConstraint` μ = 0.2 | `main_scene.py`; stessa distanza iniziale (0.03 mm) per i due oggetti |
| H = max ΔY carico−scarico a pari u, in % di ΔYmax, rispetto a k = 50 N/mm | `modules/metrics.py: hysteresis_H`; caso `direct` |
| Contatto stabile: picco-picco < 30 % della media per 5 s, senza perdita | `modules/metrics.py: contact_stability` (verifica anche la banda 4.5–5.5 mN) |
| 2 configurazioni × 20 realizzazioni ±(10–20) %, commutazione non ritarata | `modules/monte_carlo.py`, `tools/run_monte_carlo.py` (passo 5) |
| Media, dev. std, caso peggiore, indice di robustezza, tornado, contatti instabili | `results/mc_finale/summary.json`, `tools/make_figures.py` (passo 6) |

## Struttura

```text
main_scene.py          scena SOFA (createScene); profilo GUI se lanciata da runSofa
run_gui.sh, env.sh     launcher della GUI / ambiente per gli script python3.12
meshes/                mesh nominali (passo 1); meshes/scena/ = mesh di lavoro della scena
modules/
  config.py            parametri nominali con l'origine di ogni valore, configurazioni della molla,
                       gruppi incerti, profilo GUI
  geometry_gen.py      mesh del tubo e dell'oggetto, export OBJ e VTK
  spring_model.py      molla in serie: Dahl, smorzamento, k fisso o a due livelli
  contact_control.py   stato di contatto (macchina a stati) e regolatore della forza
  controller.py        controller da tastiera e ciclo autonomo, registrazione dei dati
  contact_force.py     forza di contatto da constraintForces del solver
  metrics.py           H, stabilità del contatto e banda di forza
  protocols.py         ciclo libero e prova di contatto (usati da run_cycle e Monte Carlo)
  monte_carlo.py       campionamento, regressione e tornado, riepilogo
  mc_worker.py         esecuzione di un caso del Monte Carlo
  live_plot.py         invio dei dati ai grafici, registrazione CSV (solo GUI)
  amplified_view.py    vista della deformazione amplificata (solo GUI)
tools/                 generate_meshes, mesh_convergence, run_cycle, run_monte_carlo,
                       make_figures, live_plot_viewer
results/               nominali/, mc_finale/, figures/ (create dai passi 4, 5 e 6)
```

## Ipotesi e limiti

- **Unità:** mm, N, s, MPa, massa in tonnellate.
- **Convenzione SOFA:** con `FreeMotionAnimationLoop`, valore del cavo e forze di vincolo sono impulsi, quindi si impone T·dt e si legge λ/dt.
- **Forza di contatto:** si legge dalle `constraintForces` del `BlockGaussSeidelConstraintSolver`, perché il `lambda` dei modelli di collisione mappati conta la forza due volte.
- **Glossario contro traccia:** dove i valori differiscono (E_unload 66 contro 65 GPa, ν 0.30 contro 0.33, k 1.13 contro 1.2 N/mm, parete 0.215 contro 0.22 mm) si sono usati i valori della traccia.
- **Mesh del soffice non del tutto convergente:** dimezzando il passo la forza cambia di circa −15 %. I confronti tra configurazioni restano validi; le forze assolute sul soffice sono approssimate.
- **Due isteresi di segno opposto in H:** l'isteresi di Dahl porta lo scarico sotto il carico (fino a 1.7 N di T), il passaggio a E_unload lo porta sopra (+18 % di cedevolezza). La prima domina a T bassa, la seconda a T alta: con le molle rigide e con la diretta l'anello si incrocia, e con 4.8 N/mm i due effetti si compensano in parte.
- **Ritardo dinamico del tubo:** con `rayleighStiffness` = 0.1 la punta segue T con circa 0.1 s di ritardo (misurato: −15 µm nel carico a 4.8 N/s). H dipende quindi in parte dalla velocità di variazione di T (1.2 N/s per la fissa, circa 5 N/s per 4.8 N/mm e per la diretta). Compensando il ritardo, H vale circa 16.9 % (fissa), 17.2 % (0.6 → 4.8) e 15.3 % (4.8 → 0.6): la graduatoria tra le molle non cambia.
- **Transitorio dopo il rilevamento:** la forza esce dalla banda per un tempo breve, per due motivi. Al primo contatto con il rigido la forza nasce già a 7.7–14 mN, perché il tubo in moto libero è in ritardo e tocca l'oggetto con T già a 0.9–1.3 N. Alla commutazione di k la forza salta (in giù con l'ordine scelto, fino a circa 1 mN sul soffice). La banda è verificata nella finestra di 5 s; durata del transitorio e picco di forza sono riportati a parte (`regulation_s`, `F_peak_N`).
- **Regolatore di forza:** in contatto la forza segue T entro 1–2 passi (misurato in SOFA), quindi si usa `force_gain` = 1000 s⁻¹ (sul rigido force_gain · dF/dT · dt = 0.15). Una correzione continua (PI) andrebbe in ciclo limite per l'isteresi di Dahl (circa 1.7 N di T a ogni inversione del motore): per questo il regolatore ferma il motore quando la forza è entro 0.1 mN dal riferimento e riparte solo se se ne allontana di più di 0.3 mN.
- **Perdita del contatto con l'ordine scelto:** allo scarico, quando il contatto si perde, k torna da 0.6 a 4.8 N/mm a pari allungamento e T risale di colpo (da circa 0.25 a circa 8 N sul soffice), così la punta ritocca brevemente l'oggetto. Avviene dopo la finestra di valutazione e non entra nelle metriche; nel ciclo autonomo un nuovo rilevamento durante lo scarico non riattiva la regolazione.
- **Passaggio E_load → E_unload istantaneo:** all'inversione del motore la punta si sposta di colpo (circa +18 % di cedevolezza).
- **Smorzamento c quasi ininfluente su H:** agisce sulla velocità del tendine, che è piccola perché il tubo è rigido.
