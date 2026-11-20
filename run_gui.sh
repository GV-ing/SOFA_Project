#!/usr/bin/env bash
# run_gui.sh: avvia la scena del Progetto 5 in runSofa da qualunque cartella
#
# Uso:
#   ./run_gui.sh                                   oggetto soffice, molla fissa 1.2 N/mm
#   ./run_gui.sh --object rigid --spring switched  oggetto rigido, molla a due livelli
#   ./run_gui.sh --object none                     moto libero
#
# Opzioni:  --object soft|rigid|none   --spring fixed|switched|direct
#           --kfree high|low   livello di k in moto libero per la molla switched
#           gli altri argomenti passano a runSofa; SOFA_ROOT=... per un'altra installazione
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOFA_BASE="$(dirname "$PROJECT_DIR")"     # cartella che contiene SOFA e SOFA_Project
export SOFA_ROOT="${SOFA_ROOT:-$SOFA_BASE/SOFA_v26.06.00_Linux}"

# lettura delle opzioni
EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --object) export P5_OBJECT="$2"; shift 2 ;;
    --spring) export P5_SPRING="$2"; shift 2 ;;
    --kfree) export P5_K_FREE="$2"; shift 2 ;;
    -h|--help) sed -n '2,11p' "$0"; exit 0 ;;
    *) EXTRA+=("$1"); shift ;;
  esac
done
case "${P5_OBJECT:-soft}" in soft|rigid|none) ;; *) echo "--object: soft|rigid|none"; exit 1 ;; esac
case "${P5_SPRING:-fixed}" in fixed|switched|direct) ;; *) echo "--spring: fixed|switched|direct"; exit 1 ;; esac

# controllo della cartella di SOFA
if [[ ! -d "$SOFA_ROOT" ]]; then
  echo "SOFA non trovato in $SOFA_ROOT. Impostare SOFA_ROOT=/percorso/SOFA_v26.06.00_Linux"
  exit 1
fi
# percorsi di SofaPython3, STLIB, SoftRobots e del progetto
SP="$SOFA_ROOT/plugins"
export PYTHONPATH="$SP/SofaPython3/lib/python3/site-packages:$SP/STLIB/lib/python3/site-packages:$SP/SoftRobots/lib/python3/site-packages:$PROJECT_DIR${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$SOFA_ROOT/lib:$SP/SofaPython3/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# avvio con il launcher se c'è, altrimenti con runSofa
SCENE="$PROJECT_DIR/main_scene.py"
WRAPPER="$SOFA_BASE/SOFA/run_sofa.sh"     # launcher di SOFA, se presente
echo "[run_gui] scena: $SCENE | oggetto: ${P5_OBJECT:-soft} | molla: ${P5_SPRING:-fixed}"
if [[ -x "$WRAPPER" ]]; then
  cd "$(dirname "$WRAPPER")"
  exec "$WRAPPER" "$SCENE" "${EXTRA[@]}"
elif [[ -x "$SOFA_ROOT/bin/runSofa" ]]; then
  cd "$PROJECT_DIR"
  exec "$SOFA_ROOT/bin/runSofa" -l SofaPython3 "$SCENE" "${EXTRA[@]}"
else
  echo "Nessun runSofa trovato ($WRAPPER oppure $SOFA_ROOT/bin/runSofa)"
  exit 1
fi
