# env.sh: variabili d'ambiente per SOFA v26.06 (Python 3.12) e cartella del progetto
#
# Uso:
#   source env.sh   (da qualunque cartella)
export SOFA_ROOT="$HOME/SOFA_v26.06.00_Linux_Python3.12/SOFA_v26.06.00_Linux"
export SOFA_PROJECT="$HOME/SOFA_v26.06.00_Linux_Python3.12/SOFA_Project"
export PYTHONHOME="$HOME/.local/python-3.12"
export PATH="$PYTHONHOME/bin:$PATH"
export PYTHONPATH="$SOFA_ROOT/plugins/SofaPython3/lib/python3/site-packages:$SOFA_ROOT/plugins/STLIB/lib/python3/site-packages:$SOFA_ROOT/plugins/SoftRobots/lib/python3/site-packages:$SOFA_PROJECT:$PYTHONPATH"
export LD_LIBRARY_PATH="$PYTHONHOME/lib:$SOFA_ROOT/lib:$SOFA_ROOT/plugins/SofaPython3/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cd "$SOFA_PROJECT" || return
echo "Ambiente SOFA pronto: usare python3.12 (cartella: $SOFA_PROJECT)"
