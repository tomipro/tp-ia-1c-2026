#!/usr/bin/env bash
# Prepara el entorno del TP:
#   1) clona los repos de Unitree (si faltan)
#   2) crea el entorno virtual (Python 3.10)
#   3) instala las dependencias
#   4) aplica los parches de compatibilidad (macOS)
#
# Uso:  bash setup.sh
#       PY310=/ruta/a/python3.10 bash setup.sh   # si no lo encuentra
set -e
cd "$(dirname "$0")"   # carpeta TP/

# --- localizar Python 3.10 ---
PY310="${PY310:-}"
if [ -z "$PY310" ]; then
  for c in python3.10 /opt/homebrew/bin/python3.10 \
           /Library/Frameworks/Python.framework/Versions/3.10/bin/python3.10; do
    if command -v "$c" >/dev/null 2>&1; then PY310="$c"; break; fi
  done
fi
if [ -z "$PY310" ]; then
  echo "ERROR: no encontré Python 3.10 (requerido por cyclonedds==0.10.2)."
  echo "       Instalalo o pasá la ruta: PY310=/ruta/a/python3.10 bash setup.sh"
  exit 1
fi
echo "Usando Python: $PY310"

# --- 1) clonar repos de Unitree ---
echo
echo "[1/4] Clonando repos de Unitree (si faltan)..."
clone() { [ -d "$1" ] || git clone --depth 1 "$2" "$1"; }
clone unitree_mujoco      https://github.com/unitreerobotics/unitree_mujoco.git
clone unitree_sdk2        https://github.com/unitreerobotics/unitree_sdk2.git
clone unitree_sdk2_python https://github.com/unitreerobotics/unitree_sdk2_python.git

# --- 2) entorno virtual ---
echo
echo "[2/4] Creando entorno virtual (venv310)..."
[ -d venv310 ] || "$PY310" -m venv venv310
./venv310/bin/python -m pip install --upgrade pip setuptools wheel

# --- 3) dependencias ---
echo
echo "[3/4] Instalando dependencias..."
./venv310/bin/python -m pip install "cyclonedds==0.10.2" numpy mujoco opencv-python pygame
./venv310/bin/python -m pip install -e ./unitree_sdk2_python

# --- 4) parches ---
echo
echo "[4/4] Aplicando parches de compatibilidad (SDK + bridge)..."
./venv310/bin/python tp_g1/patch_macos.py

echo
echo "Listo. Para usar:"
echo "  Terminal 1:  bash tp_g1/run_sim.sh"
echo "  Terminal 2:  bash tp_g1/run_control.sh"
