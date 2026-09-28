#!/usr/bin/env bash
# Abre el simulador MuJoCo con el G1.
# Uso:  bash run_sim.sh [--headless SEG] [--no-harness]
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TP="$(cd "$DIR/.." && pwd)"
PY="$TP/venv310/bin/python"
# En macOS el visor de MuJoCo requiere mjpython. Se invoca por interprete
# porque la ruta del proyecto tiene espacios y rompe el shebang de mjpython.
exec "$PY" -u "$TP/venv310/bin/mjpython" "$DIR/g1_sim.py" "$@"
