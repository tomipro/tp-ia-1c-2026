#!/usr/bin/env bash
# Envia comandos de bajo nivel al G1 (simulador o robot real).
# Uso:
#   bash run_control.sh                # simulador, demo completa
#   bash run_control.sh wave           # simulador, solo saludar
#   bash run_control.sh squat
#   bash run_control.sh real en0 demo  # robot real por la interfaz en0
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TP="$(cd "$DIR/.." && pwd)"
PY="$TP/venv310/bin/python"
if [ "$#" -eq 0 ]; then
  set -- sim demo
fi
exec "$PY" -u "$DIR/g1_control.py" "$@"
