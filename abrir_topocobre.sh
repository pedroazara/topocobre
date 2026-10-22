#!/usr/bin/env bash
# Abre o TopoCobre. Instala as dependências na primeira vez.
cd "$(dirname "$0")"
python3 -c "import serial, numpy, scipy" 2>/dev/null || python3 -m pip install --user -r requirements.txt
python3 -c "import tkinter" 2>/dev/null || { echo "Falta o tkinter: sudo apt install python3-tk"; exit 1; }
python3 topocobre.py
