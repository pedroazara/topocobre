"""
Corrige o Z de um G-code a partir de um mapa de altura medido manualmente.

Uso:
    python nivelar_gcode.py isolacao.nc mapa.csv
    python nivelar_gcode.py isolacao.nc mapa.csv -o saida.nc --passo 1.0

mapa.csv (grade regular, Z medido no zero de trabalho, em mm):
    x,y,z
    0,0,0.00
    17,0,-0.04
    ...

Para cada ponto do percurso: Z_novo = Z_original + h(x, y),
com h interpolado bilinearmente na grade (extrapolado fora dela).
Movimentos longos são subdivididos em trechos <= --passo mm,
senão o meio de uma reta comprida não seria corrigido.

Limitações: só G0/G1, coordenadas absolutas (G90), milímetros.
"""

import argparse
import math
import re
import sys

import numpy as np
from scipy.interpolate import RegularGridInterpolator


# ---------------------------------------------------------------------------
# Mapa de altura
# ---------------------------------------------------------------------------
def carregar_mapa(caminho):
    dados = np.loadtxt(caminho, delimiter=",", skiprows=1, ndmin=2)
    xs = np.unique(dados[:, 0])
    ys = np.unique(dados[:, 1])

    if len(dados) != len(xs) * len(ys):
        raise ValueError(f"Erro: o mapa precisa ser uma grade completa "
                 f"({len(xs)} x {len(ys)} = {len(xs) * len(ys)} pontos), "
                 f"mas tem {len(dados)}.")
    if len(xs) < 2 or len(ys) < 2:
        raise ValueError("Erro: a grade precisa de pelo menos 2 valores em X e 2 em Y.")

    Z = np.zeros((len(xs), len(ys)))
    for x, y, z in dados:
        Z[np.searchsorted(xs, x), np.searchsorted(ys, y)] = z

    interp = RegularGridInterpolator((xs, ys), Z, method="linear",
                                     bounds_error=False, fill_value=None)
    return lambda x, y: float(interp((x, y)))


# ---------------------------------------------------------------------------
# G-code
# ---------------------------------------------------------------------------
PALAVRA = re.compile(r"([A-Z])\s*(-?\d*\.?\d+)")


def remover_comentarios(linha):
    linha = re.sub(r"\(.*?\)", "", linha)
    return linha.split(";")[0]


def nivelar(entrada, saida, h, passo_max):
    x = y = z = 0.0
    modo = 0
    xy_conhecido = False
    correcoes = []

    with open(entrada) as f_in, open(saida, "w") as f_out:
        for num, linha in enumerate(f_in, 1):
            codigo = remover_comentarios(linha).upper()
            palavras = PALAVRA.findall(codigo)
            gs = [int(float(v)) for k, v in palavras if k == "G"]

            if any(g in (2, 3) for g in gs):
                raise ValueError(f"Erro na linha {num}: arcos (G2/G3) não são suportados.")
            if 91 in gs:
                raise ValueError(f"Erro na linha {num}: modo relativo (G91) não é suportado.")
            if 20 in gs:
                raise ValueError(f"Erro na linha {num}: G-code em polegadas (G20).")

            for g in gs:
                if g in (0, 1):
                    modo = g

            eixos = {k: float(v) for k, v in palavras if k in "XYZ"}
            if not eixos or modo not in (0, 1):
                f_out.write(linha)
                continue

            feed = next((float(v) for k, v in palavras if k == "F"), None)

            # Antes do primeiro movimento em XY a posição real é desconhecida:
            # repassa a linha sem mexer, para não gerar um X0 Y0 inventado
            # (que faria a máquina andar na diagonal enquanto sobe o Z).
            if not xy_conhecido and not ("X" in eixos and "Y" in eixos):
                f_out.write(linha)
                z = eixos.get("Z", z)
                continue
            xy_conhecido = True

            x1 = eixos.get("X", x)
            y1 = eixos.get("Y", y)
            z1 = eixos.get("Z", z)

            dist_xy = math.hypot(x1 - x, y1 - y)
            n = max(1, math.ceil(dist_xy / passo_max))

            for i in range(1, n + 1):
                t = i / n
                xi = x + (x1 - x) * t
                yi = y + (y1 - y) * t
                zi = z + (z1 - z) * t
                dz = h(xi, yi)
                correcoes.append(dz)

                nova = f"G{modo} X{xi:.4f} Y{yi:.4f} Z{zi + dz:.4f}"
                if feed is not None and i == 1:
                    nova += f" F{feed:g}"
                f_out.write(nova + "\n")

            x, y, z = x1, y1, z1

    return correcoes


def main():
    p = argparse.ArgumentParser(description="Nivela o Z de um G-code com mapa de altura.")
    p.add_argument("gcode", help="arquivo .nc de entrada")
    p.add_argument("mapa", help="CSV com colunas x,y,z")
    p.add_argument("-o", "--saida", help="arquivo de saída (padrão: <entrada>_nivelado.nc)")
    p.add_argument("--passo", type=float, default=1.0,
                   help="comprimento máximo de cada trecho em mm (padrão: 1.0)")
    args = p.parse_args()

    saida = args.saida or re.sub(r"(\.\w+)?$", r"_nivelado\1", args.gcode, count=1)
    # As funções levantam ValueError (para o app poder mostrar a mensagem
    # sem ser derrubado); aqui na linha de comando vira saída com erro.
    try:
        h = carregar_mapa(args.mapa)
        corr = nivelar(args.gcode, saida, h, args.passo)
    except ValueError as e:
        sys.exit(str(e))

    if corr:
        print(f"Salvo em: {saida}")
        print(f"Correção aplicada: min {min(corr):+.3f} mm, max {max(corr):+.3f} mm")
    else:
        print("Nenhum movimento encontrado no arquivo.")


if __name__ == "__main__":
    main()
