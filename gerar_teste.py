"""
Gera G-code de teste do nivelamento.

  aneis   (padrão) retângulos concêntricos, cada um um sulco fechado. O cobre
          entre dois sulcos vira um anel isolado: multímetro entre anéis
          vizinhos SEM apito = aquele sulco atravessou o cobre na volta inteira.
  espiral um único percurso retangular em espiral, sem levantar a fresa.
          Serve para inspeção visual: com V-bit a largura do sulco acompanha a
          profundidade, então largura constante = profundidade constante.

Uso:
    python gerar_teste.py 50 50                      # anéis, 0.05 mm
    python gerar_teste.py 50 50 --prof 0.07 --espaco 2.5
    python gerar_teste.py 50 50 --tipo espiral -o espiral.nc

Depois: no TopoCobre, "Aplicar no G-code" com este arquivo, e rode o
_nivelado.nc no gSender com o mesmo zero usado na medição.
"""

import argparse


def retangulos(largura, altura, margem, espaco):
    """Lista de retângulos (x0, y0, x1, y1) do mais externo para o mais interno."""
    x0, y0, x1, y1 = margem, margem, largura - margem, altura - margem
    rets = []
    while x1 - x0 > 0 and y1 - y0 > 0:
        rets.append((x0, y0, x1, y1))
        x0, y0, x1, y1 = x0 + espaco, y0 + espaco, x1 - espaco, y1 - espaco
    return rets


def caminho_espiral(largura, altura, margem, espaco):
    """Vértices de uma espiral retangular contínua, de fora para dentro."""
    x0, y0, x1, y1 = margem, margem, largura - margem, altura - margem
    pts = [(x0, y0)]
    while x1 - x0 > 0 and y1 - y0 > 0:
        pts += [(x1, y0), (x1, y1), (x0, y1)]
        if y1 - (y0 + espaco) <= 0 or x1 - (x0 + espaco) <= 0:
            pts.append((x0, y0 + espaco) if y0 + espaco < y1 else (x0, y1))
            break
        pts.append((x0, y0 + espaco))
        x0, y0, x1, y1 = x0 + espaco, y0 + espaco, x1 - espaco, y1 - espaco
        pts.append((x0, y0))
    return pts


def gerar(args):
    L = []
    w = L.append
    w(f"(teste de nivelamento: {args.tipo}, {args.largura}x{args.altura} mm, "
      f"prof {args.prof} mm, espaco {args.espaco} mm)")
    w("G21")
    w("G90")
    w("G94")
    w(f"G0 Z{args.zseg:.3f}")
    if args.rpm > 0:
        w(f"M3 S{args.rpm:g}")
        w("G4 P3")                       # espera o spindle pegar rotação
    z = -abs(args.prof)

    if args.tipo == "aneis":
        rets = retangulos(args.largura, args.altura, args.margem, args.espaco)
        for x0, y0, x1, y1 in rets:
            w(f"G0 X{x0:.3f} Y{y0:.3f}")
            w(f"G1 Z{z:.3f} F{args.feed_z:g}")
            w(f"G1 X{x1:.3f} Y{y0:.3f} F{args.feed:g}")
            w(f"G1 X{x1:.3f} Y{y1:.3f}")
            w(f"G1 X{x0:.3f} Y{y1:.3f}")
            w(f"G1 X{x0:.3f} Y{y0:.3f}")
            w(f"G0 Z{args.zseg:.3f}")
        resumo = f"{len(rets)} anéis"
    else:
        pts = caminho_espiral(args.largura, args.altura, args.margem, args.espaco)
        w(f"G0 X{pts[0][0]:.3f} Y{pts[0][1]:.3f}")
        w(f"G1 Z{z:.3f} F{args.feed_z:g}")
        w(f"G1 X{pts[1][0]:.3f} Y{pts[1][1]:.3f} F{args.feed:g}")
        for x, y in pts[2:]:
            w(f"G1 X{x:.3f} Y{y:.3f}")
        w(f"G0 Z{args.zseg:.3f}")
        resumo = f"espiral com {len(pts)} vértices"

    if args.rpm > 0:
        w("M5")
    w("G0 X0 Y0")
    return "\n".join(L) + "\n", resumo


def main():
    p = argparse.ArgumentParser(description="G-code de teste do nivelamento (anéis ou espiral).")
    p.add_argument("largura", type=float, help="largura da placa em X (mm)")
    p.add_argument("altura", type=float, help="altura da placa em Y (mm)")
    p.add_argument("--tipo", choices=["aneis", "espiral"], default="aneis")
    p.add_argument("--prof", type=float, default=0.05, help="profundidade do corte (mm, padrão 0.05)")
    p.add_argument("--espaco", type=float, default=3.0, help="distância entre sulcos (mm, padrão 3)")
    p.add_argument("--margem", type=float, default=2.0, help="margem até a borda (mm, padrão 2)")
    p.add_argument("--zseg", type=float, default=1.0, help="altura de deslocamento (mm, padrão 1)")
    p.add_argument("--feed", type=float, default=150, help="avanço XY (mm/min, padrão 150)")
    p.add_argument("--feed-z", type=float, default=50, help="avanço de mergulho (mm/min, padrão 50)")
    p.add_argument("--rpm", type=float, default=10000, help="rotação do spindle (0 = não liga)")
    p.add_argument("-o", "--saida", help="arquivo de saída (padrão: teste_<tipo>.nc)")
    args = p.parse_args()

    if args.espaco <= 0 or args.margem < 0 or 2 * args.margem >= min(args.largura, args.altura):
        p.error("confira margem e espaço")
    codigo, resumo = gerar(args)
    saida = args.saida or f"teste_{args.tipo}.nc"
    with open(saida, "w") as f:
        f.write(codigo)
    print(f"Salvo em {saida}: {resumo}, profundidade {abs(args.prof)} mm.")


if __name__ == "__main__":
    main()
