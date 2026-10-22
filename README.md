# TopoCobre

Mede a altura da placa numa malha de pontos (CNC com GRBL 1.1) e corrige o Z do G-code de isolação para a fresa acompanhar o desnível.

## Instalação

```
pip install -r requirements.txt
python topocobre.py
```

Sem o pyserial a interface abre normalmente, só não conecta na máquina.

## Arquivos

- `topocobre.py`: a interface.
- `nivelar_gcode.py`: a correção do G-code. Também roda sozinho: `python nivelar_gcode.py isolacao.nc mapa.csv`.

## Uso (medição manual)

1. Preencha o tamanho da placa, os pontos em X e Y e a margem, e clique em **Gerar malha**. Os pontos aparecem numerados na ordem em que serão visitados.
2. **Desconecte o gSender** antes de abrir o app. Ao abrir, ele conecta sozinho no GRBL em `/dev/ttyACM0` e no ESP32 em `/dev/ttyACM1` (as portas padrão ficam no começo do `topocobre.py`, em `PORTA_GRBL` e `PORTA_PROBE`). Se a porta estiver aberta em outro programa, ele não conecta, porque abrir a porta reinicia o GRBL e estragaria um trabalho em andamento.
3. Com o jog, leve a fresa ao canto inferior esquerdo da placa e clique em **Zerar XY**.
4. Desça o Z até encostar no cobre e clique em **Zerar Z aqui**.
5. **Registrar Z (Enter)** grava o ponto atual. Com "Avançar depois de registrar" ligado, a fresa sobe e vai sozinha para o próximo ponto. Se já existe uma medida, ela desce rápido até `último Z + folga`.
6. Em cada ponto, desça com **↓** (o passo é o escolhido em "Passo Z") até encostar, e aperte **Enter**.
7. No fim, use **Aplicar no G-code**: escolha o `.nc` gerado pelo FlatCAM, e o app grava o mapa (`<nome>_mapa.csv`) e o `<nome>_nivelado.nc` na mesma pasta.
8. Feche o app, reconecte o gSender e rode o `_nivelado.nc`. O zero G54 fica salvo no GRBL, então não precisa zerar de novo.

## Teclado

Funciona com o cursor fora dos campos de texto.

| Tecla | Ação |
|---|---|
| ↑ / ↓ | Z+ / Z− |
| W A S D | Y+ / X− / Y− / X+ |
| 1 2 3 4 | passo Z 1 / 0.1 / 0.05 / 0.01 mm |
| Q / E | passo XY menor / maior |
| Enter | registrar Z |
| N | próximo ponto |
| Esc | PARAR (funciona sempre) |

Clicar num ponto do desenho leva a fresa até ele.

## Probe ESP32 (indicador)

Na linha "Probe ESP32", escolha a porta do ESP32 (diferente da do GRBL) e conecte. O indicador fica verde (LIVRE) ou vermelho (ENCOSTOU) conforme as linhas `L`/`T` que o ESP32 manda, e a cruz da ferramenta no desenho também fica vermelha no toque.

## Nivelamento automático (ESP32)

Com o GRBL e o ESP32 conectados e a malha gerada, deixe a fresa de 1 a 2 mm acima da placa, no 1º ponto ou perto dele, e clique em **▶ Nivelar automático**. Em cada ponto:

1. Sobe "Subir entre pts" (1 mm) acima da última medida e anda até o ponto, para não arrastar na placa.
2. Desce rápido até `última medida + folga` e depois devagar ("Descida", mm/min). No primeiro `T` o jog é cancelado na hora (0x85 do GRBL).
3. Se logo depois do toque o sinal volta a ficar `L` firme, foi ruído: ele ignora e continua descendo (até 3 vezes por ponto).
4. Sobe em passos de "Passo subida" (0.005 mm) até o contato soltar e ficar `L` estável por 150 ms. Esse Z de soltura é o valor gravado, o que elimina o efeito do contato piscando no limiar.

Proteções:
- No 1º ponto ele desce no máximo "Máx. 1º ponto" (3 mm). Nos outros, no máximo "Limite abaixo" (0.3 mm) abaixo da medida anterior. Se não encostar até lá (garra solta, cobre sujo), ele para, sobe 1 mm e avisa.
- Se o ESP32 desconectar ou mandar `E` durante a descida, o jog é cancelado na hora.
- Se subir 1 mm e o contato não soltar, para e avisa (curto na fiação ou fresa presa).
- Esc ou PARAR interrompem a qualquer momento. Os pontos já medidos são mantidos, e o botão continua dos que faltam.

Não precisa zerar o Z com precisão antes: o mapa é medido no mesmo referencial do corte.

Abaixo dos campos aparece o **tempo estimado**, calculado a partir dos parâmetros e das velocidades máximas lidas do GRBL (`$110`–`$112`). Durante a medição ele mostra o tempo decorrido e quanto falta, já usando a média real dos pontos medidos.

## Testar o nivelamento

`gerar_teste.py` gera um G-code de teste que cobre a placa inteira:

```
python3 gerar_teste.py 50 50 --prof 0.05              # anéis concêntricos (teste elétrico)
python3 gerar_teste.py 50 50 --tipo espiral           # espiral (teste visual)
```

Meça a malha no TopoCobre, use **Aplicar no G-code** com o arquivo de teste e rode o `_nivelado.nc`.

- **Anéis:** com o multímetro em continuidade, encoste nos anéis de cobre vizinhos. Sem apito, aquele sulco atravessou o cobre na volta inteira. Um apito indica ponte de cobre em algum ponto daquele sulco.
- **Espiral:** com V-bit, a largura do sulco acompanha a profundidade (`w = w_ponta + 2·d·tan(α/2)`). Com uma lupa ou microscópio USB, a largura tem que ser a mesma nos cantos e no centro.

## Cuidados

- Não mexa na placa nem no zero entre medir e cortar. O mapa vale para aquela fixação.
- "Zerar Z aqui" com pontos já medidos ajusta as medidas para o novo zero. "Zerar XY" apaga as medidas.
- Spindle desligado durante a medição.
- **PARAR** faz um feed hold seguido de reset. A posição é mantida.
