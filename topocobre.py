"""
TopoCobre — mede uma malha de alturas na CNC (GRBL 1.1) e corrige o Z do G-code.

Fluxo:
  1. Gerar a malha (tamanho da placa + número de pontos).
  2. Conectar no GRBL, levar a fresa ao canto da placa com o jog e zerar XY e Z.
  3. Para cada ponto: "Próximo ponto", descer o Z com ↓ até encostar, "Registrar Z".
  4. Salvar o mapa (CSV) e "Aplicar no G-code" -> gera <nome>_nivelado.nc.

Rodar:  python topocobre.py
Dependências: pyserial (conexão), numpy + scipy (aplicar no G-code).
Sem pyserial a interface abre normalmente, só não conecta na máquina.
"""

import csv
import os
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

try:
    import serial
    import serial.tools.list_ports
except ImportError:          # deixa a interface abrir mesmo sem pyserial
    serial = None

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nivelar_gcode as ng   # noqa: E402


# Portas padrão: o app tenta conectar nelas sozinho ao abrir.
PORTA_GRBL = "/dev/ttyACM0"
PORTA_PROBE = "/dev/ttyACM1"


def porta_em_uso(porta):
    """
    No Linux, diz se outro programa (ex.: gSender) está com a porta aberta.
    Importante: abrir a porta do Arduino reinicia o GRBL, o que estragaria
    um trabalho que o gSender estivesse executando.
    """
    if not sys.platform.startswith("linux") or not os.path.exists("/proc"):
        return False
    alvo = os.path.realpath(porta)
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            fds = os.listdir(f"/proc/{pid}/fd")
        except OSError:
            continue          # processo de outro usuário ou que já terminou
        for fd in fds:
            try:
                if os.readlink(f"/proc/{pid}/fd/{fd}") == alvo:
                    return True
            except OSError:
                pass
    return False


def abrir_serial(porta, baud):
    if serial is None:
        raise RuntimeError("O pyserial não está instalado.\nRode: pip install pyserial")
    if porta_em_uso(porta):
        raise RuntimeError(f"A porta {porta} está aberta em outro programa (gSender?).\n"
                           "Desconecte lá antes de conectar aqui.")
    try:
        # exclusive: impede que outro programa abra a mesma porta depois
        return serial.Serial(porta, baud, timeout=0.05, exclusive=True)
    except ValueError:        # pyserial antigo ou sistema sem suporte
        return serial.Serial(porta, baud, timeout=0.05)


# ===========================================================================
# GRBL
# ===========================================================================
ALARMES = {
    1: "fim de curso acionado",
    2: "o movimento sairia dos limites de software da máquina",
    3: "reset durante o movimento: a posição pode ter sido perdida",
    4: "o probe já estava acionado antes de começar",
    5: "o probe não encostou dentro da profundidade máxima",
    6: "homing interrompido",
    8: "homing falhou: não saiu do fim de curso",
    9: "homing falhou: não achou o fim de curso",
}

ERROS = {
    1: "comando sem letra (G-code mal formado)",
    2: "número inválido no comando",
    3: "comando $ do GRBL não reconhecido",
    8: "comando não aceito enquanto a máquina está se movendo",
    9: "máquina em alarme ou travada: use Desbloquear",
    15: "o jog sairia dos limites da máquina",
    16: "comando de jog inválido",
    20: "comando G-code não suportado",
    22: "falta definir o avanço (F)",
}


class GrblError(Exception):
    pass


def traduzir(linha):
    """'error:9' / 'ALARM:5' -> mensagem em português."""
    tipo, _, num = linha.partition(":")
    try:
        n = int(num)
    except ValueError:
        return linha
    tabela = ALARMES if tipo == "ALARM" else ERROS
    desc = tabela.get(n, "código desconhecido")
    return f"{linha}: {desc}"


def parse_status(linha):
    """
    Interpreta um relatório de status do GRBL, ex.:
      <Idle|MPos:1.000,2.000,-3.000|FS:0,0|WCO:0.000,0.000,-2.500>
    Retorna dict com 'estado' e, quando presentes, 'mpos', 'wpos', 'wco'.
    """
    partes = linha.strip().strip("<>").split("|")
    res = {"estado": partes[0]}
    for p in partes[1:]:
        chave, _, valor = p.partition(":")
        if chave in ("MPos", "WPos", "WCO"):
            res[chave.lower()] = tuple(float(n) for n in valor.split(",")[:3])
    return res


class Grbl:
    """
    Conexão serial com o GRBL.

    Envio de comandos é do tipo "manda uma linha, espera o ok": simples e
    suficiente aqui, porque nunca mandamos G-code em sequência rápida.
    """

    def __init__(self, ao_logar=None):
        self.ser = None
        self._lock_escrita = threading.Lock()   # protege ser.write
        self._lock_comando = threading.Lock()   # um comando "linha + ok" por vez
        self._respostas = queue.Queue()
        self._lendo = False
        self.ao_logar = ao_logar or (lambda s: None)

        self.banner = None
        self.estado = "Desconectado"
        self.wpos = None          # posição de trabalho (x, y, z)
        self.wco = None           # offset de trabalho (MPos - WPos)
        self.versao_status = 0    # aumenta a cada status recebido
        self.config = {}          # $N=valor do GRBL (ex.: 110-112 = velocidade máx. X/Y/Z)

    @property
    def conectado(self):
        return self.ser is not None and self._lendo

    # --- conexão ------------------------------------------------------------
    def conectar(self, porta, baud=115200):
        self.banner = None
        try:
            self.ser = abrir_serial(porta, baud)
        except Exception as e:
            raise GrblError(str(e))
        self._lendo = True
        threading.Thread(target=self._ler, daemon=True).start()

        # O Arduino reinicia quando a porta é aberta; o GRBL se anuncia com
        # "Grbl 1.1x". Se não vier, um soft reset força o anúncio.
        if not self._esperar_banner(4):
            self.escrever_rt(b"\x18")
            if not self._esperar_banner(3):
                self.desconectar()
                raise GrblError("Não veio resposta do GRBL nessa porta.\n"
                                "Confira a porta e se o gSender está desconectado.")
        self.escrever_rt(b"?")

    def _esperar_banner(self, segundos):
        t0 = time.time()
        while time.time() - t0 < segundos:
            if self.banner:
                return True
            time.sleep(0.05)
        return False

    def desconectar(self):
        self._lendo = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.estado = "Desconectado"
        self.wpos = None
        self.wco = None

    # --- leitura ------------------------------------------------------------
    def _ler(self):
        buf = b""
        while self._lendo:
            try:
                dados = self.ser.read(256)
            except Exception:
                self._lendo = False
                self.estado = "Desconectado"
                self._respostas.put(("erro", "A conexão com o GRBL caiu."))
                break
            if not dados:
                continue
            buf += dados
            while b"\n" in buf:
                linha, buf = buf.split(b"\n", 1)
                linha = linha.decode("ascii", errors="replace").strip()
                if linha:
                    self._processar(linha)

    def _processar(self, linha):
        if linha.startswith("<"):
            self._atualizar_status(linha)
        elif linha == "ok":
            self._respostas.put(("ok", None))
        elif linha.startswith("error:"):
            self._respostas.put(("erro", traduzir(linha)))
        elif linha.startswith("ALARM:"):
            msg = traduzir(linha)
            self.ao_logar(msg)
            self._respostas.put(("erro", msg))
        elif linha.startswith("Grbl"):
            self.banner = linha
            self.ao_logar(linha)
        elif linha.startswith("$") and "=" in linha:
            chave, _, valor = linha[1:].partition("=")
            try:
                self.config[int(chave)] = float(valor.split()[0])
            except ValueError:
                pass
        else:
            self.ao_logar(linha)

    def _atualizar_status(self, linha):
        try:
            s = parse_status(linha)
        except ValueError:
            return
        self.estado = s["estado"]
        # O WCO só vem de vez em quando (e quando muda), então guardamos o último.
        if "wco" in s:
            self.wco = s["wco"]
        if "wpos" in s:
            self.wpos = s["wpos"]
        elif "mpos" in s and self.wco is not None:
            self.wpos = tuple(m - w for m, w in zip(s["mpos"], self.wco))
        self.versao_status += 1

    # --- escrita ------------------------------------------------------------
    def escrever_rt(self, byte):
        """Comandos de tempo real (?, !, 0x85, 0x18): vão direto, sem newline e sem ok."""
        if not self.ser:
            return
        with self._lock_escrita:
            self.ser.write(byte)

    def enviar(self, linha, timeout=30):
        """Manda uma linha e espera o ok. error/ALARM viram GrblError."""
        if not self.conectado:
            raise GrblError("Não está conectado ao GRBL.")
        with self._lock_comando:
            while not self._respostas.empty():   # descarta respostas velhas
                self._respostas.get_nowait()
            with self._lock_escrita:
                self.ser.write((linha + "\n").encode("ascii"))
            try:
                tipo, msg = self._respostas.get(timeout=timeout)
            except queue.Empty:
                raise GrblError(f"Sem resposta do GRBL para: {linha}")
            if tipo != "ok":
                raise GrblError(f"{msg}\n\nComando: {linha}")

    def esperar_parado(self, timeout=120):
        # O ok de um G0 chega assim que ele entra no buffer, não quando termina.
        # Um G4 P0 (pausa de zero segundos) só recebe ok quando o buffer esvazia,
        # ou seja, quando a máquina realmente parou.
        self.enviar("G4 P0", timeout)

    def esperar_idle(self, timeout=60):
        """
        Espera o estado Idle olhando o status. Diferente do G4 P0, funciona
        logo depois de um jog ($J=), que não aceita G-code comum no meio.
        """
        t0 = time.time()
        seguidos = 0
        while time.time() - t0 < timeout:
            v = self.versao_status
            self.escrever_rt(b"?")
            t1 = time.time()
            while self.versao_status == v and time.time() - t1 < 0.5:
                time.sleep(0.01)
            if self.estado.startswith("Alarm"):
                raise GrblError("A máquina entrou em alarme.")
            seguidos = seguidos + 1 if self.estado.startswith("Idle") else 0
            if seguidos >= 2:
                return
            time.sleep(0.03)
        raise GrblError("A máquina não parou a tempo.")

    def z_atual(self):
        """Pede um status novo e devolve o Z de trabalho."""
        v = self.versao_status
        self.escrever_rt(b"?")
        t0 = time.time()
        while self.versao_status == v and time.time() - t0 < 1.0:
            time.sleep(0.02)
        if self.wpos is None:
            raise GrblError("Ainda não recebi a posição de trabalho do GRBL.\n"
                            "Espere um segundo e tente de novo.")
        return self.wpos[2]

    def cancelar(self):
        """Destrava quem estiver esperando um ok."""
        self._respostas.put(("erro", "Operação interrompida."))


# ===========================================================================
# Probe externo (ESP32)
# ===========================================================================
class ProbeExterno:
    """
    Lê o ESP32 que detecta o toque fresa↔cobre por continuidade.
    O sketch manda uma linha "T" (tocou) ou "L" (livre) só quando o estado
    muda; "E" fica reservado para erro de fiação. Outras linhas (mensagens de
    boot etc.) são ignoradas.
    """

    ESTADOS = ("T", "L", "E")

    def __init__(self, ao_mudar=None):
        self.ser = None
        self._lendo = False
        self.estado = None            # None = desconectado
        self.ao_mudar = ao_mudar or (lambda e: None)
        self.tocou = threading.Event()
        self.ultima_mudanca = time.monotonic()
        self.linhas_recebidas = 0
        self.ultima_linha = ""
        # Chamado na própria thread de leitura, sem passar pela fila da
        # interface: é por aqui que o jog é cancelado no instante do toque.
        self.ao_mudar_rapido = lambda e: None

    @property
    def conectado(self):
        return self.ser is not None and self._lendo

    def conectar(self, porta, baud=115200):
        self.ser = abrir_serial(porta, baud)
        self._lendo = True
        # Abrir a porta normalmente reinicia o ESP32, e o sketch só avisa
        # mudanças: se ele ligar já encostado manda "T"; senão fica quieto.
        # Por isso começamos assumindo "livre".
        self._definir("L")
        threading.Thread(target=self._ler, daemon=True).start()

    def desconectar(self):
        self._lendo = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self._definir(None)

    def _definir(self, estado):
        if estado == self.estado:
            return
        self.estado = estado
        self.ultima_mudanca = time.monotonic()
        if estado == "T":
            self.tocou.set()
        self.ao_mudar_rapido(estado)
        self.ao_mudar(estado)

    def estavel(self, estado, janela):
        """True se está em `estado` sem nenhuma troca há pelo menos `janela` s."""
        return self.estado == estado and time.monotonic() - self.ultima_mudanca >= janela

    def _ler(self):
        buf = b""
        while self._lendo:
            try:
                dados = self.ser.read(64)
            except Exception:
                self._lendo = False
                self.ser = None
                self._definir(None)
                break
            if not dados:
                continue
            buf += dados
            while b"\n" in buf:
                linha, buf = buf.split(b"\n", 1)
                linha = linha.decode("ascii", errors="replace").strip()
                self.linhas_recebidas += 1
                self.ultima_linha = linha
                if linha in self.ESTADOS:
                    self._definir(linha)


# ===========================================================================
# Malha de pontos
# ===========================================================================
class Malha:
    def __init__(self):
        self.pontos = []        # dicts {"x", "y", "z"}; z = None até medir
        self.atual = 0
        self.largura = 0.0
        self.altura = 0.0
        self.nx = self.ny = 0
        self.ultimo_z = None    # último Z registrado (para a descida rápida)

    @staticmethod
    def _espaco(inicio, fim, n):
        return [inicio + i * (fim - inicio) / (n - 1) for i in range(n)]

    def gerar(self, largura, altura, nx, ny, margem):
        if nx < 2 or ny < 2:
            raise ValueError("A malha precisa de pelo menos 2 pontos em X e 2 em Y.")
        if largura <= 0 or altura <= 0:
            raise ValueError("O tamanho da placa precisa ser positivo.")
        if margem < 0 or 2 * margem >= min(largura, altura):
            raise ValueError("A margem é grande demais para esse tamanho de placa.")

        xs = self._espaco(margem, largura - margem, nx)
        ys = self._espaco(margem, altura - margem, ny)
        self._montar(xs, ys)
        self.largura, self.altura = largura, altura

    def _montar(self, xs, ys):
        # Zigue-zague: linhas pares da esquerda para a direita, ímpares de volta.
        # Economiza deslocamento entre os pontos.
        self.pontos = []
        for j, y in enumerate(ys):
            linha = xs if j % 2 == 0 else list(reversed(xs))
            for x in linha:
                self.pontos.append({"x": x, "y": y, "z": None})
        self.nx, self.ny = len(xs), len(ys)
        self.atual = 0
        self.ultimo_z = None

    @property
    def vazia(self):
        return not self.pontos

    def medidos(self):
        return [p for p in self.pontos if p["z"] is not None]

    def completa(self):
        return bool(self.pontos) and all(p["z"] is not None for p in self.pontos)

    def registrar(self, i, z):
        self.pontos[i]["z"] = z
        self.ultimo_z = z

    def deslocar_z(self, dz):
        """Usado ao rezerar o Z: mantém as medidas no novo referencial."""
        for p in self.medidos():
            p["z"] += dz
        if self.ultimo_z is not None:
            self.ultimo_z += dz

    def limpar_medidas(self):
        for p in self.pontos:
            p["z"] = None
        self.ultimo_z = None

    def proximo_nao_medido(self, depois_de):
        n = len(self.pontos)
        for k in range(1, n + 1):
            i = (depois_de + k) % n
            if self.pontos[i]["z"] is None:
                return i
        return None

    def salvar_csv(self, caminho):
        if not self.completa():
            raise ValueError("Ainda há pontos sem medida.")
        with open(caminho, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["x", "y", "z"])
            for p in sorted(self.pontos, key=lambda p: (p["y"], p["x"])):
                w.writerow([f"{p['x']:.4f}", f"{p['y']:.4f}", f"{p['z']:.4f}"])

    def carregar_csv(self, caminho):
        with open(caminho, newline="") as f:
            linhas = [r for r in csv.DictReader(f)]
        if not linhas:
            raise ValueError("O CSV está vazio.")
        dados = {(round(float(r["x"]), 4), round(float(r["y"]), 4)): float(r["z"])
                 for r in linhas}
        xs = sorted({k[0] for k in dados})
        ys = sorted({k[1] for k in dados})
        if len(dados) != len(xs) * len(ys):
            raise ValueError("O CSV não forma uma grade completa.")
        self._montar(xs, ys)
        for p in self.pontos:
            p["z"] = dados[(round(p["x"], 4), round(p["y"], 4))]
        # O tamanho da placa não fica no CSV: assume a margem igual dos dois lados.
        self.largura = xs[-1] + xs[0]
        self.altura = ys[-1] + ys[0]


# ===========================================================================
# Interface
# ===========================================================================
def cor_altura(t):
    """t em [0, 1] -> cor hex: azul (baixo) -> verde -> vermelho (alto)."""
    paradas = [(0.0, (45, 95, 220)), (0.5, (40, 170, 80)), (1.0, (215, 60, 45))]
    t = max(0.0, min(1.0, t))
    for (t0, c0), (t1, c1) in zip(paradas, paradas[1:]):
        if t <= t1:
            f = (t - t0) / (t1 - t0)
            r, g, b = (round(a + (b_ - a) * f) for a, b_ in zip(c0, c1))
            return f"#{r:02x}{g:02x}{b:02x}"
    return "#d73c2d"


class App:
    def __init__(self, root):
        self.root = root
        root.title("TopoCobre — versão 21/10 20:25")
        root.geometry("1150x840")
        root.minsize(980, 680)

        self.grbl = Grbl(ao_logar=lambda s: self.na_ui(lambda: self.mensagem(s)))
        self.probe = ProbeExterno(
            ao_mudar=lambda e: self.na_ui(lambda: self._mostrar_probe(e)))
        self.malha = Malha()
        self.csv_salvo = None
        self._descendo = threading.Event()   # um jog de sondagem está em andamento
        self._abortar = threading.Event()
        self.auto_rodando = False
        self.probe.ao_mudar_rapido = self._probe_mudou_rapido

        # Uma thread executa as tarefas da máquina, uma de cada vez.
        # A interface nunca é tocada por ela: tudo passa pela fila de UI.
        self.tarefas = queue.Queue()
        self.ui = queue.Queue()
        self.ocupado = False
        threading.Thread(target=self._worker, daemon=True).start()

        self._criar_variaveis()
        self._montar_interface()
        self._atualizar_portas()
        self._habilitar_maquina(False)

        self._ligar_teclado()

        self._drenar_ui()
        self._pedir_status()
        root.after(500, self._conectar_automatico)

    # --- infraestrutura -------------------------------------------------------
    def na_ui(self, func):
        self.ui.put(func)

    def _drenar_ui(self):
        while not self.ui.empty():
            self.ui.get_nowait()()
        self.root.after(50, self._drenar_ui)

    def _worker(self):
        while True:
            func = self.tarefas.get()
            self.ocupado = True
            try:
                func()
            except GrblError as e:
                self.na_ui(lambda m=str(e): messagebox.showerror("GRBL", m))
            except Exception as e:
                self.na_ui(lambda m=repr(e): messagebox.showerror("Erro inesperado", m))
            finally:
                self.ocupado = False

    def executar(self, func, eh_jog=False):
        if not self.grbl.conectado:
            messagebox.showwarning("Desconectado", "Conecte no GRBL primeiro.")
            return
        if self.auto_rodando:
            if not eh_jog:
                self.mensagem("Nivelamento automático em andamento. Use PARAR (Esc) para interromper.")
            return
        # Jog com a máquina ocupada só acumularia na fila: melhor ignorar.
        if eh_jog and (self.ocupado or not self.tarefas.empty()):
            return
        self.tarefas.put(func)

    # Atalhos. Setas ↑/↓ ficam no Z porque é o eixo usado o tempo todo na
    # medição; XY vai no WASD para não mover a mesa sem querer com a fresa
    # encostada.
    ATALHOS = [
        ("↑ / ↓", "Z+ / Z−"),
        ("W A S D", "Y+ X− Y− X+"),
        ("1 2 3 4", "passo Z 1 / 0.1 / 0.05 / 0.01"),
        ("Q / E", "passo XY menor / maior"),
        ("Enter", "registrar Z"),
        ("N", "próximo ponto"),
        ("Esc", "PARAR"),
    ]

    def _ligar_teclado(self):
        r = self.root

        def tecla(seq, acao):
            r.bind_all(seq, lambda e: self._tecla(e, acao))

        def letra(c, acao):
            tecla(f"<KeyPress-{c.lower()}>", acao)
            tecla(f"<KeyPress-{c.upper()}>", acao)

        tecla("<Up>", lambda: self.jog_z(+1))
        tecla("<Down>", lambda: self.jog_z(-1))
        letra("w", lambda: self.jog_xy("Y", +1))
        letra("s", lambda: self.jog_xy("Y", -1))
        letra("a", lambda: self.jog_xy("X", -1))
        letra("d", lambda: self.jog_xy("X", +1))
        for k, passo in zip("1234", ("1", "0.1", "0.05", "0.01")):
            tecla(f"<KeyPress-{k}>", lambda p=passo: self._passo_z(p))
        letra("q", lambda: self._passo_xy(-1))
        letra("e", lambda: self._passo_xy(+1))
        tecla("<Return>", self.registrar)
        tecla("<KP_Enter>", self.registrar)
        letra("n", self.proximo)
        # Esc funciona sempre, até com o cursor num campo de texto.
        r.bind_all("<Escape>", lambda e: self.parar())

    def _passo_z(self, passo):
        self.var_passo_z.set(passo)
        self.mensagem(f"Passo Z: {passo} mm")

    def _passo_xy(self, direcao):
        opcoes = ["0.1", "1", "10"]
        atual = self.var_passo_xy.get()
        i = opcoes.index(atual) if atual in opcoes else 1
        i = max(0, min(len(opcoes) - 1, i + direcao))
        self.var_passo_xy.set(opcoes[i])
        self.mensagem(f"Passo XY: {opcoes[i]} mm")

    def _tecla(self, evento, acao):
        # Não sequestra as setas/Enter enquanto se digita num campo.
        if evento.widget.winfo_class() in ("TEntry", "Entry", "TSpinbox", "TCombobox"):
            return
        acao()
        return "break"

    def mensagem(self, texto):
        self.var_msg.set(texto)

    def _num(self, var, nome):
        try:
            return float(var.get().replace(",", "."))
        except ValueError:
            raise ValueError(f"Valor inválido em '{nome}'.")

    # --- variáveis -------------------------------------------------------------
    def _criar_variaveis(self):
        s = tk.StringVar
        self.var_porta = s(value=PORTA_GRBL)
        self.var_porta_probe = s(value=PORTA_PROBE)
        self.var_placa_x = s(value="50")
        self.var_placa_y = s(value="50")
        self.var_pts_x = s(value="4")
        self.var_pts_y = s(value="4")
        self.var_margem = s(value="1")
        self.var_z_seguro = s(value="2")
        self.var_folga = s(value="0.5")
        self.var_passo_xy = s(value="1")
        self.var_passo_z = s(value="0.1")
        self.var_avancar = tk.BooleanVar(value=True)
        self.var_auto_f = s(value="20")
        self.var_auto_passo = s(value="0.005")
        self.var_auto_subida = s(value="1")
        self.var_auto_margem = s(value="0.3")
        self.var_auto_max1 = s(value="3")
        self.var_auto_tempo = s(value="")
        self.var_estado = s(value="Desconectado")
        self.var_pos = s(value="X —   Y —   Z —")
        self.var_ponto = s(value="Nenhuma malha")
        self.var_msg = s(value="Gere a malha para começar.")

    # --- layout ----------------------------------------------------------------
    def _montar_interface(self):
        esquerda = ttk.Frame(self.root, padding=8)
        esquerda.pack(side="left", fill="y")
        direita = ttk.Frame(self.root, padding=(0, 8, 8, 8))
        direita.pack(side="left", fill="both", expand=True)

        self._botoes_maquina = []
        self._secao_conexao(esquerda)
        self._secao_malha(esquerda)
        self._secao_jog(esquerda)
        self._secao_medicao(esquerda)
        self._secao_arquivos(direita)

        self.canvas = tk.Canvas(direita, bg="#f4f4f2", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda e: self.desenhar())
        self.canvas.bind("<Button-1>", self._clique_canvas)

        ttk.Label(direita, textvariable=self.var_msg, anchor="w",
                  foreground="#444").pack(fill="x", pady=(6, 0))

    def _botao(self, pai, texto, cmd, maquina=True, **grid):
        b = ttk.Button(pai, text=texto, command=cmd)
        b.grid(**grid)
        if maquina:
            self._botoes_maquina.append(b)
        return b

    def _campo(self, pai, rotulo, var, linha, col=0, largura=7):
        ttk.Label(pai, text=rotulo).grid(row=linha, column=col, sticky="w", padx=(0, 4), pady=2)
        e = ttk.Entry(pai, textvariable=var, width=largura)
        e.grid(row=linha, column=col + 1, sticky="w", pady=2)
        return e

    def _secao_conexao(self, pai):
        f = ttk.LabelFrame(pai, text="Conexão (GRBL)", padding=6)
        f.pack(fill="x", pady=(0, 6))
        self.combo_porta = ttk.Combobox(f, textvariable=self.var_porta, width=14, state="readonly")
        self.combo_porta.grid(row=0, column=0, padx=(0, 4))
        ttk.Button(f, text="↻", width=3, command=self._atualizar_portas).grid(row=0, column=1)
        self.btn_conectar = ttk.Button(f, text="Conectar", command=self.alternar_conexao)
        self.btn_conectar.grid(row=0, column=2, padx=4)

        self._botao(f, "Desbloquear", self.desbloquear, row=1, column=0, sticky="we", pady=(6, 0))
        parar = tk.Button(f, text="PARAR", command=self.parar, bg="#c62828", fg="white",
                          activebackground="#8e0000", font=("TkDefaultFont", 10, "bold"))
        parar.grid(row=1, column=1, columnspan=2, sticky="we", padx=(4, 0), pady=(6, 0))

        ttk.Label(f, textvariable=self.var_estado, font=("TkDefaultFont", 10, "bold")).grid(
            row=2, column=0, columnspan=3, sticky="w", pady=(6, 0))
        ttk.Label(f, textvariable=self.var_pos, font=("Consolas", 10)).grid(
            row=3, column=0, columnspan=3, sticky="w")

    def _secao_malha(self, pai):
        f = ttk.LabelFrame(pai, text="Malha", padding=6)
        f.pack(fill="x", pady=(0, 6))
        self._campo(f, "Placa X (mm)", self.var_placa_x, 0, 0)
        self._campo(f, "Placa Y (mm)", self.var_placa_y, 0, 2)
        self._campo(f, "Pontos X", self.var_pts_x, 1, 0)
        self._campo(f, "Pontos Y", self.var_pts_y, 1, 2)
        self._campo(f, "Margem (mm)", self.var_margem, 2, 0)
        ttk.Button(f, text="Gerar malha", command=self.gerar_malha).grid(
            row=2, column=2, columnspan=2, sticky="we", padx=(4, 0))
        ttk.Separator(f).grid(row=3, column=0, columnspan=4, sticky="we", pady=6)
        self._campo(f, "Z seguro", self.var_z_seguro, 4, 0)
        self._campo(f, "Folga aprox.", self.var_folga, 4, 2)

    def _secao_jog(self, pai):
        f = ttk.LabelFrame(pai, text="Jog", padding=6)
        f.pack(fill="x", pady=(0, 6))

        xy = ttk.Frame(f)
        xy.grid(row=0, column=0, rowspan=3, padx=(0, 12))
        self._botao(xy, "Y+", lambda: self.jog_xy("Y", +1), row=0, column=1)
        self._botao(xy, "X−", lambda: self.jog_xy("X", -1), row=1, column=0)
        self._botao(xy, "X+", lambda: self.jog_xy("X", +1), row=1, column=2)
        self._botao(xy, "Y−", lambda: self.jog_xy("Y", -1), row=2, column=1)
        for b in xy.winfo_children():
            b.configure(width=4)

        self._botao(f, "Z+ ↑", lambda: self.jog_z(+1), row=0, column=1, sticky="we")
        self._botao(f, "Z− ↓", lambda: self.jog_z(-1), row=2, column=1, sticky="we")

        passos = ttk.Frame(f)
        passos.grid(row=3, column=0, columnspan=2, sticky="w", pady=(6, 0))
        ttk.Label(passos, text="Passo XY:").grid(row=0, column=0, sticky="w")
        for i, v in enumerate(("0.1", "1", "10")):
            ttk.Radiobutton(passos, text=v, value=v, variable=self.var_passo_xy).grid(
                row=0, column=i + 1, padx=2)
        ttk.Label(passos, text="Passo Z:").grid(row=1, column=0, sticky="w")
        for i, v in enumerate(("1", "0.1", "0.05", "0.01")):
            ttk.Radiobutton(passos, text=v, value=v, variable=self.var_passo_z).grid(
                row=1, column=i + 1, padx=2)

        zeros = ttk.Frame(f)
        zeros.grid(row=4, column=0, columnspan=2, sticky="we", pady=(6, 0))
        self._botao(zeros, "Zerar XY", self.zerar_xy, row=0, column=0, padx=(0, 4))
        self._botao(zeros, "Zerar Z aqui", self.zerar_z, row=0, column=1, padx=(0, 4))
        self._botao(zeros, "Ir p/ X0 Y0", self.ir_origem, row=0, column=2)

    def _secao_medicao(self, pai):
        f = ttk.LabelFrame(pai, text="Medição", padding=6)
        f.pack(fill="x", pady=(0, 6))
        ttk.Label(f, textvariable=self.var_ponto).grid(row=0, column=0, columnspan=2, sticky="w")
        self._botao(f, "Próximo ponto (N)", self.proximo, row=1, column=0,
                    sticky="we", padx=(0, 4), pady=4)
        self._botao(f, "Registrar Z (Enter)", self.registrar, row=1, column=1, sticky="we", pady=4)
        ttk.Checkbutton(f, text="Avançar depois de registrar", variable=self.var_avancar).grid(
            row=2, column=0, columnspan=2, sticky="w")
        atalhos = ttk.Frame(f)
        atalhos.grid(row=3, column=0, columnspan=2, sticky="w", pady=(4, 0))
        for i, (tecla, acao) in enumerate(self.ATALHOS):
            ttk.Label(atalhos, text=tecla, font=("TkDefaultFont", 8, "bold")).grid(
                row=i // 2, column=(i % 2) * 2, sticky="w", padx=(0, 4))
            ttk.Label(atalhos, text=acao, foreground="#555", font=("TkDefaultFont", 8)).grid(
                row=i // 2, column=(i % 2) * 2 + 1, sticky="w", padx=(0, 10))

    def _secao_arquivos(self, pai):
        # barra em cima do desenho
        f = ttk.Frame(pai)
        f.pack(fill="x", pady=(0, 6))
        ttk.Label(f, text="Mapa:").pack(side="left", padx=(0, 6))
        ttk.Button(f, text="Salvar CSV", command=self.salvar_csv).pack(side="left", padx=(0, 4))
        ttk.Button(f, text="Carregar CSV", command=self.carregar_csv).pack(side="left", padx=(0, 4))
        ttk.Button(f, text="Aplicar no G-code", command=self.aplicar).pack(side="left")
        self._secao_probe(pai)

    def _secao_probe(self, pai):
        # Linha própria acima do desenho, sempre à vista durante a medição.
        f = ttk.Frame(pai)
        f.pack(fill="x", pady=(0, 6))
        ttk.Label(f, text="Probe ESP32:").pack(side="left", padx=(0, 6))
        self.combo_probe = ttk.Combobox(f, textvariable=self.var_porta_probe,
                                        width=16, state="readonly")
        self.combo_probe.pack(side="left")
        self.btn_probe = ttk.Button(f, text="Conectar", command=self.alternar_probe)
        self.btn_probe.pack(side="left", padx=4)
        # Diagnóstico: mostra o que está chegando cru do ESP32.
        self.var_probe_rx = tk.StringVar(value="")
        ttk.Label(f, textvariable=self.var_probe_rx, foreground="#666",
                  font=("Consolas", 9)).pack(side="left", padx=(6, 0))
        self.lbl_probe = tk.Label(f, width=16, font=("TkDefaultFont", 14, "bold"),
                                  fg="white", relief="ridge", bd=2, padx=8, pady=4)
        self.lbl_probe.pack(side="right")
        self._mostrar_probe(None)
        self._secao_auto(pai)

    def _secao_auto(self, pai):
        f = ttk.LabelFrame(pai, text="Nivelamento automático (ESP32)", padding=(6, 2, 6, 6))
        f.pack(fill="x", pady=(0, 6))
        campos = [
            ("Descida", self.var_auto_f, "mm/min"),
            ("Passo subida", self.var_auto_passo, "mm"),
            ("Subir entre pts", self.var_auto_subida, "mm"),
            ("Limite abaixo", self.var_auto_margem, "mm"),
            ("Máx. 1º ponto", self.var_auto_max1, "mm"),
        ]
        for i, (rot, var, un) in enumerate(campos):
            linha, col = divmod(i, 3)
            ttk.Label(f, text=rot).grid(row=linha, column=col * 3, sticky="e", padx=(10 if col else 0, 3))
            ttk.Entry(f, textvariable=var, width=6).grid(row=linha, column=col * 3 + 1, sticky="w")
            ttk.Label(f, text=un, foreground="#666").grid(row=linha, column=col * 3 + 2, sticky="w")
        b = ttk.Button(f, text="▶ Nivelar automático", command=self.nivelar_automatico)
        b.grid(row=1, column=6, columnspan=3, sticky="we", padx=(10, 0))
        self._botoes_maquina.append(b)
        ttk.Label(f, textvariable=self.var_auto_tempo, foreground="#1f3b73").grid(
            row=2, column=0, columnspan=9, sticky="w", pady=(4, 0))
        # A estimativa acompanha qualquer mudança nos parâmetros.
        for var in (self.var_auto_f, self.var_auto_passo, self.var_auto_subida,
                    self.var_auto_margem, self.var_auto_max1, self.var_folga):
            var.trace_add("write", lambda *a: self._atualizar_estimativa())

    # --- estado da interface -------------------------------------------------------
    def _habilitar_maquina(self, sim):
        for b in self._botoes_maquina:
            b.configure(state="normal" if sim else "disabled")

    def _atualizar_portas(self):
        portas = [p.device for p in serial.tools.list_ports.comports()] if serial else []
        self.combo_porta["values"] = portas
        self.combo_probe["values"] = portas
        if portas and self.var_porta.get() not in portas:
            self.var_porta.set(portas[0])
        if portas and self.var_porta_probe.get() not in portas:
            # sugere uma porta diferente da do GRBL
            outras = [p for p in portas if p != self.var_porta.get()]
            self.var_porta_probe.set((outras or portas)[0])
        # as portas padrão têm preferência sempre que aparecem
        if PORTA_GRBL in portas and not self.grbl.conectado:
            self.var_porta.set(PORTA_GRBL)
        if PORTA_PROBE in portas and not self.probe.conectado:
            self.var_porta_probe.set(PORTA_PROBE)

    # --- probe externo ----------------------------------------------------------
    PROBE_VISUAL = {
        None: ("sem probe", "#9e9e9e"),
        "L": ("LIVRE", "#2e7d32"),
        "T": ("ENCOSTOU", "#c62828"),
        "E": ("ERRO FIAÇÃO", "#ef6c00"),
    }

    def _mostrar_probe(self, estado):
        texto, cor = self.PROBE_VISUAL.get(estado, ("?", "#9e9e9e"))
        self.lbl_probe.configure(text=texto, bg=cor)
        if estado is None and self.btn_probe["text"] == "Desconectar":
            # caiu sozinho (cabo puxado, ESP32 reiniciou de vez...)
            self.btn_probe.configure(text="Conectar")
            self.mensagem("O ESP32 desconectou.")
        if hasattr(self, "canvas"):
            self.desenhar()   # a cruz da ferramenta muda de cor com o toque

    def alternar_probe(self, silencioso=False):
        if self.probe.conectado:
            self.probe.desconectar()
            self.btn_probe.configure(text="Conectar")
            return
        porta = self.var_porta_probe.get()
        if not porta:
            messagebox.showwarning("Porta", "Escolha a porta do ESP32.")
            return
        if self.grbl.conectado and porta == self.var_porta.get():
            messagebox.showwarning("Porta", "Essa porta é a do GRBL. Escolha a do ESP32.")
            return
        try:
            self.probe.conectar(porta)
        except Exception as e:
            if silencioso:
                self.mensagem(f"ESP32 não conectou: {str(e).splitlines()[0]}")
            else:
                messagebox.showerror("ESP32", str(e))
            return
        self.btn_probe.configure(text="Desconectar")
        if self.grbl.conectado:
            self.mensagem(f"GRBL e ESP32 conectados. Encoste a fresa para testar o probe.")
        else:
            self.mensagem(f"ESP32 conectado em {porta}. Encoste a fresa para testar.")

    def _pedir_status(self):
        pr = self.probe
        if pr.conectado:
            ult = pr.ultima_linha[:18] or "nada ainda"
            self.var_probe_rx.set(f"recebido: {pr.linhas_recebidas} linhas | última: {ult}")
        else:
            self.var_probe_rx.set("")
        g = self.grbl
        if g.conectado:
            try:
                g.escrever_rt(b"?")
            except Exception:
                pass
            self.var_estado.set(g.estado)
            if g.wpos and g.wpos != getattr(self, "_wpos_desenhada", None):
                x, y, z = g.wpos
                self.var_pos.set(f"X {x:8.3f}   Y {y:8.3f}   Z {z:8.3f}")
                self._wpos_desenhada = g.wpos
                self.desenhar()     # só redesenha quando a ferramenta se moveu
        elif self.btn_conectar["text"] == "Desconectar":
            # a conexão caiu sozinha
            self._ao_desconectar()
            messagebox.showerror("GRBL", "A conexão com o GRBL caiu.")
        self.root.after(200, self._pedir_status)

    def _atualizar_ponto(self):
        self._atualizar_estimativa()
        m = self.malha
        if m.vazia:
            self.var_ponto.set("Nenhuma malha")
            return
        p = m.pontos[m.atual]
        z = f"{p['z']:+.3f} mm" if p["z"] is not None else "não medido"
        self.var_ponto.set(f"Ponto {m.atual + 1}/{len(m.pontos)}  "
                           f"(X {p['x']:.2f}  Y {p['y']:.2f})  —  {z}   "
                           f"[{len(m.medidos())} medidos]")

    # --- conexão --------------------------------------------------------------------
    def _conectar_automatico(self):
        """Ao abrir: conecta no GRBL e no ESP32 nas portas padrão, se existirem."""
        disponiveis = set(self.combo_porta["values"])
        if PORTA_GRBL in disponiveis:
            self.var_porta.set(PORTA_GRBL)
            self.alternar_conexao(silencioso=True)
        if PORTA_PROBE in disponiveis:
            self.var_porta_probe.set(PORTA_PROBE)
            self.alternar_probe(silencioso=True)
        if not self.grbl.conectado and not self.probe.conectado:
            self.mensagem(f"Não conectei sozinho: {PORTA_GRBL} e {PORTA_PROBE} "
                          "não estão disponíveis ou estão em uso.")

    def alternar_conexao(self, silencioso=False):
        if self.grbl.conectado:
            self.grbl.desconectar()
            self._ao_desconectar()
            return
        porta = self.var_porta.get()
        if not porta:
            messagebox.showwarning("Porta", "Escolha a porta do GRBL.")
            return
        self.mensagem(f"Conectando em {porta}...")
        self.root.update_idletasks()
        try:
            self.grbl.conectar(porta)
        except Exception as e:
            self.mensagem(f"GRBL não conectou: {str(e).splitlines()[0]}")
            if not silencioso:
                messagebox.showerror("Conexão", str(e))
            return
        self.btn_conectar.configure(text="Desconectar")
        self._habilitar_maquina(True)
        self.mensagem(f"Conectado: {self.grbl.banner}")
        # Lê as configurações ($$) para a estimativa de tempo usar as
        # velocidades máximas reais da máquina.
        self.executar(lambda: (self.grbl.enviar("$$"),
                               self.na_ui(self._atualizar_estimativa)))

    def _ao_desconectar(self):
        self.btn_conectar.configure(text="Conectar")
        self._habilitar_maquina(False)
        self.var_estado.set("Desconectado")
        self.var_pos.set("X —   Y —   Z —")
        self.mensagem("Desconectado.")
        self.desenhar()

    def desbloquear(self):
        self.executar(lambda: self.grbl.enviar("$X"))

    def parar(self):
        g = self.grbl
        if not g.conectado:
            return
        while not self.tarefas.empty():
            self.tarefas.get_nowait()
        # Primeiro um feed hold (desacelera e para sem perder a posição),
        # depois o reset limpa o buffer. Resetar com a máquina já parada
        # mantém a posição; resetar em movimento geraria ALARM:3.
        self._abortar.set()
        g.escrever_rt(b"!")
        self.root.after(400, lambda: (g.escrever_rt(b"\x18"), g.cancelar()))
        self.mensagem("Parado.")

    # --- nivelamento automático ---------------------------------------------------
    # Janela de estabilidade do contato: o sinal tem que ficar sem trocar
    # por esse tempo para valer. Absorve o "pisca" entre T e L no limiar do toque.
    JANELA = 0.15
    F_APROX = 100        # mm/min, descida rápida até perto da superfície
    F_SUBIDA = 20        # mm/min, passos de subida na busca do ponto de soltura
    SUBIDA_MAX = 1.0     # mm, se subir isso e não soltar, algo está errado
    FALSOS_MAX = 3       # toques espúrios tolerados por ponto

    @staticmethod
    def formatar_tempo(seg):
        seg = int(round(seg))
        if seg < 60:
            return f"{seg} s"
        m, s = divmod(seg, 60)
        if m < 60:
            return f"{m} min {s:02d} s"
        h, m = divmod(m, 60)
        return f"{h} h {m:02d} min"

    def _estimar_segundos(self, pendentes, ultimo_z_conhecido):
        """
        Estimativa antes de começar, a partir dos parâmetros. Durante a
        medição ela é substituída pela média real dos pontos já feitos.
        """
        try:
            f_desc = self._num(self.var_auto_f, "")
            passo = self._num(self.var_auto_passo, "")
            subida = self._num(self.var_auto_subida, "")
            folga = self._num(self.var_folga, "")
            if f_desc <= 0 or passo <= 0:
                return None
        except ValueError:
            return None
        cfg = self.grbl.config
        v_xy = min(cfg.get(110, 1000.0), cfg.get(111, 1000.0)) / 60   # mm/s (G0)
        v_z = cfg.get(112, 500.0) / 60
        m = self.malha
        pos = self.grbl.wpos[:2] if self.grbl.wpos else None
        total = 0.0
        tem_ref = ultimo_z_conhecido
        for i in pendentes:
            p = m.pontos[i]
            dist = 0.0 if pos is None else ((p["x"] - pos[0]) ** 2 + (p["y"] - pos[1]) ** 2) ** 0.5
            pos = (p["x"], p["y"])
            t = dist / v_xy + subida / v_z + 0.3            # deslocamento + aceleração
            if tem_ref:
                t += max(subida - folga, 0) / (self.F_APROX / 60)   # aproximação rápida
                t += folga / (f_desc / 60)                           # descida lenta
            else:
                t += subida / (self.F_APROX / 60)           # volta à altura inicial
                t += 1.5 / (f_desc / 60)                    # supõe ~1.5 mm até a placa
                tem_ref = True
            t += 1.5 * self.JANELA                          # confirmação do toque
            subir = max(f_desc / 60 * 0.03, passo) + 0.005  # avanço após o toque + faixa de pisca
            t += (subir / passo + 1) * (self.JANELA + 0.12)  # passos de subida
            t += 0.6                                        # idas e voltas de comandos
            total += t
        return total

    def _atualizar_estimativa(self):
        if self.auto_rodando or not hasattr(self, "var_auto_tempo"):
            return
        m = self.malha
        pend = [i for i, p in enumerate(m.pontos) if p["z"] is None]
        if not pend:
            self.var_auto_tempo.set("Todos os pontos medidos." if not m.vazia else "")
            return
        seg = self._estimar_segundos(pend, m.ultimo_z is not None)
        if seg is None:
            self.var_auto_tempo.set("")
            return
        fonte = "" if self.grbl.config else " (velocidades padrão; conecte para usar as da máquina)"
        self.var_auto_tempo.set(f"Tempo estimado: ~{self.formatar_tempo(seg)} "
                                f"para {len(pend)} ponto(s){fonte}")

    def _probe_mudou_rapido(self, estado):
        # Thread do ESP32. Se um jog de sondagem está rodando, cancela na hora
        # (0x85 = jog cancel do GRBL) ao tocar, ao dar erro ou se o ESP32 cair.
        # A 20 mm/min, ~20 ms de atraso dão ~7 µm de avanço depois do toque.
        if self._descendo.is_set() and estado in ("T", "E", None):
            try:
                self.grbl.escrever_rt(b"\x85")
            except Exception:
                pass

    def _checar(self):
        if self._abortar.is_set():
            raise GrblError("Nivelamento interrompido.")
        if self.probe.estado is None:
            raise GrblError("O ESP32 desconectou.")
        if self.probe.estado == "E":
            raise GrblError("O ESP32 indicou erro de fiação.")
        if not self.grbl.conectado:
            raise GrblError("A conexão com o GRBL caiu.")

    def _jog_ate_toque(self, z_alvo, feed):
        """Jog em Z até z_alvo, parando no primeiro T. Retorna True se tocou."""
        g, pr = self.grbl, self.probe
        if pr.estado == "T":
            return True
        z0 = g.z_atual()
        if z_alvo >= z0:
            return False
        pr.tocou.clear()
        self._descendo.set()
        try:
            g.enviar(f"$J=G90 G21 Z{z_alvo:.4f} F{feed:g}")
            t0 = time.monotonic()
            prazo = (z0 - z_alvo) / feed * 60 + 5
            while not pr.tocou.is_set():
                self._checar()
                g.escrever_rt(b"?")
                time.sleep(0.03)
                # Logo depois do $J= o status ainda pode dizer Idle: espera um pouco.
                if time.monotonic() - t0 > 0.4 and g.estado.startswith("Idle"):
                    break                   # chegou no alvo sem encostar
                if time.monotonic() - t0 > prazo:
                    g.escrever_rt(b"\x85")
                    raise GrblError("A descida demorou mais que o esperado.")
            if pr.tocou.is_set():
                g.escrever_rt(b"\x85")     # reforço; o principal já foi na thread do ESP32
        finally:
            self._descendo.clear()
        g.esperar_idle()
        return pr.tocou.is_set()

    def _descer_ate_tocar(self, z_aprox, limite, f_lento):
        """Desce (rápido até z_aprox, depois lento até o limite) até um toque confirmado."""
        pr = self.probe
        falsos = 0
        trechos = []
        if z_aprox is not None and z_aprox > limite:
            trechos.append((z_aprox, self.F_APROX))
        trechos.append((limite, f_lento))
        for alvo, feed in trechos:
            while self._jog_ate_toque(alvo, feed):
                # Parou num T. Se em seguida o sinal fica L firme, foi ruído:
                # conta e continua descendo. Se fica T ou piscando, é contato.
                time.sleep(self.JANELA * 1.5)
                if pr.estavel("L", self.JANELA):
                    falsos += 1
                    if falsos > self.FALSOS_MAX:
                        raise GrblError("Muitos toques falsos neste ponto (ruído no sinal do ESP32).")
                    continue
                return falsos
        raise GrblError(f"Não encostou até o limite (Z {limite:.3f}).\n"
                        "Garra solta, cobre sujo, ou o limite está alto demais.")

    def _subir_ate_soltar(self, passo):
        """Sobe em passos até o contato soltar de forma estável. Retorna (z, passos)."""
        g, pr = self.grbl, self.probe
        subido = 0.0
        passos = 0
        while True:
            self._checar()
            time.sleep(self.JANELA)
            if pr.estavel("L", self.JANELA):
                return g.z_atual(), passos
            if subido >= self.SUBIDA_MAX:
                raise GrblError(f"Subiu {subido:.2f} mm e o contato não soltou.\n"
                                "Fios em curto ou fresa presa no cobre?")
            g.enviar(f"$J=G91 G21 Z{passo:.4f} F{self.F_SUBIDA}")
            g.esperar_idle()
            subido += passo
            passos += 1

    def nivelar_automatico(self):
        m = self.malha
        if m.vazia:
            messagebox.showwarning("Automático", "Gere a malha primeiro.")
            return
        if not self.grbl.conectado or not self.probe.conectado:
            messagebox.showwarning("Automático", "Conecte o GRBL e o ESP32 primeiro.")
            return
        if self.probe.estado != "L":
            messagebox.showwarning("Automático", "O probe não está LIVRE. Afaste a fresa da placa.")
            return
        pendentes = [i for i, p in enumerate(m.pontos) if p["z"] is None]
        if not pendentes:
            messagebox.showinfo("Automático", "Todos os pontos já foram medidos.")
            return
        try:
            f_desc = self._num(self.var_auto_f, "Veloc. descida")
            passo = self._num(self.var_auto_passo, "Passo subida")
            subida = self._num(self.var_auto_subida, "Subir entre pontos")
            margem = self._num(self.var_auto_margem, "Limite abaixo do anterior")
            max1 = self._num(self.var_auto_max1, "Descida máx. 1º ponto")
            folga = self._num(self.var_folga, "Folga aprox.")
            zs = self._num(self.var_z_seguro, "Z seguro")
        except ValueError as e:
            messagebox.showerror("Valor", str(e))
            return
        if not (0 < f_desc <= 200 and 0 < passo <= 0.05 and subida > 0 and margem > 0 and max1 > 0):
            messagebox.showerror("Valor", "Confira os parâmetros do automático "
                                          "(velocidade até 200 mm/min, passo até 0.05 mm).")
            return
        primeira = m.ultimo_z is None
        aviso = (f"Vai medir {len(pendentes)} ponto(s) sozinho.\n\n"
                 "• Spindle desligado e garras ligadas.\n")
        if primeira:
            aviso += (f"• Deixe a fresa de 1 a 2 mm acima da placa: no 1º ponto ela desce "
                      f"no máximo {max1:g} mm a partir de onde está.\n")
        aviso += "\nEsc ou PARAR interrompe a qualquer momento. Começar?"
        if not messagebox.askyesno("Nivelamento automático", aviso):
            return

        self._abortar.clear()
        self.auto_rodando = True
        total = len(pendentes)
        estimativa_inicial = self._estimar_segundos(pendentes, m.ultimo_z is not None) or 0
        tempos = []           # duração real de cada ponto

        def mostrar_tempo(feitos, t_inicio):
            gasto = time.monotonic() - t_inicio
            restantes = total - feitos
            if len(tempos) >= 2:
                # o 1º ponto costuma ser mais lento (descida longa): fica de fora da média
                media = sum(tempos[1:]) / len(tempos[1:])
                resta = media * restantes
            else:
                resta = max(estimativa_inicial - gasto, 0) if feitos == 0 else \
                    estimativa_inicial * restantes / total
            self.var_auto_tempo.set(
                f"Medindo {feitos}/{total} — decorrido {self.formatar_tempo(gasto)}, "
                f"falta ~{self.formatar_tempo(resta)}")

        def tarefa():
            g = self.grbl
            resumo = {"falsos": 0}
            t_inicio = time.monotonic()
            self.na_ui(lambda: mostrar_tempo(0, t_inicio))
            try:
                g.enviar("G21 G90")
                g.esperar_parado()
                for n, i in enumerate(pendentes, 1):
                    self._checar()
                    t_ponto = time.monotonic()
                    p = m.pontos[i]
                    self.na_ui(lambda i=i, n=n: (
                        setattr(m, "atual", i), self._atualizar_ponto(), self.desenhar(),
                        self.mensagem(f"Automático: ponto {i + 1} ({n}/{total})...")))

                    # 1) sobe um pouco antes de andar, para não arrastar na placa
                    z_agora = g.z_atual()
                    base = m.ultimo_z if m.ultimo_z is not None else z_agora
                    z_desloc = min(max(base + subida, z_agora), max(zs, z_agora))
                    g.enviar(f"G0 Z{z_desloc:.4f}")
                    g.enviar(f"G0 X{p['x']:.4f} Y{p['y']:.4f}")
                    g.esperar_parado()

                    # 2) desce até encostar (com proteção de profundidade)
                    if m.ultimo_z is None:
                        # 1º ponto: a fresa começou acima da placa, então dá para
                        # voltar rápido até a altura inicial e só daí ir devagar.
                        z_aprox, limite = z_agora, z_agora - max1
                    else:
                        z_aprox, limite = m.ultimo_z + folga, m.ultimo_z - margem
                    resumo["falsos"] += self._descer_ate_tocar(z_aprox, limite, f_desc)

                    # 3) sobe devagar até o contato soltar: esse é o Z medido
                    z, _ = self._subir_ate_soltar(passo)

                    tempos.append(time.monotonic() - t_ponto)

                    def registrar(i=i, z=z, n=n):
                        m.registrar(i, z)
                        self.csv_salvo = None
                        self._atualizar_ponto()
                        self.desenhar()
                        mostrar_tempo(n, t_inicio)
                    self.na_ui(registrar)

                # fim: sobe para o Z seguro
                g.enviar(f"G0 Z{max(zs, g.z_atual()):.4f}")
                g.esperar_parado()
                med = [q["z"] for q in m.pontos if q["z"] is not None]
                gasto = self.formatar_tempo(time.monotonic() - t_inicio)
                self.na_ui(lambda: self.var_auto_tempo.set(f"Concluído em {gasto}."))
                msg = (f"Nivelamento concluído: {total} ponto(s), variação "
                       f"{max(med) - min(med):.3f} mm, toques falsos ignorados: {resumo['falsos']}.")
                self.na_ui(lambda: self.mensagem(msg))
            except GrblError:
                self.na_ui(lambda: self.mensagem(
                    f"Automático parou no ponto {m.atual + 1}. Os pontos já medidos foram mantidos."))
                # volta a mostrar a estimativa do que falta (roda depois do finally)
                self.na_ui(self._atualizar_estimativa)
                # Garante que nada continua descendo, e sobe um pouco se der.
                self._descendo.clear()
                try:
                    g.escrever_rt(b"\x85")
                    if not self._abortar.is_set() and g.conectado:
                        g.esperar_idle(timeout=5)
                        g.enviar("G91 G0 Z1")
                        g.enviar("G90")
                except Exception:
                    pass
                raise
            finally:
                self.auto_rodando = False
        self.tarefas.put(tarefa)

    # --- malha ------------------------------------------------------------------------
    def gerar_malha(self):
        if self.malha.medidos() and not messagebox.askyesno(
                "Gerar malha", "Já há pontos medidos. Gerar uma malha nova apaga as medidas. Continuar?"):
            return
        try:
            largura = self._num(self.var_placa_x, "Placa X")
            altura = self._num(self.var_placa_y, "Placa Y")
            nx = int(self._num(self.var_pts_x, "Pontos X"))
            ny = int(self._num(self.var_pts_y, "Pontos Y"))
            margem = self._num(self.var_margem, "Margem")
            self.malha.gerar(largura, altura, nx, ny, margem)
        except ValueError as e:
            messagebox.showerror("Malha", str(e))
            return
        self.csv_salvo = None
        passo_x = (largura - 2 * margem) / (nx - 1)
        passo_y = (altura - 2 * margem) / (ny - 1)
        self.mensagem(f"Malha de {nx}×{ny} = {nx * ny} pontos, espaçamento "
                      f"{passo_x:.1f} × {passo_y:.1f} mm.")
        self._atualizar_ponto()
        self.desenhar()

    # --- movimento --------------------------------------------------------------------
    def jog_xy(self, eixo, sinal):
        d = sinal * float(self.var_passo_xy.get())
        self.executar(lambda: self.grbl.enviar(f"$J=G91 G21 {eixo}{d:.3f} F1000"), eh_jog=True)

    def jog_z(self, sinal):
        passo = float(self.var_passo_z.get())
        feed = 300 if passo >= 1 else 100
        d = sinal * passo
        self.executar(lambda: self.grbl.enviar(f"$J=G91 G21 Z{d:.3f} F{feed}"), eh_jog=True)

    def ir_origem(self):
        try:
            zs = self._num(self.var_z_seguro, "Z seguro")
        except ValueError as e:
            messagebox.showerror("Valor", str(e))
            return

        def tarefa():
            g = self.grbl
            g.enviar("G21 G90")
            g.enviar(f"G0 Z{zs:.3f}")
            g.enviar("G0 X0 Y0")
            g.esperar_parado()
        self.executar(tarefa)

    def ir_para_ponto(self, i):
        m = self.malha
        if m.vazia:
            return
        try:
            zs = self._num(self.var_z_seguro, "Z seguro")
            folga = self._num(self.var_folga, "Folga aprox.")
        except ValueError as e:
            messagebox.showerror("Valor", str(e))
            return
        m.atual = i
        self._atualizar_ponto()
        self.desenhar()
        p = m.pontos[i]
        # Só desce rápido até perto da placa se já existe uma medida de referência;
        # antes disso o Z0 pode não estar perto da superfície.
        z_aprox = None if m.ultimo_z is None else min(m.ultimo_z + folga, zs)

        def tarefa():
            g = self.grbl
            g.enviar("G21 G90")
            g.enviar(f"G0 Z{zs:.3f}")
            g.enviar(f"G0 X{p['x']:.3f} Y{p['y']:.3f}")
            if z_aprox is not None:
                g.enviar(f"G0 Z{z_aprox:.3f}")
            g.esperar_parado()
            self.na_ui(lambda: self.mensagem(
                f"No ponto {i + 1}. Desça o Z com ↓ até encostar e registre."))
        self.executar(tarefa)

    def proximo(self):
        m = self.malha
        if m.vazia:
            return
        i = m.proximo_nao_medido(m.atual)
        if i is None:
            self.mensagem("Todos os pontos foram medidos. Salve o CSV ou aplique no G-code.")
            return
        self.ir_para_ponto(i)

    def registrar(self):
        m = self.malha
        if m.vazia:
            return
        i = m.atual

        def tarefa():
            self.grbl.esperar_parado()
            time.sleep(0.3)               # garante um status com a posição final
            z = self.grbl.z_atual()

            def na_tela():
                m.registrar(i, z)
                self.csv_salvo = None
                self._atualizar_ponto()
                self.desenhar()
                self.mensagem(f"Ponto {i + 1}: Z = {z:+.3f} mm")
                if self.var_avancar.get():
                    self.proximo()
            self.na_ui(na_tela)
        self.executar(tarefa)

    def zerar_xy(self):
        if self.malha.medidos():
            if not messagebox.askyesno(
                    "Zerar XY", "Mudar o zero de XY invalida os pontos já medidos "
                                "(eles seriam apagados). Continuar?"):
                return
            self.malha.limpar_medidas()
            self._atualizar_ponto()
            self.desenhar()
        self.executar(lambda: self.grbl.enviar("G10 L20 P1 X0 Y0"))

    def zerar_z(self):
        m = self.malha
        if m.medidos() and not messagebox.askyesno(
                "Zerar Z", "Já há pontos medidos. Eles serão ajustados para o novo zero "
                           "(continuam valendo). Continuar?"):
            return

        def tarefa():
            g = self.grbl
            g.esperar_parado()
            time.sleep(0.3)
            zc = g.z_atual()
            g.enviar("G10 L20 P1 Z0")
            # O novo zero fica zc acima/abaixo do antigo: corrige as medidas
            # já feitas para continuarem no mesmo referencial do corte.
            self.na_ui(lambda: (m.deslocar_z(-zc), self._atualizar_ponto(), self.desenhar(),
                                self.mensagem("Z zerado nesta posição.")))
        self.executar(tarefa)

    # --- arquivos -------------------------------------------------------------------
    def salvar_csv(self):
        if not self.malha.completa():
            messagebox.showwarning("CSV", "Ainda há pontos sem medida.")
            return
        caminho = filedialog.asksaveasfilename(defaultextension=".csv",
                                               filetypes=[("CSV", "*.csv")],
                                               initialfile="mapa.csv")
        if not caminho:
            return
        self.malha.salvar_csv(caminho)
        self.csv_salvo = caminho
        self.mensagem(f"Mapa salvo em {caminho}")

    def carregar_csv(self):
        caminho = filedialog.askopenfilename(filetypes=[("CSV", "*.csv")])
        if not caminho:
            return
        try:
            self.malha.carregar_csv(caminho)
        except (ValueError, KeyError) as e:
            messagebox.showerror("CSV", f"Não consegui ler o mapa: {e}")
            return
        self.csv_salvo = caminho
        self.var_pts_x.set(str(self.malha.nx))
        self.var_pts_y.set(str(self.malha.ny))
        self._atualizar_ponto()
        self.desenhar()
        self.mensagem(f"Mapa carregado: {self.malha.nx}×{self.malha.ny} pontos.")

    def aplicar(self):
        if not self.malha.completa():
            messagebox.showwarning("G-code", "Meça todos os pontos (ou carregue um CSV) antes.")
            return
        entrada = filedialog.askopenfilename(
            title="G-code a nivelar", filetypes=[("G-code", "*.nc *.gcode *.ngc *.tap"), ("Todos", "*.*")])
        if not entrada:
            return
        base, ext = os.path.splitext(entrada)
        mapa = self.csv_salvo or base + "_mapa.csv"
        if not self.csv_salvo:
            self.malha.salvar_csv(mapa)
            self.csv_salvo = mapa
        saida = base + "_nivelado" + (ext or ".nc")
        try:
            h = ng.carregar_mapa(mapa)
            corr = ng.nivelar(entrada, saida, h, 1.0)
        except ValueError as e:
            messagebox.showerror("G-code", str(e))
            return
        if not corr:
            messagebox.showwarning("G-code", "Nenhum movimento encontrado no arquivo.")
            return
        messagebox.showinfo(
            "G-code nivelado",
            f"Salvo em:\n{saida}\n\nCorreção aplicada: de {min(corr):+.3f} a {max(corr):+.3f} mm")
        self.mensagem(f"G-code nivelado: {saida}")

    # --- desenho ----------------------------------------------------------------------
    def desenhar(self):
        c = self.canvas
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        m = self.malha
        self._pontos_tela = []
        if m.vazia or w < 50:
            c.create_text(w / 2, h / 2, text="Preencha o tamanho da placa e clique em Gerar malha",
                          fill="#888", font=("TkDefaultFont", 12))
            return

        pad = 60
        esc = min((w - 2 * pad) / m.largura, (h - 2 * pad) / m.altura)
        ox = (w - m.largura * esc) / 2
        oy = (h + m.altura * esc) / 2

        def tela(x, y):                 # Y da máquina para cima, Y da tela para baixo
            return ox + x * esc, oy - y * esc

        # placa
        x0, y0 = tela(0, 0)
        x1, y1 = tela(m.largura, m.altura)
        c.create_rectangle(x0, y1, x1, y0, fill="#e8c9a0", outline="#a87c4f", width=2)
        c.create_text(x0, y0 + 14, text="X0 Y0", fill="#555", anchor="w")
        c.create_text(x1, y0 + 14, text=f"{m.largura:g} mm", fill="#555", anchor="e")
        c.create_text(x0 - 8, y1, text=f"{m.altura:g} mm", fill="#555", anchor="e")

        # ordem de visita
        trajeto = [coord for p in m.pontos for coord in tela(p["x"], p["y"])]
        c.create_line(*trajeto, fill="#b9a183", dash=(3, 3))

        medidos = [p["z"] for p in m.medidos()]
        zmin = min(medidos) if medidos else 0.0
        zmax = max(medidos) if medidos else 0.0
        faixa = (zmax - zmin) or 1.0

        for i, p in enumerate(m.pontos):
            cx, cy = tela(p["x"], p["y"])
            self._pontos_tela.append((cx, cy))
            r = 9 if i == m.atual else 7
            if p["z"] is None:
                cor = "#bdbdbd"
            else:
                cor = cor_altura((p["z"] - zmin) / faixa)
            contorno = "#111" if i == m.atual else "#666"
            c.create_oval(cx - r, cy - r, cx + r, cy + r, fill=cor, outline=contorno,
                          width=3 if i == m.atual else 1)
            c.create_text(cx, cy, text=str(i + 1), fill="white" if p["z"] is not None else "#333",
                          font=("TkDefaultFont", 7))
            if p["z"] is not None:
                c.create_text(cx + 12, cy - 12, text=f"{p['z']:+.3f}", anchor="w",
                              fill="#222", font=("Consolas", 9))

        # legenda
        if medidos:
            lx, ly = 16, 16
            for k in range(100):
                c.create_line(lx + k * 1.5, ly, lx + k * 1.5, ly + 10, fill=cor_altura(k / 99), width=2)
            c.create_text(lx, ly + 22, text=f"{zmin:+.3f}", anchor="w", font=("Consolas", 9))
            c.create_text(lx + 150, ly + 22, text=f"{zmax:+.3f} mm", anchor="e", font=("Consolas", 9))
            c.create_text(lx + 160, ly + 5, text=f"variação {zmax - zmin:.3f} mm",
                          anchor="w", fill="#444")

        # posição da ferramenta
        if self.grbl.conectado and self.grbl.wpos:
            tx, ty = tela(self.grbl.wpos[0], self.grbl.wpos[1])
            cor = "#c62828" if self.probe.estado == "T" else "#1f3b73"
            c.create_line(tx - 10, ty, tx + 10, ty, fill=cor, width=2)
            c.create_line(tx, ty - 10, tx, ty + 10, fill=cor, width=2)

    def _clique_canvas(self, evento):
        for i, (cx, cy) in enumerate(getattr(self, "_pontos_tela", [])):
            if (evento.x - cx) ** 2 + (evento.y - cy) ** 2 <= 12 ** 2:
                if self.grbl.conectado:
                    self.ir_para_ponto(i)
                else:
                    self.malha.atual = i
                    self._atualizar_ponto()
                    self.desenhar()
                return


def main():
    root = tk.Tk()
    try:
        ttk.Style().theme_use("vista" if sys.platform == "win32" else "clam")
    except tk.TclError:
        pass
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
