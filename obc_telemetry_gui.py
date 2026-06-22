#!/usr/bin/env python3
"""
obc_fusion_gui.py
OBC Telemetry Monitor — Fusion AHRS
STM32L476 · MPU6050 · FreeRTOS
"""

import sys, os, re, math, time, struct, threading, collections, json, base64
from typing import Optional

import numpy as np
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QPushButton, QLineEdit, QGroupBox,
    QSplitter, QTabWidget, QSizePolicy, QGraphicsOpacityEffect
)
from PyQt6.QtCore import (
    Qt, QTimer, pyqtSignal, QObject, QThread, QPointF,
    QPropertyAnimation, QEasingCurve, QPoint, QRect
)
from PyQt6.QtGui import (
    QFont, QColor, QPalette, QPainter, QPen, QBrush, QPolygonF, QPixmap
)
from PyQt6.QtOpenGLWidgets import QOpenGLWidget
import pyqtgraph as pg
import serial

try:
    import paho.mqtt.client as mqtt
    PAHO_OK = True
except ImportError:
    mqtt = None
    PAHO_OK = False

try:
    from OpenGL.GL import (
        glBegin, glEnd, glVertex3f, glColor3f, glLineWidth,
        glClearColor, glClear, glEnable, glMatrixMode, glLoadIdentity,
        glViewport, glFrustum, glMultMatrixf, glTranslatef,
        GL_LINES, GL_QUADS, GL_DEPTH_TEST,
        GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT,
        GL_MODELVIEW, GL_PROJECTION,
    )
    OPENGL_OK = True
except ImportError:
    OPENGL_OK = False

# ── Tema ─────────────────────────────────────────────────────────────────────
BG      = "#060B14"
BG_CARD = "#0C1424"
BG_MID  = "#080D18"
BORDER  = "#0F2040"
TEXT    = "#C8DCF0"
DIM     = "#2E4A6A"
ACCENT  = "#00AAFF"
GREEN   = "#00FFB3"
AMBER   = "#FFD600"
RED     = "#FF1F4B"
CYAN    = "#00D4FF"
PINK    = "#FF4D6D"

WINDOW = 500

BIN_SOF  = 0xAA
BIN_EOF  = 0x55
BIN_SIZE = 14
BIN_FMT  = "<Bhhhhbb"

ASCII_RE = re.compile(
    r"T=(\d+)\s*\|"
    r"\s*AX=(-?\d+)\s+AY=(-?\d+)\s+AZ=(-?\d+)\s*\|"
    r"\s*GX=(-?\d+)\s+GY=(-?\d+)\s+GZ=(-?\d+)\s*\|"
    r"\s*MX=(-?\d+)\s+MY=(-?\d+)\s+MZ=(-?\d+)\s*\|"
    r"\s*Q=(-?\d+),(-?\d+),(-?\d+),(-?\d+)\s*\|"
    r"\s*MAG=(OK|ERR)"
)

def parse_ascii(line: str) -> Optional[dict]:
    m = ASCII_RE.search(line)
    if m:
        g = m.groups()
        return {
            "t_ms": int(g[0]),
            "ax": int(g[1]), "ay": int(g[2]), "az": int(g[3]),
            "gx": int(g[4]), "gy": int(g[5]), "gz": int(g[6]),
            "mx": int(g[7]), "my": int(g[8]), "mz": int(g[9]),
            "q0": int(g[10]), "q1": int(g[11]),
            "q2": int(g[12]), "q3": int(g[13]),
            "mag_ok": (g[14] == "OK"), "seq": 0,
            "q0f": int(g[10]) / 10000.0,
            "q1f": int(g[11]) / 10000.0,
            "q2f": int(g[12]) / 10000.0,
            "q3f": int(g[13]) / 10000.0,
        }

    return parse_quaternion_payload(line)


LORA_Q_RE = re.compile(r"Q,([+-]\d+),([+-]\d+),([+-]\d+),([+-]\d+)")

def parse_quaternion_payload(text: str) -> Optional[dict]:
    m = LORA_Q_RE.search(text.strip())
    if not m:
        return None
    q0, q1, q2, q3 = (int(v) for v in m.groups())
    return {
        "t_ms": None,
        "ax": 0, "ay": 0, "az": 0,
        "gx": 0, "gy": 0, "gz": 0,
        "mx": 0, "my": 0, "mz": 0,
        "q0": q0, "q1": q1, "q2": q2, "q3": q3,
        "mag_ok": False, "seq": 0,
        "q0f": q0 / 10000.0,
        "q1f": q1 / 10000.0,
        "q2f": q2 / 10000.0,
        "q3f": q3 / 10000.0,
    }

def parse_ttn_uplink(payload: bytes) -> Optional[dict]:
    try:
        msg = json.loads(payload.decode("utf-8", errors="replace"))
        uplink = msg.get("uplink_message", {})
        frm = uplink.get("frm_payload") or msg.get("frm_payload") or msg.get("data")
        if not frm:
            return None
        raw = base64.b64decode(frm)
        text = raw.decode("utf-8", errors="replace")
        rec = parse_quaternion_payload(text)
        if rec is None:
            return None
        rec["rssi"] = uplink.get("rx_metadata", [{}])[0].get("rssi") if uplink.get("rx_metadata") else None
        rec["snr"] = uplink.get("rx_metadata", [{}])[0].get("snr") if uplink.get("rx_metadata") else None
        rec["freq"] = uplink.get("settings", {}).get("frequency")
        rec["source"] = "ttn"
        rec["payload_text"] = text
        return rec
    except Exception:
        return None

def parse_binary(buf: bytes):
    i = 0
    while i <= len(buf) - BIN_SIZE:
        if buf[i] != BIN_SOF:
            i += 1; continue
        if buf[i + BIN_SIZE - 1] != BIN_EOF:
            i += 1; continue
        chunk = buf[i:i + BIN_SIZE]
        chk_calc = sum(chunk[:11]) & 0xFFFF
        chk_recv = struct.unpack_from("<H", chunk, 11)[0]
        if chk_calc != chk_recv:
            i += 1; continue
        _, q0, q1, q2, q3, seq, mag_ok = struct.unpack_from(BIN_FMT, chunk, 0)
        return {
            "q0": q0, "q1": q1, "q2": q2, "q3": q3,
            "seq": seq, "mag_ok": bool(mag_ok),
            "q0f": q0/10000.0, "q1f": q1/10000.0,
            "q2f": q2/10000.0, "q3f": q3/10000.0,
            "t_ms": None,
            "ax": 0, "ay": 0, "az": 0,
            "gx": 0, "gy": 0, "gz": 0,
            "mx": 0, "my": 0, "mz": 0,
        }, i + BIN_SIZE
    return None, len(buf)


# ── TelemetryBuffer ───────────────────────────────────────────────────────────
class TelemetryBuffer:
    KEYS = ["t_s", "q0f", "q1f", "q2f", "q3f"]
    def __init__(self):
        self._lock = threading.Lock()
        self._data = {k: collections.deque(maxlen=WINDOW) for k in self.KEYS}
        self._t0 = None
        self._last = None

    def push(self, record):
        with self._lock:
            if self._t0 is None: self._t0 = time.time()
            t = time.time() - self._t0
            self._data["t_s"].append(t)
            for k in ["q0f","q1f","q2f","q3f"]:
                self._data[k].append(record.get(k, 0.0))
            self._last = record

    def snapshot(self):
        with self._lock:
            return {k: np.array(self._data[k]) for k in self.KEYS}

    def clear(self):
        with self._lock:
            for k in self.KEYS: self._data[k].clear()
            self._t0 = None; self._last = None


# ── SerialWorker ──────────────────────────────────────────────────────────────
class SerialWorker(QObject):
    data_ready = pyqtSignal(dict)
    status_sig = pyqtSignal(bool, str)

    def __init__(self, port, baud, binary):
        super().__init__()
        self.port = port; self.baud = baud; self.binary = binary
        self._run = False
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start_reading(self):
        self._run = True; self._thread.start()

    def stop(self): self._run = False

    def _loop(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1.0)
            self.status_sig.emit(True, "")
        except Exception as e:
            self.status_sig.emit(False, str(e)); return

        buf = b""
        while self._run:
            try:
                if self.binary:
                    chunk = ser.read(BIN_SIZE * 4)
                    if chunk:
                        buf += chunk
                        while True:
                            result = parse_binary(buf)
                            if result[0] is None:
                                buf = buf[result[1]:]; break
                            rec, consumed = result
                            buf = buf[consumed:]
                            self.data_ready.emit(rec)
                else:
                    line = ser.readline()
                    if line:
                        rec = parse_ascii(line.decode("utf-8", errors="replace").strip())
                        if rec: self.data_ready.emit(rec)
            except Exception as e:
                self.status_sig.emit(False, str(e)); break
        try: ser.close()
        except: pass
        self.status_sig.emit(False, "")


# ── TTN MQTT Worker ───────────────────────────────────────────────────────────
class TtnMqttWorker(QObject):
    data_ready = pyqtSignal(dict)
    status_sig = pyqtSignal(bool, str)

    def __init__(self, app_id, device_id, api_key, host="au1.cloud.thethings.network"):
        super().__init__()
        self.app_id = app_id.strip()
        self.device_id = device_id.strip()
        self.api_key = api_key.strip()
        self.host = host.strip()
        self._client = None

    def start_reading(self):
        if not PAHO_OK:
            self.status_sig.emit(False, "Falta paho-mqtt: pip install paho-mqtt")
            return
        if not self.app_id or not self.device_id or not self.api_key:
            self.status_sig.emit(False, "Completa App ID, Device ID y API key")
            return

        username = f"{self.app_id}@ttn"
        topic = f"v3/{self.app_id}@ttn/devices/{self.device_id}/up"
        client_id = f"obc-gui-{int(time.time())}"
        print(f"[TTN] Connecting host={self.host}:8883 username={username} topic={topic}", flush=True)
        self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        self._client.username_pw_set(username, self.api_key)
        self._client.tls_set()
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        self._topic = topic

        try:
            self._client.connect(self.host, 8883, 60)
            self._client.loop_forever()
        except Exception as e:
            print(f"[TTN][ERROR] connect/loop exception: {e}", flush=True)
            self.status_sig.emit(False, str(e))

    def stop(self):
        if self._client is not None:
            try:
                self._client.disconnect()
            except Exception:
                pass

    @staticmethod
    def _reason_code_value(reason_code):
        return getattr(reason_code, "value", reason_code)

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        rc = self._reason_code_value(reason_code)
        if rc == 0:
            print(f"[TTN] Connected. Subscribing to {self._topic}", flush=True)
            client.subscribe(self._topic)
            self.status_sig.emit(True, self._topic)
        else:
            print(f"[TTN][ERROR] MQTT connect failed rc={rc} reason={reason_code} app={self.app_id}", flush=True)
            self.status_sig.emit(False, f"MQTT connect rc={rc} {reason_code}")

    def _on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties=None):
        rc = self._reason_code_value(reason_code)
        print(f"[TTN] Disconnected rc={rc} reason={reason_code}", flush=True)
        self.status_sig.emit(False, "")

    def _on_message(self, client, userdata, msg):
        print(f"[TTN] Message topic={msg.topic} bytes={len(msg.payload)}", flush=True)
        rec = parse_ttn_uplink(msg.payload)
        if rec:
            print(f"[TTN] Parsed payload={rec.get('payload_text', '')}", flush=True)
            self.data_ready.emit(rec)
        else:
            print("[TTN][WARN] Message received but no Q payload parsed", flush=True)


# ── CubeSat3D ─────────────────────────────────────────────────────────────────
class CubeSat3D(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.setMinimumSize(260, 260)

    def set_quaternion(self, w, x, y, z):
        self.q = (w, x, y, z)
        self.update()

    @staticmethod
    def _quat_to_matrix(w, x, y, z):
        return np.array([
            1-2*(y*y+z*z),  2*(x*y+w*z),   2*(x*z-w*y),  0,
            2*(x*y-w*z),   1-2*(x*x+z*z),  2*(y*z+w*x),  0,
            2*(x*z+w*y),    2*(y*z-w*x),  1-2*(x*x+y*y),  0,
            0,              0,              0,             1,
        ], dtype=np.float32)

    def initializeGL(self):
        if not OPENGL_OK: return
        glClearColor(0.04, 0.06, 0.10, 1.0)
        glEnable(GL_DEPTH_TEST)

    def resizeGL(self, w, h):
        if not OPENGL_OK: return
        h = max(h, 1)
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION); glLoadIdentity()
        a = w / h
        glFrustum(-a*0.5, a*0.5, -0.5, 0.5, 1.0, 10.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        if not OPENGL_OK: return
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -3.5)
        w, x, y, z = self.q
        glMultMatrixf(self._quat_to_matrix(w, x, y, z))
        self._draw_body()
        self._draw_axes()
        self._draw_plane()

    def _draw_plane(self):
        """Plano XY de referencia tenue."""
        if not OPENGL_OK: return
        glColor3f(0.05, 0.15, 0.25)
        glLineWidth(1.0)
        glBegin(GL_LINES)
        s = 1.2
        step = 0.3
        v = -s
        while v <= s + 0.01:
            glVertex3f(v, -s, 0); glVertex3f(v,  s, 0)
            glVertex3f(-s, v, 0); glVertex3f( s, v, 0)
            v += step
        glEnd()
        # Borde del plano más visible
        glColor3f(0.1, 0.3, 0.5)
        glLineWidth(1.5)
        glBegin(GL_LINES)
        glVertex3f(-s,-s,0); glVertex3f( s,-s,0)
        glVertex3f( s,-s,0); glVertex3f( s, s,0)
        glVertex3f( s, s,0); glVertex3f(-s, s,0)
        glVertex3f(-s, s,0); glVertex3f(-s,-s,0)
        glEnd()

    def _draw_body(self):
        s = 0.55
        faces = [
            ((0.06,0.24,0.50),[(-s,-s,s),(s,-s,s),(s,s,s),(-s,s,s)]),
            ((0.04,0.16,0.38),[(-s,-s,-s),(-s,s,-s),(s,s,-s),(s,-s,-s)]),
            ((0.05,0.20,0.44),[(-s,s,-s),(-s,s,s),(s,s,s),(s,s,-s)]),
            ((0.03,0.14,0.32),[(-s,-s,-s),(s,-s,-s),(s,-s,s),(-s,-s,s)]),
            ((0.05,0.18,0.42),[(s,-s,-s),(s,s,-s),(s,s,s),(s,-s,s)]),
            ((0.03,0.12,0.30),[(-s,-s,-s),(-s,-s,s),(-s,s,s),(-s,s,-s)]),
        ]
        glBegin(GL_QUADS)
        for col, verts in faces:
            glColor3f(*col)
            for v in verts: glVertex3f(*v)
        glEnd()
        edges = [
            (-s,-s,-s),(s,-s,-s),(s,-s,-s),(s,s,-s),(s,s,-s),(-s,s,-s),(-s,s,-s),(-s,-s,-s),
            (-s,-s,s),(s,-s,s),(s,-s,s),(s,s,s),(s,s,s),(-s,s,s),(-s,s,s),(-s,-s,s),
            (-s,-s,-s),(-s,-s,s),(s,-s,-s),(s,-s,s),(s,s,-s),(s,s,s),(-s,s,-s),(-s,s,s),
        ]
        glLineWidth(1.5); glColor3f(0.0, 0.65, 1.0)
        glBegin(GL_LINES)
        for v in edges: glVertex3f(*v)
        glEnd()

    def _draw_axes(self):
        L = 1.1
        glLineWidth(3.0)
        glBegin(GL_LINES)
        glColor3f(1.0,0.2,0.2); glVertex3f(0,0,0); glVertex3f(L,0,0)
        glColor3f(0.2,1.0,0.2); glVertex3f(0,0,0); glVertex3f(0,L,0)
        glColor3f(0.2,0.5,1.0); glVertex3f(0,0,0); glVertex3f(0,0,L)
        glEnd()

    def paintEvent(self, event):
        if not OPENGL_OK:
            from PyQt6.QtGui import QPainter
            p = QPainter(self)
            p.fillRect(self.rect(), QColor(10,14,26))
            p.setPen(QColor(255,200,0))
            p.setFont(QFont("monospace",9))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter,
                       "PyOpenGL no disponible\npip install PyOpenGL")
            p.end()
        else:
            super().paintEvent(event)


# ── CompassWidget ─────────────────────────────────────────────────────────────
class CompassWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.heading_deg = 0.0
        self.mx = self.my = self.mz = 0
        self.mag_ok = False
        self.setMinimumSize(260, 260)
        self._history = collections.deque(maxlen=1)

    def set_mag(self, mx, my, mz, mag_ok):
        self.mx = mx; self.my = my; self.mz = mz; self.mag_ok = mag_ok
        if mag_ok and (mx != 0 or my != 0):
            raw = math.degrees(math.atan2(float(my), float(mx))) % 360
            self._history.append(raw)
            sins = sum(math.sin(math.radians(h)) for h in self._history)
            coss = sum(math.cos(math.radians(h)) for h in self._history)
            self.heading_deg = math.degrees(math.atan2(sins, coss)) % 360
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w/2.0, h/2.0
        R = min(w, h)/2.0 - 14

        p.fillRect(self.rect(), QColor(BG))

        ring_col = QColor(GREEN) if self.mag_ok else QColor(RED)
        p.setPen(QPen(ring_col, 2))
        p.setBrush(QBrush(QColor(BG_CARD)))
        p.drawEllipse(QPointF(cx, cy), R, R)

        p.setPen(QPen(QColor(BORDER), 1))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(QPointF(cx, cy), R*0.52, R*0.52)

        for deg in range(0, 360, 10):
            rad = math.radians(deg)
            tick = 12 if deg % 30 == 0 else 5
            col  = QColor(DIM) if deg % 30 == 0 else QColor(BORDER)
            p.setPen(QPen(col, 1))
            x1 = cx + (R-tick)*math.sin(rad); y1 = cy - (R-tick)*math.cos(rad)
            x2 = cx + R*math.sin(rad);         y2 = cy - R*math.cos(rad)
            p.drawLine(QPointF(x1,y1), QPointF(x2,y2))

        for label, deg, color in [("N",0,RED),("E",90,TEXT),("S",180,TEXT),("W",270,TEXT),
                                    ("NE",45,DIM),("SE",135,DIM),("SW",225,DIM),("NW",315,DIM)]:
            rad = math.radians(deg)
            dist = R-24 if len(label)==1 else R-22
            tx = cx + dist*math.sin(rad); ty = cy - dist*math.cos(rad)
            sz = 10 if len(label)==1 else 7
            bold = QFont.Weight.Bold if len(label)==1 else QFont.Weight.Normal
            p.setFont(QFont("monospace", sz, bold))
            p.setPen(QColor(color))
            p.drawText(QPointF(tx - sz*0.35*len(label), ty + sz*0.5), label)

        h_rad = math.radians(self.heading_deg)
        nlen = R*0.70; tlen = R*0.35; hw = 7
        tip_x  = cx + nlen*math.sin(h_rad);  tip_y  = cy - nlen*math.cos(h_rad)
        tail_x = cx - tlen*math.sin(h_rad);  tail_y = cy + tlen*math.cos(h_rad)
        perp = h_rad + math.pi/2
        lx = cx + hw*math.sin(perp); ly = cy - hw*math.cos(perp)
        rx = cx - hw*math.sin(perp); ry = cy + hw*math.cos(perp)

        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(QColor(RED)))
        p.drawPolygon(QPolygonF([QPointF(tip_x,tip_y),QPointF(lx,ly),
                                  QPointF(cx,cy),QPointF(rx,ry)]))
        p.setBrush(QBrush(QColor(TEXT)))
        p.drawPolygon(QPolygonF([QPointF(tail_x,tail_y),QPointF(lx,ly),
                                  QPointF(cx,cy),QPointF(rx,ry)]))

        p.setBrush(QBrush(QColor(BG_CARD)))
        p.setPen(QPen(QColor(ACCENT), 1))
        p.drawEllipse(QPointF(cx, cy), 7, 7)

        p.setFont(QFont("monospace", 14, QFont.Weight.Bold))
        p.setPen(QColor(ACCENT) if self.mag_ok else QColor(RED))
        p.drawText(QPointF(cx-38, cy+R*0.36), f"{self.heading_deg:05.1f}°")

        p.setFont(QFont("monospace", 8))
        p.setPen(QColor(GREEN) if self.mag_ok else QColor(AMBER))
        p.drawText(QPointF(cx-20, cy+R*0.50), "MAG OK" if self.mag_ok else "MAG ERR")

        p.setFont(QFont("monospace", 7))
        p.setPen(QColor(DIM))
        p.drawText(QPointF(6, h-28), f"MX={self.mx:+6d}")
        p.drawText(QPointF(6, h-16), f"MY={self.my:+6d}")
        p.drawText(QPointF(6, h-4),  f"MZ={self.mz:+6d}")
        p.end()


# ── MainWindow ────────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OBC Fusion AHRS Monitor")
        self.resize(1280, 800)

        self.buf_usb = TelemetryBuffer()
        self.buf_ttn = TelemetryBuffer()
        self._worker_usb = self._thread_usb = None
        self._worker_ttn = self._thread_ttn = None
        self._pkt_total = self._hz_count = 0
        self._pkt_ttn = 0
        self._hz_ts = time.time(); self._hz = 0.0

        self._apply_theme()
        self._build()
        self._splash = None
        self._splash_bg = None
        if self._logo_pixmap is not None:
            QTimer.singleShot(50, self._start_logo_intro)

        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh)
        self._timer.start(50)

    def _load_logo_pixmap(self):
        candidates = [
            "/home/eduardonovoac/cubesat_logo.png",
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "cubesat_logo.png"),
            os.path.join(os.path.expanduser("~"), "cubesat_logo.png"),
        ]
        for path in candidates:
            if os.path.isfile(path):
                pix = QPixmap(path)
                if not pix.isNull():
                    return pix
        return None

    def _start_logo_intro(self):
        """Splash de apertura: fondo (color de fondo de la GUI) + logo grande
        centrado se quedan quietos ~3 s, luego el fondo se difumina rápido
        mientras el logo se achica y se mueve hasta self.logo_label (esquina
        de la barra de conexión)."""
        big = 480

        self._splash_bg = QWidget(self._root)
        self._splash_bg.setStyleSheet(f"background-color:{BG};")
        self._splash_bg.setGeometry(self._root.rect())
        self._splash_bg.show()

        self._splash = QLabel(self._root)
        self._splash.setPixmap(self._logo_pixmap)
        self._splash.setScaledContents(True)
        self._splash.setStyleSheet("background: transparent;")
        self._splash.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self._splash.setAlignment(Qt.AlignmentFlag.AlignCenter)
        start_rect = QRect(0, 0, big, big)
        start_rect.moveCenter(self._root.rect().center())
        self._splash.setGeometry(start_rect)
        self._splash.show()

        self._splash_bg.raise_()
        self._splash.raise_()

        QTimer.singleShot(3000, self._end_logo_hold)

    def _end_logo_hold(self):
        self._fade_out_splash_bg()
        self._animate_logo_intro()

    def _fade_out_splash_bg(self):
        if self._splash_bg is None:
            return
        effect = QGraphicsOpacityEffect(self._splash_bg)
        self._splash_bg.setGraphicsEffect(effect)

        anim = QPropertyAnimation(effect, b"opacity", self)
        anim.setDuration(300)
        anim.setStartValue(1.0)
        anim.setEndValue(0.0)
        anim.setEasingCurve(QEasingCurve.Type.OutCubic)
        anim.finished.connect(self._finish_splash_bg)
        anim.start()
        self._splash_bg_anim = anim  # mantener referencia viva mientras corre

    def _finish_splash_bg(self):
        if self._splash_bg is not None:
            self._splash_bg.hide()
            self._splash_bg.deleteLater()
            self._splash_bg = None

    def _animate_logo_intro(self):
        if self._splash is None:
            return
        top_left = self.logo_label.mapTo(self._root, QPoint(0, 0))
        end_rect = QRect(top_left, self.logo_label.size())

        anim = QPropertyAnimation(self._splash, b"geometry", self)
        anim.setDuration(600)
        anim.setStartValue(self._splash.geometry())
        anim.setEndValue(end_rect)
        anim.setEasingCurve(QEasingCurve.Type.OutCubic)
        anim.finished.connect(self._finish_logo_intro)
        anim.start()
        self._logo_intro_anim = anim  # mantener referencia viva mientras corre

    def _finish_logo_intro(self):
        if self._splash is not None:
            self._splash.hide()
            self._splash.deleteLater()
            self._splash = None

    def _apply_theme(self):
        QApplication.setStyle("Fusion")
        pal = QPalette()
        pal.setColor(QPalette.ColorRole.Window,        QColor(BG))
        pal.setColor(QPalette.ColorRole.WindowText,    QColor(TEXT))
        pal.setColor(QPalette.ColorRole.Base,          QColor(BG_MID))
        pal.setColor(QPalette.ColorRole.AlternateBase, QColor(BG_CARD))
        pal.setColor(QPalette.ColorRole.Text,          QColor(TEXT))
        pal.setColor(QPalette.ColorRole.Button,        QColor(BG_CARD))
        pal.setColor(QPalette.ColorRole.ButtonText,    QColor(TEXT))
        pal.setColor(QPalette.ColorRole.Highlight,     QColor(ACCENT))
        QApplication.setPalette(pal)

    def _card(self, title, color=ACCENT):
        g = QGroupBox(title)
        g.setStyleSheet(f"""
            QGroupBox {{color:{color};font:bold 9pt monospace;
                border:1px solid {color}40;border-radius:5px;
                margin-top:8px;padding-top:4px;}}
            QGroupBox::title{{subcontrol-origin:margin;left:8px;padding:0 3px;}}
        """)
        return g

    def _row(self, layout, label):
        w = QWidget(); h = QHBoxLayout(w); h.setContentsMargins(0,1,0,1)
        k = QLabel(label); k.setFont(QFont("monospace",8))
        k.setStyleSheet(f"color:{DIM};")
        v = QLabel("—"); v.setFont(QFont("monospace",10))
        v.setStyleSheet(f"color:{TEXT};")
        v.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignVCenter)
        h.addWidget(k); h.addStretch(); h.addWidget(v)
        layout.addWidget(w)
        return v

    def _btn(self, text, color=ACCENT):
        b = QPushButton(text)
        b.setFont(QFont("monospace",9,QFont.Weight.Bold))
        b.setFixedHeight(26)
        b.setStyleSheet(
            f"QPushButton{{background:{BG};color:{color};"
            f"border:1px solid {color}55;border-radius:4px;padding:3px 10px;}}"
            f"QPushButton:hover{{background:{color}20;}}")
        return b

    def _build(self):
        root = QWidget(); self.setCentralWidget(root)
        vl = QVBoxLayout(root); vl.setContentsMargins(6,6,6,6); vl.setSpacing(4)

        # ── Barra de conexión ─────────────────────────────────────────────────
        conn = QWidget()
        conn.setStyleSheet(f"background:{BG_CARD};border-radius:5px;")
        cl = QHBoxLayout(conn); cl.setContentsMargins(8,6,8,6); cl.setSpacing(10)

        usb_g = self._card("USB  UART2  115200", CYAN)
        ul = QHBoxLayout(usb_g)
        self.usb_port = QLineEdit("/dev/ttyACM0")
        self.usb_port.setFixedWidth(130)
        self.usb_port.setStyleSheet(f"background:{BG};color:{TEXT};border:1px solid {BORDER};border-radius:3px;padding:2px 5px;font:9pt monospace;")
        self.btn_usb_conn = self._btn("Connect", CYAN)
        self.btn_usb_disc = self._btn("Disconnect", RED)
        self.lbl_usb_st = QLabel("●"); self.lbl_usb_st.setStyleSheet(f"color:{RED};font-size:14px;")
        self.btn_usb_conn.clicked.connect(self._conn_usb)
        self.btn_usb_disc.clicked.connect(self._disc_usb)
        ul.addWidget(QLabel("Port:")); ul.addWidget(self.usb_port)
        ul.addWidget(self.btn_usb_conn); ul.addWidget(self.btn_usb_disc); ul.addWidget(self.lbl_usb_st)
        cl.addWidget(usb_g, 1)

        ttn_g = self._card("TTN MQTT  LoRaWAN", GREEN)
        tl = QHBoxLayout(ttn_g)
        self.ttn_app = QLineEdit("connection-lora")
        self.ttn_dev = QLineEdit("stm32-obc-usm")
        self.ttn_key = QLineEdit("")
        self.ttn_key.setEchoMode(QLineEdit.EchoMode.Password)
        self.ttn_key.setPlaceholderText("TTN API key")
        for w, width in [(self.ttn_app, 170), (self.ttn_dev, 140), (self.ttn_key, 220)]:
            w.setFixedWidth(width)
            w.setStyleSheet(f"background:{BG};color:{TEXT};border:1px solid {BORDER};border-radius:3px;padding:2px 5px;font:9pt monospace;")
        self.btn_ttn_conn = self._btn("Connect", GREEN)
        self.btn_ttn_disc = self._btn("Disconnect", RED)
        self.lbl_ttn_st = QLabel("●"); self.lbl_ttn_st.setStyleSheet(f"color:{RED};font-size:14px;")
        self.btn_ttn_conn.clicked.connect(self._conn_ttn)
        self.btn_ttn_disc.clicked.connect(self._disc_ttn)
        tl.addWidget(QLabel("App:")); tl.addWidget(self.ttn_app)
        tl.addWidget(QLabel("Dev:")); tl.addWidget(self.ttn_dev)
        tl.addWidget(QLabel("Key:")); tl.addWidget(self.ttn_key)
        tl.addWidget(self.btn_ttn_conn); tl.addWidget(self.btn_ttn_disc); tl.addWidget(self.lbl_ttn_st)
        cl.addWidget(ttn_g, 2)

        self.btn_clear = self._btn("Clear", AMBER)
        self.btn_clear.clicked.connect(self._clear)
        cl.addWidget(self.btn_clear)

        self.logo_label = QLabel()
        self.logo_label.setFixedSize(40, 40)
        self.logo_label.setScaledContents(True)
        self.logo_label.setStyleSheet("background: transparent;")
        self._logo_pixmap = self._load_logo_pixmap()
        if self._logo_pixmap is not None:
            self.logo_label.setPixmap(self._logo_pixmap)
        cl.addWidget(self.logo_label)

        vl.addWidget(conn)
        self._root = root

        # ── Splitter ──────────────────────────────────────────────────────────
        split = QSplitter(Qt.Orientation.Horizontal)
        split.setStyleSheet(f"QSplitter::handle{{background:{BORDER};width:2px;}}")

        # Panel izquierdo
        left = QWidget()
        ll = QVBoxLayout(left); ll.setContentsMargins(0,0,4,0); ll.setSpacing(5)

        qg = self._card("QUATERNION  (×10000→float)", GREEN)
        ql = QVBoxLayout(qg)
        self.lbl_q0    = self._row(ql, "q0  (w)")
        self.lbl_q1    = self._row(ql, "q1  (x)")
        self.lbl_q2    = self._row(ql, "q2  (y)")
        self.lbl_q3    = self._row(ql, "q3  (z)")
        self.lbl_qnorm = self._row(ql, "|q|")
        ll.addWidget(qg)

        mg2 = self._card("MAG RAW [LSB]", PINK)
        ml2 = QVBoxLayout(mg2)
        self.lbl_mx  = self._row(ml2, "MX")
        self.lbl_my  = self._row(ml2, "MY")
        self.lbl_mz  = self._row(ml2, "MZ")
        self.lbl_hdg = self._row(ml2, "Heading")
        ll.addWidget(mg2)

        ig = self._card("IMU RAW [LSB]", CYAN)
        il = QVBoxLayout(ig)
        self.lbl_ax = self._row(il, "AX"); self.lbl_ay = self._row(il, "AY")
        self.lbl_az = self._row(il, "AZ"); self.lbl_gx = self._row(il, "GX")
        self.lbl_gy = self._row(il, "GY"); self.lbl_gz = self._row(il, "GZ")
        ll.addWidget(ig)

        sg = self._card("ESTADO", AMBER)
        sl = QVBoxLayout(sg)
        self.lbl_tms   = self._row(sl, "T [ms]")
        self.lbl_mag   = self._row(sl, "MAG")
        self.lbl_hz    = self._row(sl, "Rx rate")
        self.lbl_total = self._row(sl, "Paquetes")
        ll.addWidget(sg); ll.addStretch()
        split.addWidget(left)

        # ── Tabs ──────────────────────────────────────────────────────────────
        tabs = QTabWidget()
        tabs.setStyleSheet(f"""
            QTabWidget::pane{{border:1px solid {BORDER};border-radius:4px;}}
            QTabBar::tab{{background:{BG_CARD};color:{DIM};padding:4px 14px;
                font:8pt monospace;border-radius:3px 3px 0 0;}}
            QTabBar::tab:selected{{color:{ACCENT};background:{BG_MID};}}
        """)

        # ── Tab 1: Actitud + Brújula combinadas ───────────────────────────────
        tab_combo = QWidget()
        tc = QVBoxLayout(tab_combo); tc.setContentsMargins(4,4,4,4); tc.setSpacing(6)

        # Nota informativa
        note = QLabel(
            "Cubo 3D  ·  Rojo=X  Verde=Y  Azul=Z  ·  Plano XY de referencia  "
            "│  Brújula  ·  Aguja ROJA → Norte magnético  ·  Aro VERDE = MAG OK")
        note.setWordWrap(True)
        note.setStyleSheet(f"color:{AMBER};font:8pt monospace;padding:3px;background:{BG_CARD};border-radius:3px;")
        tc.addWidget(note)

        # Fila principal: cubo cable | cubo LoRaWAN | brújula
        row = QHBoxLayout(); row.setSpacing(8)

        cube_wrap = QWidget()
        cube_wrap.setStyleSheet(f"background:{BG_CARD};border:1px solid {BORDER};border-radius:4px;")
        cw_l = QVBoxLayout(cube_wrap); cw_l.setContentsMargins(4,4,4,4); cw_l.setSpacing(2)
        lbl_cube = QLabel("CABLE / USB  —  Quaternion")
        lbl_cube.setStyleSheet(f"color:{CYAN};font:bold 8pt monospace;")
        lbl_cube.setAlignment(Qt.AlignmentFlag.AlignCenter)
        cw_l.addWidget(lbl_cube)
        self.cube3d = CubeSat3D()
        self.cube3d.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        cw_l.addWidget(self.cube3d)
        self.lbl_usb_q = QLabel("USB: sin datos")
        self.lbl_usb_q.setStyleSheet(f"color:{DIM};font:7pt monospace;")
        self.lbl_usb_q.setAlignment(Qt.AlignmentFlag.AlignCenter)
        cw_l.addWidget(self.lbl_usb_q)
        row.addWidget(cube_wrap, 1)

        lora_wrap = QWidget()
        lora_wrap.setStyleSheet(f"background:{BG_CARD};border:1px solid {BORDER};border-radius:4px;")
        lw_l = QVBoxLayout(lora_wrap); lw_l.setContentsMargins(4,4,4,4); lw_l.setSpacing(2)
        lbl_lora = QLabel("LoRaWAN / TTN  —  Quaternion")
        lbl_lora.setStyleSheet(f"color:{GREEN};font:bold 8pt monospace;")
        lbl_lora.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lw_l.addWidget(lbl_lora)
        self.cube_lora = CubeSat3D()
        self.cube_lora.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        lw_l.addWidget(self.cube_lora)
        self.lbl_lora_q = QLabel("TTN: sin datos")
        self.lbl_lora_q.setStyleSheet(f"color:{DIM};font:7pt monospace;")
        self.lbl_lora_q.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lw_l.addWidget(self.lbl_lora_q)
        row.addWidget(lora_wrap, 1)

        # Brújula con etiqueta
        comp_wrap = QWidget()
        comp_wrap.setStyleSheet(f"background:{BG_CARD};border:1px solid {BORDER};border-radius:4px;")
        comp_wrap.setFixedWidth(320)
        cmp_l = QVBoxLayout(comp_wrap); cmp_l.setContentsMargins(4,4,4,4); cmp_l.setSpacing(2)
        lbl_comp = QLabel("BRÚJULA  —  heading = atan2(MY, MX)")
        lbl_comp.setStyleSheet(f"color:{PINK};font:bold 8pt monospace;")
        lbl_comp.setAlignment(Qt.AlignmentFlag.AlignCenter)
        cmp_l.addWidget(lbl_comp)
        self.compass = CompassWidget()
        self.compass.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        cmp_l.addWidget(self.compass)
        # Referencia headings
        ref = QLabel("0°=N   90°=E   180°=S   270°=W   (sensor horizontal)")
        ref.setStyleSheet(f"color:{DIM};font:7pt monospace;")
        ref.setAlignment(Qt.AlignmentFlag.AlignCenter)
        cmp_l.addWidget(ref)
        row.addWidget(comp_wrap)

        tc.addLayout(row, 1)
        tabs.addTab(tab_combo, "Actitud & Brújula")

        # ── Tab 2: Convergencia ───────────────────────────────────────────────
        pg.setConfigOption("background", BG_MID)
        pg.setConfigOption("foreground", TEXT)

        tab_conv = QWidget()
        tcl = QVBoxLayout(tab_conv); tcl.setContentsMargins(4,4,4,4); tcl.setSpacing(4)
        note2 = QLabel(
            "Quaternion vs tiempo  ·  En reposo: q0→estable, q1/q2/q3→constante  "
            "·  |q|≈1.0 siempre  ·  Varianza baja cuando FusionBias corrige el bias del giróscopo")
        note2.setWordWrap(True)
        note2.setStyleSheet(f"color:{AMBER};font:8pt monospace;padding:3px;")
        tcl.addWidget(note2)

        self.pw_q = pg.PlotWidget()
        self.pw_q.setLabel("left","Componente quaternion")
        self.pw_q.setLabel("bottom","t [s]")
        self.pw_q.showGrid(x=True,y=True,alpha=0.25)
        self.pw_q.addLegend(offset=(5,5))
        self.pw_q.setSizePolicy(QSizePolicy.Policy.Expanding,QSizePolicy.Policy.Expanding)
        self.pw_q.setYRange(-1.1,1.1)
        self.curve_q0 = self.pw_q.plot(pen=pg.mkPen(GREEN, width=2),   name="q0 (w)")
        self.curve_q1 = self.pw_q.plot(pen=pg.mkPen(RED,   width=1.5), name="q1 (x)")
        self.curve_q2 = self.pw_q.plot(pen=pg.mkPen(CYAN,  width=1.5), name="q2 (y)")
        self.curve_q3 = self.pw_q.plot(pen=pg.mkPen(AMBER, width=1.5), name="q3 (z)")
        self.pw_q.addLine(y=0,pen=pg.mkPen(DIM,width=1,style=Qt.PenStyle.DashLine))
        tcl.addWidget(self.pw_q,1)

        self.pw_norm = pg.PlotWidget()
        self.pw_norm.setLabel("left","|q| (debe ser ≈1)")
        self.pw_norm.setLabel("bottom","t [s]")
        self.pw_norm.showGrid(x=True,y=True,alpha=0.25)
        self.pw_norm.setMaximumHeight(130)
        self.pw_norm.setYRange(0.9,1.1)
        self.pw_norm.addLine(y=1.0,pen=pg.mkPen(GREEN,width=1,style=Qt.PenStyle.DashLine))
        self.curve_norm = self.pw_norm.plot(pen=pg.mkPen(GREEN,width=1.5))
        tcl.addWidget(self.pw_norm)
        tabs.addTab(tab_conv,"Convergencia / Bias")

        split.addWidget(tabs)
        split.setSizes([240, 1000])
        vl.addWidget(split,1)

        self.statusBar().setStyleSheet(f"background:{BG_CARD};color:{DIM};font:8pt monospace;")
        self.statusBar().showMessage("Conecta un puerto para comenzar")

    # ── Conexión ──────────────────────────────────────────────────────────────
    def _conn_usb(self):
        self._disc_usb()
        w = SerialWorker(self.usb_port.text().strip(), 115200, False)
        t = QThread(); w.moveToThread(t)
        w.data_ready.connect(self._on_data_usb)
        w.status_sig.connect(lambda ok,e: self._set_status(self.lbl_usb_st,ok,e))
        t.started.connect(w.start_reading)
        self._worker_usb = w; self._thread_usb = t; t.start()

    def _disc_usb(self):
        if self._worker_usb: self._worker_usb.stop(); self._worker_usb = None
        if self._thread_usb: self._thread_usb.quit(); self._thread_usb.wait(1000); self._thread_usb = None
        self.lbl_usb_st.setStyleSheet(f"color:{RED};font-size:14px;")

    def _conn_ttn(self):
        self._disc_ttn()
        w = TtnMqttWorker(self.ttn_app.text(), self.ttn_dev.text(), self.ttn_key.text())
        t = QThread(); w.moveToThread(t)
        w.data_ready.connect(self._on_data_ttn)
        w.status_sig.connect(lambda ok,e: self._set_status(self.lbl_ttn_st,ok,e))
        t.started.connect(w.start_reading)
        self._worker_ttn = w; self._thread_ttn = t; t.start()

    def _disc_ttn(self):
        if self._worker_ttn:
            self._worker_ttn.stop()
            self._worker_ttn = None
        if self._thread_ttn:
            self._thread_ttn.quit()
            if not self._thread_ttn.wait(3000):
                self.statusBar().showMessage("TTN MQTT no cerro a tiempo; forzando cierre del hilo")
                self._thread_ttn.terminate()
                self._thread_ttn.wait(1000)
            self._thread_ttn = None
        self.lbl_ttn_st.setStyleSheet(f"color:{RED};font-size:14px;")

    def _set_status(self, lbl, ok, err):
        lbl.setStyleSheet(f"color:{GREEN if ok else RED};font-size:14px;")
        if err:
            self.statusBar().showMessage(("OK: " if ok else "Error: ") + err)

    def _on_data_usb(self, rec): self.buf_usb.push(rec); self._on_record(rec, "usb")
    def _on_data_ttn(self, rec): self.buf_ttn.push(rec); self._on_record(rec, "ttn")

    def _on_record(self, rec, source="usb"):
        self._pkt_total += 1; self._hz_count += 1
        now = time.time()
        if now - self._hz_ts >= 1.0:
            self._hz = self._hz_count/(now-self._hz_ts)
            self._hz_count = 0; self._hz_ts = now

        q0f=rec["q0f"]; q1f=rec["q1f"]; q2f=rec["q2f"]; q3f=rec["q3f"]
        norm = math.sqrt(q0f**2+q1f**2+q2f**2+q3f**2)

        self.lbl_q0.setText(f"{rec['q0']:+6d}  →  {q0f:+.4f}")
        self.lbl_q1.setText(f"{rec['q1']:+6d}  →  {q1f:+.4f}")
        self.lbl_q2.setText(f"{rec['q2']:+6d}  →  {q2f:+.4f}")
        self.lbl_q3.setText(f"{rec['q3']:+6d}  →  {q3f:+.4f}")
        col = GREEN if abs(norm-1.0)<0.05 else AMBER
        self.lbl_qnorm.setText(f"{norm:.5f}")
        self.lbl_qnorm.setStyleSheet(f"color:{col};font:bold 10pt monospace;")

        self.lbl_ax.setText(str(rec.get("ax","—"))); self.lbl_ay.setText(str(rec.get("ay","—")))
        self.lbl_az.setText(str(rec.get("az","—"))); self.lbl_gx.setText(str(rec.get("gx","—")))
        self.lbl_gy.setText(str(rec.get("gy","—"))); self.lbl_gz.setText(str(rec.get("gz","—")))

        tms = rec.get("t_ms")
        self.lbl_tms.setText(str(tms) if tms else "—")
        mag_ok = rec.get("mag_ok", False)
        self.lbl_mag.setText("OK" if mag_ok else "ERR")
        self.lbl_mag.setStyleSheet(f"color:{GREEN if mag_ok else AMBER};font:bold 10pt monospace;")
        self.lbl_total.setText(str(self._pkt_total))

        mx=rec.get("mx",0); my=rec.get("my",0); mz=rec.get("mz",0)
        self.lbl_mx.setText(str(mx)); self.lbl_my.setText(str(my)); self.lbl_mz.setText(str(mz))
        if mag_ok and (mx!=0 or my!=0):
            hdg = math.degrees(math.atan2(float(my),float(mx))) % 360
            self.lbl_hdg.setText(f"{hdg:.1f}°")
        else:
            self.lbl_hdg.setText("—")

        if source == "ttn":
            self._pkt_ttn += 1
            self.cube_lora.set_quaternion(q0f, q1f, q2f, q3f)
            meta = []
            if rec.get("rssi") is not None: meta.append(f"RSSI={rec['rssi']}")
            if rec.get("snr") is not None: meta.append(f"SNR={rec['snr']}")
            self.lbl_lora_q.setText(f"{rec['q0']:+6d},{rec['q1']:+6d},{rec['q2']:+6d},{rec['q3']:+6d}  " + " ".join(meta))
        else:
            self.compass.set_mag(mx, my, mz, mag_ok)
            self.cube3d.set_quaternion(q0f, q1f, q2f, q3f)
            self.lbl_usb_q.setText(f"{rec['q0']:+6d},{rec['q1']:+6d},{rec['q2']:+6d},{rec['q3']:+6d}")

    def _clear(self):
        self.buf_usb.clear(); self.buf_ttn.clear()
        self._pkt_total = 0; self._pkt_ttn = 0
        for c in [self.curve_q0,self.curve_q1,self.curve_q2,self.curve_q3,self.curve_norm]:
            c.setData([],[])
        self.statusBar().showMessage("Datos limpiados")

    def _refresh(self):
        self.lbl_hz.setText(f"{self._hz:.1f} Hz")
        snap = self.buf_usb.snapshot()
        if len(snap["t_s"])<2: return
        t = snap["t_s"]
        self.curve_q0.setData(t, snap["q0f"]); self.curve_q1.setData(t, snap["q1f"])
        self.curve_q2.setData(t, snap["q2f"]); self.curve_q3.setData(t, snap["q3f"])
        norm = np.sqrt(snap["q0f"]**2+snap["q1f"]**2+snap["q2f"]**2+snap["q3f"]**2)
        self.curve_norm.setData(t, norm)

    def closeEvent(self, event):
        self._disc_usb(); self._disc_ttn(); self._timer.stop(); event.accept()


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("OBC Fusion Monitor")
    win = MainWindow(); win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
