#!/usr/bin/env python3
"""
Controlador de bajo nivel para el Unitree G1 (29 DOF) por DDS.

Sirve TANTO para el simulador (unitree_mujoco) COMO para el robot real:
el mismo codigo funciona en los dos, solo cambia la red.

Uso:
    python g1_control.py sim [demo]           # simulador (dominio 1, iface "lo0")
    python g1_control.py real <iface> [demo]  # robot real (dominio 0, iface de red)

    demo: demo | stand | wave | squat | march | twist | sixseven   (default: demo)
          - demo     : recorre todas las rutinas en secuencia (y repite)
          - stand    : mantiene la postura de parado
          - wave     : saluda con el brazo derecho
          - squat    : sentadillas suaves
          - march    : marcha en el lugar
          - twist    : gira el torso (waist yaw)
          - sixseven : meme "6 7" (manos al frente subiendo y bajando)

    Gestos locales extra: si existe el archivo tp_g1/local_extras.py (no
    versionado), sus demos se agregan automaticamente (ver POSE_EXTRAS).

Cortar con Ctrl+C.

NOTA IMPORTANTE (robot real): el G1 solo acepta comandos de bajo nivel
cuando el servicio de locomocion (sport_mode) esta apagado. Con el argumento
"real" se intenta liberar automaticamente via MotionSwitcherClient.
"""
import sys
import time
import math

import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

# Gestos locales opcionales: si existe tp_g1/local_extras.py (no versionado),
# puede registrar demos extra como POSE_EXTRAS = {"nombre": funcion(t)}.
try:
    from local_extras import POSE_EXTRAS as _POSE_EXTRAS
except Exception:
    _POSE_EXTRAS = {}

NUM_MOTOR = 29

# --- Indices de las juntas (unitree_hg, G1 29 DOF) -------------------------
# Pierna izquierda
L_HIP_PITCH, L_HIP_ROLL, L_HIP_YAW, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL = 0, 1, 2, 3, 4, 5
# Pierna derecha
R_HIP_PITCH, R_HIP_ROLL, R_HIP_YAW, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL = 6, 7, 8, 9, 10, 11
# Cintura
WAIST_YAW, WAIST_ROLL, WAIST_PITCH = 12, 13, 14
# Brazo izquierdo
L_SHOULDER_PITCH, L_SHOULDER_ROLL, L_SHOULDER_YAW, L_ELBOW, L_WRIST_ROLL, L_WRIST_PITCH, L_WRIST_YAW = 15, 16, 17, 18, 19, 20, 21
# Brazo derecho
R_SHOULDER_PITCH, R_SHOULDER_ROLL, R_SHOULDER_YAW, R_ELBOW, R_WRIST_ROLL, R_WRIST_PITCH, R_WRIST_YAW = 22, 23, 24, 25, 26, 27, 28

# Ganancias PD (mismas que usa Unitree en sus ejemplos)
KP = np.array([60, 60, 60, 100, 40, 40,   # pierna izq
               60, 60, 60, 100, 40, 40,   # pierna der
               100, 60, 60,               # cintura
               40, 40, 40, 40, 40, 40, 40,  # brazo izq
               40, 40, 40, 40, 40, 40, 40], dtype=float)  # brazo der
KD = np.array([1, 1, 1, 2, 1, 1,
               1, 1, 1, 2, 1, 1,
               2, 2, 2,
               1, 1, 1, 1, 1, 1, 1,
               1, 1, 1, 1, 1, 1, 1], dtype=float)

# Postura de "parado" (todas las juntas a cero = piernas estiradas)
STAND = np.zeros(NUM_MOTOR)


def pose_for(demo, t):
    """Devuelve el vector q_deseado (29) para la demo en el instante t."""
    if demo in _POSE_EXTRAS:
        return np.asarray(_POSE_EXTRAS[demo](t), dtype=float)

    q = STAND.copy()

    if demo == "stand":
        pass

    elif demo == "wave":
        # Saludo: brazo derecho levantado al frente/arriba, la mano oscila de
        # costado. OJO: en el G1 shoulder_pitch NEGATIVO = brazo hacia adelante.
        q[R_SHOULDER_PITCH] = -1.35
        q[R_SHOULDER_ROLL] = -0.6 + 0.4 * math.sin(2 * math.pi * 1.4 * t)
        q[R_ELBOW] = 0.9
        q[L_SHOULDER_ROLL] = 0.25

    elif demo == "squat":
        a = 0.5 * (1 - math.cos(2 * math.pi * 0.4 * t))  # suave 0..1
        q[[L_HIP_PITCH, R_HIP_PITCH]] = -0.5 * a
        q[[L_KNEE, R_KNEE]] = 1.0 * a
        q[[L_ANKLE_PITCH, R_ANKLE_PITCH]] = -0.5 * a
        q[[L_SHOULDER_PITCH, R_SHOULDER_PITCH]] = 0.6 * a
        q[[L_ELBOW, R_ELBOW]] = 0.6 * a

    elif demo == "march":
        s = math.sin(2 * math.pi * 0.6 * t)
        aL, aR = 0.5 * (1 + s), 0.5 * (1 - s)
        q[L_HIP_PITCH], q[L_KNEE], q[L_ANKLE_PITCH] = 0.35 * aL, 0.5 * aL, -0.2 * aL
        q[R_HIP_PITCH], q[R_KNEE], q[R_ANKLE_PITCH] = 0.35 * aR, 0.5 * aR, -0.2 * aR
        q[[L_ELBOW, R_ELBOW]] = 0.5
        q[[L_SHOULDER_PITCH, R_SHOULDER_PITCH]] = 0.2 * s

    elif demo == "twist":
        q[WAIST_YAW] = 0.4 * math.sin(2 * math.pi * 0.5 * t)
        q[[L_SHOULDER_ROLL, R_SHOULDER_ROLL]] = 0.3
        q[[L_SHOULDER_PITCH, R_SHOULDER_PITCH]] = 0.3
        q[[L_ELBOW, R_ELBOW]] = 0.8

    elif demo == "sixseven":
        # Meme "6 7": manos ARRIBA a la altura de la cara, palmas enfrentadas,
        # suben y bajan alternadas (balancín) mientras se dice "six... seven".
        s = math.sin(2 * math.pi * 1.6 * t)
        q[L_SHOULDER_PITCH] = -1.45 - 0.22 * s
        q[R_SHOULDER_PITCH] = -1.45 + 0.22 * s
        q[L_SHOULDER_ROLL] = -0.10
        q[R_SHOULDER_ROLL] = 0.10
        q[L_ELBOW] = 1.30 + 0.12 * s
        q[R_ELBOW] = 1.30 - 0.12 * s
        q[L_WRIST_ROLL] = 1.40
        q[R_WRIST_ROLL] = -1.40

    return q


# Secuencia para la demo combinada: (nombre, duracion)
DEMO_SEQUENCE = [("stand", 2.0), ("wave", 5.0), ("squat", 8.0),
                 ("march", 8.0), ("twist", 5.0), ("sixseven", 6.0), ("stand", 2.0)]


def sequence_pose(t):
    """Recorre DEMO_SEQUENCE en loop y devuelve la pose del instante t."""
    total = sum(d for _, d in DEMO_SEQUENCE)
    t = t % total
    acc = 0.0
    for name, dur in DEMO_SEQUENCE:
        if t < acc + dur:
            return pose_for(name, t - acc)
        acc += dur
    return STAND.copy()


class G1LowLevelController:
    def __init__(self, demo="demo"):
        self.demo = demo
        self.control_dt = 0.005          # 200 Hz
        self.max_vel = 5.0               # velocidad máx. de las juntas [rad/s] (suaviza saltos)
        self.time = 0.0
        self.ticks = 0                   # contador de publicaciones (heartbeat)
        self.q_cmd = STAND.copy()        # postura actualmente comandada (con slew-rate)
        self.mode_machine = 0
        self.low_state = None
        self.got_state = False
        self.crc = CRC()
        self.cmd = unitree_hg_msg_dds__LowCmd_()
        for i in range(NUM_MOTOR):
            self.cmd.motor_cmd[i].mode = 1  # 1 = habilitado
        self.publisher = None
        self.thread = None

    def Init(self):
        # Suscribirse al estado del robot (para conocer mode_machine y estado inicial)
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self._LowStateHandler, 10)

        # Publicador de comandos de motor
        self.publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.publisher.Init()

        # Esperar el primer mensaje de estado
        t0 = time.time()
        while not self.got_state and time.time() - t0 < 5.0:
            time.sleep(0.05)
        if not self.got_state:
            print("[AVISO] No llegó LowState. ¿Está corriendo el simulador/robot en el dominio correcto?")

    def _LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        self.mode_machine = msg.mode_machine
        self.got_state = True

    def Start(self):
        self.thread = RecurrentThread(
            interval=self.control_dt, target=self._Write, name="g1_lowcmd"
        )
        self.thread.Start()

    def _Write(self):
        self.time += self.control_dt
        self.ticks += 1

        if self.demo == "demo":
            qd = sequence_pose(self.time)
        else:
            qd = pose_for(self.demo, self.time)

        # Limitación de velocidad: acerca la postura comandada a la deseada
        # a lo sumo max_vel rad/s (evita saltos bruscos entre rutinas).
        step = self.max_vel * self.control_dt
        self.q_cmd += np.clip(qd - self.q_cmd, -step, step)

        for i in range(NUM_MOTOR):
            self.cmd.motor_cmd[i].q = self.q_cmd[i]
            self.cmd.motor_cmd[i].dq = 0.0
            self.cmd.motor_cmd[i].kp = KP[i]
            self.cmd.motor_cmd[i].kd = KD[i]
            self.cmd.motor_cmd[i].tau = 0.0
        self.cmd.mode_pr = 0            # control serie (P/R)
        self.cmd.mode_machine = self.mode_machine
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.publisher.Write(self.cmd)


def main():
    args = sys.argv[1:]
    real = False
    iface = "lo0"
    domain = 1

    if args and args[0] == "real":
        real = True
        domain = 0
        if len(args) < 2:
            print("Falta la interfaz de red. Ej: python g1_control.py real en0 demo")
            sys.exit(1)
        iface = args[1]
        args = args[1:]
    elif args and args[0] == "sim":
        args = args[1:]

    demo = args[0] if args else "demo"

    print(f"Modo: {'REAL' if real else 'SIMULACIÓN'} | iface={iface} dominio={domain} | demo={demo}")

    ChannelFactoryInitialize(domain, iface)

    if real:
        # Liberar el servicio de locomoción para poder mandar comandos de bajo nivel
        try:
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
            msc = MotionSwitcherClient()
            msc.SetTimeout(5.0)
            msc.Init()
            status, result = msc.CheckMode()
            while result.get("name"):
                msc.ReleaseMode()
                status, result = msc.CheckMode()
                time.sleep(1)
            print("Sport mode liberado. Robot listo para bajo nivel.")
        except Exception as e:
            print("[AVISO] No se pudo usar MotionSwitcherClient:", e)

    ctrl = G1LowLevelController(demo=demo)
    ctrl.Init()
    ctrl.Start()
    print(f"Publicando comandos a {int(round(1/ctrl.control_dt))} Hz. Ctrl+C para salir.")
    try:
        last = 0
        while True:
            time.sleep(1.0)
            print(f"  [t={ctrl.time:5.1f}s] publicaciones={ctrl.ticks} (+{ctrl.ticks - last}/s) tarea={ctrl.demo}")
            last = ctrl.ticks
    except KeyboardInterrupt:
        print("\nSaliendo...")


if __name__ == "__main__":
    main()
