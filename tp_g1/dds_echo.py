"""
Prueba de humo de DDS (cyclonedds) entre dos procesos en macOS.

Uso:
    python dds_echo.py sub [iface]   # se queda escuchando rt/lowstate
    python dds_echo.py pub [iface]   # publica 300 mensajes rt/lowstate

Por defecto iface = "lo0" (interfaz loopback de macOS).
"""
import sys
import time

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_

DOMAIN = 1


def run_sub(iface):
    ChannelFactoryInitialize(DOMAIN, iface)
    seen = {"n": 0}

    def handler(msg):
        if seen["n"] == 0:
            print("RECIBIDO LowState -> motor0.q =", msg.motor_state[0].q, flush=True)
        seen["n"] += 1

    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(handler, 10)

    t0 = time.time()
    while time.time() - t0 < 8 and seen["n"] == 0:
        time.sleep(0.1)
    print("total recibidos:", seen["n"], flush=True)
    sys.exit(0 if seen["n"] > 0 else 2)


def run_pub(iface):
    ChannelFactoryInitialize(DOMAIN, iface)
    pub = ChannelPublisher("rt/lowstate", LowState_)
    pub.Init()
    msg = unitree_hg_msg_dds__LowState_()
    for _ in range(300):
        msg.motor_state[0].q = 0.42
        pub.Write(msg)
        time.sleep(0.01)
    print("publicados 300", flush=True)


if __name__ == "__main__":
    role = sys.argv[1] if len(sys.argv) > 1 else "sub"
    iface = sys.argv[2] if len(sys.argv) > 2 else "lo0"
    if role == "sub":
        run_sub(iface)
    else:
        run_pub(iface)
