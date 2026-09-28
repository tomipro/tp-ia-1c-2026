#!/usr/bin/env python3
"""
Simulador MuJoCo del Unitree G1 (basado en unitree_mujoco/simulate_python).

Abre la ventana de MuJoCo con el G1 y, opcionalmente, aplica un "arnés"
virtual tipo grúa que sujeta la pelvis: la fija en horizontal (x-y) y la
mantiene derecha, pero deja LIBRE la altura (z), de modo que el G1 se apoya
con los pies en el piso. Es el equivalente a un soporte de laboratorio que
evita que el humanoide se caiga mientras se prueban controladores de bajo
nivel (sin un controlador de equilibrio, el G1 se caería solo).

Uso:
    python g1_sim.py                 # abre el visor con arnés (pies en el piso)
    python g1_sim.py --no-harness    # sin arnés (se cae: el G1 necesita un
                                     # controlador de equilibrio o el sport mode)
    python g1_sim.py --headless 5    # sin ventana, corre 5 s y termina (para test)

En otra terminal, correr el controlador:
    python g1_control.py sim demo
"""
import os
import sys
import time
from threading import Thread, Lock

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.abspath(os.path.join(HERE, "..", "unitree_mujoco", "simulate_python"))
os.chdir(SIM_DIR)
sys.path.insert(0, SIM_DIR)

import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402

# Configuración del robot (no hace falta editar config.py del repo de Unitree)
import config  # noqa: E402
config.ROBOT = "g1"
config.ROBOT_SCENE = "../unitree_robots/g1/scene_29dof.xml"
config.INTERFACE = "lo0" if sys.platform == "darwin" else "lo"
config.DOMAIN_ID = 1
config.USE_JOYSTICK = 0
config.PRINT_SCENE_INFORMATION = False

from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: E402
try:
    from unitree_sdk2py_bridge import UnitreeSdk2Bridge, SIM_LOCK  # noqa: E402
except ImportError:  # bridge sin parchear: no expone SIM_LOCK
    from unitree_sdk2py_bridge import UnitreeSdk2Bridge  # noqa: E402
    SIM_LOCK = None

# --- Parámetros del arnés (tipo "grúa") ------------------------------------
# Sujeta la pelvis en x-y y la mantiene derecha con un torque suave, pero deja
# LIBRE la altura (z): el G1 se apoya en el piso con los pies y puede agacharse.
K_XY = 20000.0      # rigidez horizontal (fija la pelvis en x-y)
C_XY = 2000.0       # amortiguación horizontal
C_Z = 80.0          # amortiguación vertical (z libre)
K_ANG = 500.0       # rigidez para mantener el torso derecho (roll/pitch)
C_ANG = 40.0        # amortiguación angular (baja: evita inestabilidad)
SIM_DT = 0.002      # paso de integración del simulador [s]


def main():
    args = sys.argv[1:]
    headless = "--headless" in args
    use_harness = "--no-harness" not in args
    duration = 0.0
    for i, a in enumerate(args):
        if a == "--headless" and i + 1 < len(args):
            try:
                duration = float(args[i + 1])
            except ValueError:
                pass

    model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
    data = mujoco.MjData(model)
    model.opt.timestep = SIM_DT
    pelvis = model.body("pelvis").id
    foot = model.body("left_ankle_roll_link").id

    viewer = None
    if not headless:
        viewer = mujoco.viewer.launch_passive(model, data)

    time.sleep(0.2)

    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    bridge = UnitreeSdk2Bridge(model, data)
    if config.USE_JOYSTICK:
        bridge.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
    if config.PRINT_SCENE_INFORMATION:
        bridge.PrintSceneInformation()

    stop = {"flag": False}
    locker = SIM_LOCK if SIM_LOCK is not None else Lock()

    def physics():
        next_log = 0.0
        t_acc = 0.0
        while not stop["flag"]:
            step_start = time.perf_counter()
            with locker:
                if use_harness:
                    # Fuerza: fija la pelvis en x-y; z libre (solo amortiguación)
                    data.xfrc_applied[pelvis, 0] = -K_XY * data.qpos[0] - C_XY * data.qvel[0]
                    data.xfrc_applied[pelvis, 1] = -K_XY * data.qpos[1] - C_XY * data.qvel[1]
                    data.xfrc_applied[pelvis, 2] = -C_Z * data.qvel[2]
                    # Torque suave: mantiene el torso derecho (roll/pitch)
                    up = data.xmat[pelvis].reshape(3, 3)[:, 2]
                    err = np.cross(up, np.array([0.0, 0.0, 1.0]))
                    data.xfrc_applied[pelvis, 3:6] = K_ANG * err - C_ANG * data.qvel[3:6]
                mujoco.mj_step(model, data)
                t_acc += model.opt.timestep
                if headless and t_acc >= next_log:
                    print(f"t={t_acc:4.1f} pelvis_z={data.qpos[2]:.3f} pie_z={data.xpos[foot, 2]:.3f} "
                          f"Lknee={data.sensordata[3]:+.2f} Rshoulder={data.sensordata[22]:+.2f} "
                          f"Rwrist={data.sensordata[26]:+.2f} waistYaw={data.sensordata[12]:+.2f}")
                    next_log += 0.5
            rem = model.opt.timestep - (time.perf_counter() - step_start)
            if rem > 0:
                time.sleep(rem)

    def render():
        while not stop["flag"]:
            if viewer is not None:
                with locker:
                    viewer.sync()
            time.sleep(config.VIEWER_DT)

    t_phys = Thread(target=physics, daemon=True)
    t_rend = Thread(target=render, daemon=True)
    t_phys.start()
    t_rend.start()

    print(f"Simulador G1 corriendo (arnés={'ON' if use_harness else 'OFF'}).")
    if headless:
        time.sleep(duration)
        stop["flag"] = True
        t_phys.join(timeout=1.0)
        print(f"[headless] z final = {data.qpos[2]:.3f} m")
    else:
        while viewer.is_running():
            time.sleep(0.1)
        stop["flag"] = True


if __name__ == "__main__":
    main()
