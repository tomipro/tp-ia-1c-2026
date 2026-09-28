#!/usr/bin/env python3
"""
Parches de compatibilidad para macOS (idempotentes).

1) unitree_sdk2_python: `timerfd` es exclusivo de Linux y rompe RecurrentThread.
   Se reemplaza esa parte por un bucle portable con time.sleep.
2) unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py: agrega un lock
   (SIM_LOCK) que serializa el acceso a mj_data entre los hilos DDS y el de
   física, evitando comandos corruptos.

Uso:  python patch_macos.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SDK = os.path.abspath(os.path.join(HERE, "..", "unitree_sdk2_python"))
MUJOCO_PY = os.path.abspath(os.path.join(HERE, "..", "unitree_mujoco", "simulate_python"))

TIMERFD = os.path.join(SDK, "unitree_sdk2py", "utils", "timerfd.py")
THREAD = os.path.join(SDK, "unitree_sdk2py", "utils", "thread.py")
BRIDGE = os.path.join(MUJOCO_PY, "unitree_sdk2py_bridge.py")

TIMERFD_OLD = '''# function timerfd_create
timerfd_create = CLIBLookup("timerfd_create", ctypes.c_int, (ctypes.c_long, ctypes.c_int))

# function timerfd_settime
timerfd_settime = CLIBLookup("timerfd_settime", ctypes.c_int, (ctypes.c_int, ctypes.c_int, ctypes.POINTER(itimerspec), ctypes.POINTER(itimerspec)))

# function timerfd_gettime
timerfd_gettime = CLIBLookup("timerfd_gettime", ctypes.c_int, (ctypes.c_int, ctypes.POINTER(itimerspec)))'''

TIMERFD_NEW = '''# timerfd_* son llamadas al sistema exclusivas de Linux. En macOS no existen,
# asi que se dejan en None y RecurrentThread usa un bucle portable (ver thread.py).
if sys.platform.startswith("linux"):
    timerfd_create = CLIBLookup("timerfd_create", ctypes.c_int, (ctypes.c_long, ctypes.c_int))
    timerfd_settime = CLIBLookup("timerfd_settime", ctypes.c_int, (ctypes.c_int, ctypes.c_int, ctypes.POINTER(itimerspec), ctypes.POINTER(itimerspec)))
    timerfd_gettime = CLIBLookup("timerfd_gettime", ctypes.c_int, (ctypes.c_int, ctypes.POINTER(itimerspec)))
else:
    timerfd_create = None
    timerfd_settime = None
    timerfd_gettime = None'''

THREAD_OLD = '''    def __LoopFunc(self):
        # clock type CLOCK_MONOTONIC = 1
        tfd = timerfd_create(1, 0)
        spec = itimerspec.from_seconds(self.__inter, self.__inter)
        timerfd_settime(tfd, 0, ctypes.byref(spec), None)

        while not self.__quit:
            try:
                self.__loopTarget(*self.__loopArgs, **self.__loopKwargs)
            except:
                info = sys.exc_info()
                print(f"[RecurrentThread] target func raise exception: name={info[0].__name__}, args={str(info[1].args)}")

            try:
                buf = os.read(tfd, 8)
                # print(struct.unpack("Q", buf)[0])
            except OSError as e:
                if e.errno != errno.EAGAIN:
                    raise e

        os.close(tfd)'''

THREAD_NEW = '''    def __LoopFunc(self):
        if sys.platform.startswith("linux"):
            self.__LoopFuncTimerfd()
        else:
            self.__LoopFuncSleep()

    def __LoopFuncTimerfd(self):
        # clock type CLOCK_MONOTONIC = 1
        tfd = timerfd_create(1, 0)
        spec = itimerspec.from_seconds(self.__inter, self.__inter)
        timerfd_settime(tfd, 0, ctypes.byref(spec), None)

        while not self.__quit:
            try:
                self.__loopTarget(*self.__loopArgs, **self.__loopKwargs)
            except:
                info = sys.exc_info()
                print(f"[RecurrentThread] target func raise exception: name={info[0].__name__}, args={str(info[1].args)}")

            try:
                buf = os.read(tfd, 8)
                # print(struct.unpack("Q", buf)[0])
            except OSError as e:
                if e.errno != errno.EAGAIN:
                    raise e

        os.close(tfd)

    def __LoopFuncSleep(self):
        # Fallback portable (macOS / Windows): mantiene el periodo con time.sleep.
        next_time = time.perf_counter()
        while not self.__quit:
            try:
                self.__loopTarget(*self.__loopArgs, **self.__loopKwargs)
            except:
                info = sys.exc_info()
                print(f"[RecurrentThread] target func raise exception: name={info[0].__name__}, args={str(info[1].args)}")

            next_time += self.__inter
            delay = next_time - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_time = time.perf_counter()'''

# --- Parches del bridge del simulador (unitree_mujoco) ---------------------
BRIDGE_IMPORT_OLD = '''import config
if config.ROBOT=="g1":'''

BRIDGE_IMPORT_NEW = '''import config

import threading
# Lock compartido: serializa el acceso a mj_data entre los hilos DDS y el de fisica.
SIM_LOCK = threading.Lock()

if config.ROBOT=="g1":'''

BRIDGE_LOWCMD_OLD = '''    def LowCmdHandler(self, msg: LowCmd_):
        if self.mj_data != None:'''

BRIDGE_LOWCMD_NEW = '''    def LowCmdHandler(self, msg: LowCmd_):
        with SIM_LOCK:
            self._LowCmdHandler(msg)

    def _LowCmdHandler(self, msg: LowCmd_):
        if self.mj_data != None:'''

BRIDGE_LOWSTATE_OLD = '''    def PublishLowState(self):
        if self.mj_data != None:'''

BRIDGE_LOWSTATE_NEW = '''    def PublishLowState(self):
        with SIM_LOCK:
            self._PublishLowState()

    def _PublishLowState(self):
        if self.mj_data != None:'''

BRIDGE_HIGHSTATE_OLD = '''    def PublishHighState(self):

        if self.mj_data != None:'''

BRIDGE_HIGHSTATE_NEW = '''    def PublishHighState(self):
        with SIM_LOCK:
            self._PublishHighState()

    def _PublishHighState(self):

        if self.mj_data != None:'''


def patch(path, old, new, marker, label):
    with open(path, "r") as f:
        text = f.read()
    if marker in text:
        print(f"[=] {label}: ya estaba parcheado.")
        return
    if old not in text:
        print(f"[!] {label}: no se encontró el bloque esperado. Revisar a mano.")
        return
    with open(path, "w") as f:
        f.write(text.replace(old, new))
    print(f"[+] {label}: parcheado.")


def main():
    if not os.path.isdir(SDK):
        print(f"No se encontró el SDK en {SDK}")
        sys.exit(1)

    # asegurar 'import sys' en timerfd.py
    with open(TIMERFD, "r") as f:
        t = f.read()
    if "import sys" not in t:
        t = t.replace("import math\nimport ctypes\n", "import math\nimport ctypes\nimport sys\n", 1)
        with open(TIMERFD, "w") as f:
            f.write(t)
        print("[+] timerfd.py: agregado 'import sys'.")

    # asegurar 'import time' en thread.py
    with open(THREAD, "r") as f:
        t = f.read()
    if "\nimport time\n" not in t:
        t = t.replace("import threading\n", "import threading\nimport time\n", 1)
        with open(THREAD, "w") as f:
            f.write(t)
        print("[+] thread.py: agregado 'import time'.")

    patch(TIMERFD, TIMERFD_OLD, TIMERFD_NEW, "sys.platform.startswith", "timerfd.py")
    patch(THREAD, THREAD_OLD, THREAD_NEW, "__LoopFuncSleep", "thread.py")

    # --- bridge del simulador ---
    if os.path.isfile(BRIDGE):
        patch(BRIDGE, BRIDGE_IMPORT_OLD, BRIDGE_IMPORT_NEW, "SIM_LOCK = threading.Lock()", "bridge import")
        patch(BRIDGE, BRIDGE_LOWCMD_OLD, BRIDGE_LOWCMD_NEW, "_LowCmdHandler", "bridge LowCmdHandler")
        patch(BRIDGE, BRIDGE_LOWSTATE_OLD, BRIDGE_LOWSTATE_NEW, "_PublishLowState", "bridge PublishLowState")
        patch(BRIDGE, BRIDGE_HIGHSTATE_OLD, BRIDGE_HIGHSTATE_NEW, "_PublishHighState", "bridge PublishHighState")
    else:
        print(f"[i] No se encontró el bridge en {BRIDGE} (¿clonaste unitree_mujoco?)")

    print("Listo.")


if __name__ == "__main__":
    main()
