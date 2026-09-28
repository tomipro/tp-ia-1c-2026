# TP — Simulación y control del Unitree G1 (MuJoCo + unitree_sdk2)

Trabajo práctico con el robot humanoide **Unitree G1** usando el simulador
MuJoCo de Unitree y su SDK de Python por DDS.

El objetivo de esta carpeta es tener un entorno **funcional y reproducible**
que sirve tanto para **simular** como para **controlar el robot real**, ya que
usa el mismo protocolo de comunicación (DDS) que el G1 físico.

---

## 1. Estado actual (lo que quedó andando)

- Simulador MuJoCo abriendo el **G1 de 29 DOF** en macOS (Apple Silicon).
- Controlador de bajo nivel en Python que envía comandos de motor por DDS
  (`rt/lowcmd`) y recibe el estado (`rt/lowstate`).
- Varias **demos**: parado, saludar, sentadillas, marcha en el lugar y giro
  de torso. También una demo combinada que recorre todo.
- Código **sim → real**: el mismo controlador funciona en el simulador y en
  el robot físico cambiando solo la interfaz de red.

> Nota: este repositorio contiene **nuestro código** (`tp_g1/` + `README.md` +
> `setup.sh`). Los repos de Unitree (`unitree_mujoco`, `unitree_sdk2`,
> `unitree_sdk2_python`) **no** se versionan acá: los clona `setup.sh` y aplica
> los parches necesarios.

---

## 2. Estructura

```
TP/
├── setup.sh                  # clona los repos de Unitree + crea el entorno + parches
├── README.md
├── tp_g1/                    # ★ nuestro código
│   ├── g1_sim.py             #   lanzador del simulador (G1 + "arnés")
│   ├── g1_control.py         #   controlador de bajo nivel + demos
│   ├── run_sim.sh            #   atajo para abrir el simulador
│   ├── run_control.sh        #   atajo para enviar comandos
│   ├── patch_macos.py        #   parches de compatibilidad (SDK + bridge)
│   ├── dds_echo.py           #   diagnóstico de DDS
│   └── poses/                #   imágenes de referencia
├── unitree_mujoco/           # (lo clona setup.sh) simulador MuJoCo
├── unitree_sdk2/             # (lo clona setup.sh) SDK C++
├── unitree_sdk2_python/      # (lo clona setup.sh) SDK Python
└── venv310/                  # (lo crea setup.sh) entorno virtual
```

---

## 3. Requisitos / entorno

- **macOS** (probado en Apple Silicon, arm64) o Linux.
- **Python 3.10 arm64** (por las wheels de `cyclonedds==0.10.2`).
  En esta máquina se usó `/Library/Frameworks/Python.framework/Versions/3.10`.
- Paquetes del entorno virtual (`venv310`): `cyclonedds==0.10.2`, `mujoco`,
  `numpy`, `opencv-python`, `pygame` y `unitree_sdk2py` (instalado en modo
  editable desde `unitree_sdk2_python`).

Preparar el entorno (clona los repos de Unitree, crea el venv e instala todo):

```bash
cd TP
bash setup.sh
```

Requiere **Python 3.10** (por las wheels de `cyclonedds==0.10.2`). Si no está en
el PATH, pasá la ruta: `PY310=/ruta/a/python3.10 bash setup.sh`.

---

## 4. Cómo ejecutar (dos terminales)

**Terminal 1 — simulador:**

```bash
cd TP
bash tp_g1/run_sim.sh
```

Se abre la ventana de MuJoCo con el G1.

**Terminal 2 — controlador:**

```bash
cd TP
bash tp_g1/run_control.sh            # demo completa (recomendado)
bash tp_g1/run_control.sh wave       # solo saludar
bash tp_g1/run_control.sh squat      # sentadillas
bash tp_g1/run_control.sh march      # marcha en el lugar
bash tp_g1/run_control.sh twist      # giro de torso
bash tp_g1/run_control.sh stand      # mantener parado
```

Cortar la terminal 2 con `Ctrl+C` (el robot queda "suelto" y se desploma
por gravedad, es normal).

Demos disponibles: `demo | stand | wave | squat | march | twist`.

> Los gestos **locales** opcionales (archivo `tp_g1/local_extras.py`, que no se
> versiona) se agregan automáticamente a la lista de demos.

---

## 5. Notas específicas de macOS

Tres cosas se ajustaron para que funcione en macOS (todas resueltas):

1. **`mjpython` para el visor.** En macOS, el visor de MuJoCo
   (`launch_passive`) solo funciona lanzando el script con `mjpython`.
   Lo automatizamos en `run_sim.sh`.
2. **Parche de `timerfd`.** `unitree_sdk2py` usa `timerfd` (exclusivo de
   Linux) en `RecurrentThread`. `patch_macos.py` reemplaza esa parte por un
   bucle portable con `time.sleep`, manteniendo el comportamiento en Linux.
   (Ya está aplicado en este clon.)
3. **Interfaz de red `lo0`.** El simulador usa la interfaz de loopback; en
   macOS se llama `lo0` (en Linux sería `lo`). `g1_sim.py` lo configura solo
   (no hace falta editar `config.py` de Unitree).

---

## 5-bis. Cambios aplicados a los repos clonados

Para que todo funcione se hicieron estos cambios (además de nuestro código en
`tp_g1/`):

| Archivo | Cambio |
|---|---|
| `unitree_sdk2_python/.../utils/timerfd.py` | `timerfd` solo en Linux; en macOS queda en `None`. |
| `unitree_sdk2_python/.../utils/thread.py` | `RecurrentThread` usa un bucle portable con `time.sleep` fuera de Linux. |
| `unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py` | Se agregó un `SIM_LOCK` que serializa el acceso a `mj_data` entre los hilos DDS y el de física (evita comandos corruptos por concurrencia). |

En cambio, `g1_sim.py` configura el robot (G1, escena 29 DOF, `lo0`, sin
joystick) **en runtime**, así que no hace falta tocar `config.py` de Unitree.

Todos los parches se re-aplican con `python tp_g1/patch_macos.py` (es
idempotente).

> Nota: de forma **intermitente** (cada varios ciclos) y más frecuente si la
> computadora está muy cargada, puede aparecer un "flojeo" momentáneo del
> robot. Es un problema conocido de la capa DDS/comunicación en esta
> configuración; si pasa, volver a lanzar el controlador. En máquinas
> tranquilas la demo corre estable.

---

## 6. Arquitectura y protocolo

```
   ┌────────────────────┐        DDS (CycloneDDS)        ┌──────────────────────┐
   │  g1_sim.py         │  rt/lowstate  (estado) ───────▶ │  g1_control.py       │
   │  MuJoCo + G1       │                                 │  controlador PD      │
   │  + bridge SDK2     │  ◀───────  rt/lowcmd (comando)   │  200 Hz              │
   └────────────────────┘                                 └──────────────────────┘
```

- **Robot:** G1 29 DOF → usa los mensajes IDL `unitree_hg` (el Go2/H1 usan
  `unitree_go`).
- **Tópicos:** `rt/lowcmd` (comandos de motor), `rt/lowstate` (estado),
  `rt/sportmodestate` (posición/velocidad, solo sim), `rt/wirelesscontroller`.
- **Comando de motor:** cada junta recibe `q` (posición), `dq` (velocidad),
  `kp`, `kd`, `tau`. El bridge del simulador traduce a torque:
  `tau_total = tau + kp·(q−q_med) + kd·(dq−dq_med)`.
- **Orden de las juntas (0..28):** pierna izq, pierna der, cintura, brazo izq,
  brazo der (referencia completa en
  `unitree_mujoco/unitree_robots/g1/g1_joint_index_dds.md`).
- **`mode_pr` / `mode_machine`:** el G1 usa control serie (PR) para tobillos;
  el simulador conserva el `mode_machine` para poder pasar el mismo código al
  robot real.

---

## 7. ¿Por qué el G1 se cae si no le damos soporte?

Un humanoide parado es un **péndulo invertido**: con las juntas en PD "blando"
(el ejemplo oficial de Unitree usa `kp≈40` en tobillos) el robot se vuelca,
porque hace falta una rigidez de tobillo mayor que `m·g·h` (≈200 N·m/rad) o,
mejor, un **controlador de equilibrio**. El G1 real no tiene este problema
porque su servicio de locomoción (`sport_mode`) se encarga del balance.

Para poder practicar control de bajo nivel sin pelear con el balance, el
simulador aplica un **"arnés" virtual tipo grúa** sobre la pelvis:
- la fija en horizontal (x-y) para que no se caiga ni se vaya de la base,
- la mantiene derecha (un torque suave corrige roll/pitch),
- pero deja **libre la altura (z)**, así el G1 **se apoya con los pies en el
  piso** y hasta puede agacharse (las rodillas se flexionan y los pies quedan
  plantados).

Es el equivalente a un soporte de laboratorio o a la *banda elástica* que trae
el repo original (`ENABLE_ELASTIC_BAND`). Se puede desactivar:

```bash
bash tp_g1/run_sim.sh --no-harness   # sin soporte (se cae: necesita balance)
```

Parámetros del arnés en `tp_g1/g1_sim.py` (`K_XY`, `C_ANG`, `K_ANG`, `SIM_DT`).

---

## 8. Pasar del simulador al robot real (sim → real)

El controlador ya está preparado. Solo cambia la red:

```bash
# Simulador: dominio DDS 1, interfaz lo0
bash tp_g1/run_control.sh sim demo

# Robot real: dominio DDS 0, interfaz de red conectada al robot (ej. en0)
bash tp_g1/run_control.sh real en0 demo
```

En el robot real, `g1_control.py` intenta **liberar el servicio de
locomoción** automáticamente (`MotionSwitcherClient`) porque el G1 solo acepta
comandos de bajo nivel con el `sport_mode` apagado.

> ⚠️ Seguridad: antes de mover el robot real, verificar que esté suspendido o
> en un espacio despejado, y probar primero con amplitudes chicas.

---

## 9. Uso del simulador oficial (sin nuestro lanzador)

También se puede correr el script original de Unitree:

```bash
cd TP/unitree_mujoco/simulate_python
../../venv310/bin/python -u ../../venv310/bin/mjpython unitree_mujoco.py
```

Su configuración está en `config.py`. Para correrlo con el G1 hay que dejarlo
así: `ROBOT="g1"`, `ROBOT_SCENE="../unitree_robots/g1/scene_29dof.xml"`,
`INTERFACE="lo0"` (macOS) y `USE_JOYSTICK=0`. Nuestro `g1_sim.py` ya lo hace
automáticamente, así que normalmente no hace falta.

El simulador **C++** (`simulate/`) y el ejemplo ROS2 no se usan acá porque están
pensados para Linux/Ubuntu.

---

## 10. Diagnóstico rápido

- **El visor no abre** → asegurarse de usar `run_sim.sh` (usa `mjpython`).
- **El controlador no hace nada** → verificar que el simulador esté corriendo y
  que ambos usen el mismo dominio/interfaz (`g1_sim.py` usa dominio 1 / `lo0`).
- **`dlsym ... timerfd_create`** → falta aplicar `patch_macos.py`.
- **`Could not locate cyclonedds`** → instalar primero la wheel
  `cyclonedds==0.10.2` con Python 3.10.
- **Probar la comunicación DDS sola**:
  ```bash
  ./venv310/bin/python tp_g1/dds_echo.py sub lo0    # en una terminal
  ./venv310/bin/python tp_g1/dds_echo.py pub lo0    # en otra
  ```

---

## 11. Ideas para ampliar el TP

- Definir **posturas nuevas** (editando `pose_for` en `g1_control.py`).
- Leer el **IMU** (`low_state.imu_state`) y hacer control realimentado.
- Implementar un **controlador de equilibrio** simple (LQR sobre el modelo
  linealizado, o control de CoM/ZMP) para que camine sin arnés.
- Usar los ejemplos de **alto nivel** del SDK (`unitree_sdk2py/g1/loco`) contra
  el robot real.
- Integrar **cámara** o **visión** (el SDK trae ejemplos con OpenCV).
- Pasar todo a **ROS2** con `unitree_ros2` (requiere Linux).

---

## 12. Referencias

- unitree_mujoco: https://github.com/unitreerobotics/unitree_mujoco
- unitree_sdk2: https://github.com/unitreerobotics/unitree_sdk2
- unitree_sdk2_python: https://github.com/unitreerobotics/unitree_sdk2_python
- Documentación Unitree: https://support.unitree.com/home/en/developer
