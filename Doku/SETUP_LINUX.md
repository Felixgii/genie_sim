# Genie Sim auf nativem Linux

Anleitung zum Aufsetzen des Genie Sim RT Engine (`geniesim_ros`) mit dem AgiBot Genie G2
auf nativem Linux, ausgehend von Upstream-Commit `6ca11c7`. Stand 2026-10-16.
Der frühere WSL2-Stand liegt auf dem Branch `wsl-versuch` (Tag `wsl-ende`).

## Umgebung

| | |
|---|---|
| OS / Shell | CachyOS, Kernel 7.2.9, KDE Plasma (Wayland) / fish 4.9 |
| CPU / RAM | Intel Core i9-9900K / 23 GB |
| GPU | RTX 2080 Ti (11 GB), Treiber 615.78.08 |
| Docker | 29.9.1, NVIDIA Container Toolkit 1.20.1 |
| Host-venv | uv 0.13.1, Python 3.12 (`geniesim_cli` verlangt `<3.13`) |

| Gepinnt | |
|---|---|
| Assets | `agibot-world/GenieSimAssets` @ `a0813aad7c16165daffbe6c2737754e0344809ee` |
| Basis-Image | `nvcr.io/nvidia/isaac-sim:5.1.0@sha256:f3563cb2ba0c18af0b2fb321360dcb73a917b899f879e3213623d6bee484fa54` |
| isaacsim (pip, im Image) | 6.2.0.0 |
| newton | 1.6.1 |

**Aktiv ist Isaac Sim 6.2 (pip, System-Python 3.12), nicht die 5.1 des Basis-Images.**

## Host-Setup

```fish
sudo pacman -S --needed git-lfs nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker run --rm --gpus all nvidia/cuda:12.8.1-base-ubuntu24.04 nvidia-smi   # GPU im Container?

git config --global core.autocrlf false
git config --global core.eol lf
cd ~/Studium/BA/genie_sim
git lfs install; git lfs pull                     # vor dem ersten Szenenstart

uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -e source/geniesim_cli
ln -s ~/Studium/BA/genie_sim/.venv/bin/geniesim ~/.local/bin/geniesim
fish_add_path ~/.local/bin

git clone https://huggingface.co/datasets/agibot-world/GenieSimAssets assets   # ca. 28 GB
git -C assets checkout a0813aad7c16165daffbe6c2737754e0344809ee
printf '%s\n' '*.STL filter=lfs diff=lfs merge=lfs -text' '*.nurec filter=lfs diff=lfs merge=lfs -text' > assets/.git/info/attributes
git -C assets lfs pull; git -C assets lfs checkout
git -C assets lfs ls-files | awk '$2=="-"' | wc -l     # muss 0 sein
uv pip install --python .venv/bin/python -e assets      # bringt auch pxr mit
```

## Image und Container

```fish
geniesim docker build                        # lokal, ca. 40 min
cat $XAUTHORITY > ~/.Xauthority; chmod 600 ~/.Xauthority   # Wayland, einmal pro Login
geniesim docker up
geniesim docker into
```

Im Container:

```bash
vulkaninfo --summary | grep deviceName       # RTX 2080 Ti, nicht llvmpipe
cd /workspace && geniesim ros build dev      # nach frischem Clone und nach neuen Python-Dateien
source /workspace/devel/setup.bash
```

## Szenen starten

PhysX, Manipulationsszene:

```bash
ros2 launch genie_sim_bringup app.launch.py scene:=scene_pnp_g2_op \
  launcher_config:=launcher_ovrtx_isaac_physx headless:=false
```

Newton, fahrbare Basis (`navigation:=true` startet den Servo für `/cmd_twist`; einmal ENTER):

```bash
ros2 launch genie_sim_bringup app.launch.py scene:=scene_flat_g2_sp \
  launcher_config:=launcher_newton_mjwarp headless:=true navigation:=true
python3 /workspace/Doku/tools/drive_twist_test.py 0.2 4   # /odom muss sich bewegen
```

Kameras aus: `launcher_config:=/workspace/Doku/tools/launcher_ovrtx_isaac_physx_norender.yaml`.

## Health-Check

| `scene_pnp_g2_op` | Soll |
|---|---|
| Engine-Log | `[classify] body=5 arm=0 head=3 gripper=16 chassis_drive=4 chassis_steer=4` |
| `/joint_states` / `/tf` / `/tf_static` | 46 Joints / 49 Frames / 10 Frames |
| Isaac-Sim-Fenster | Roboter sichtbar |
| `robot.usda` | `.venv/bin/python Doku/tools/usd_robot_check.py assets/scenes/scene_pnp_g2_op/robot.usda` → Selection `physx`, 64 Meshes |

| `scene_flat_g2_sp` | Soll |
|---|---|
| Engine-Log | `joint map: 42 controllable joint(s)`, keine Warnung `joint_target_pos is None` |
| `drive_twist_test.py 0.2 4` | `/odom` x ≈ 0,78 m |

## Probleme

**`permission denied … docker.sock`**: User ist in der Gruppe `docker`, die Sitzung noch nicht.
Ab- und wieder anmelden.

**Asset-Dateien bleiben 133-Byte-LFS-Pointer** (`mujoco/robot/…/*.STL`, `*.nurec`): Die
`.gitattributes` des Asset-Repos deckt `*.STL` und `*.nurec` nicht ab.
Lokale Attribute wie oben setzen, dann `lfs pull` und `lfs checkout`.

**`docker pull` von `registry.agibot.com` hängt**: Die Registry ist nur im AgiBot-Netz
erreichbar. Image mit `geniesim docker build` lokal bauen.

**Kein Isaac-Sim-Fenster, X11-Fehler**: Unter Wayland fehlen `~/.Xauthority` und `xhost`.
Cookie wie oben kopieren, vor `geniesim docker up`.

**`Package 'genie_sim_bringup' not found`**: Workspace nicht gebaut oder nicht gesourct.
`geniesim ros build dev`, `source /workspace/devel/setup.bash`.

**Zweite Engine oder Reste nach Abbruch** (`/rtf` springt, Topics doppelt): Nach `kill -9`
bleiben `robot_state_publisher` / Servo übrig. Vor jedem Start prüfen, dass
`ps -eo cmd | grep -E 'ros2 launch|genie_sim_|robot_state|servo_node' | grep -v grep` leer ist.

**`simple_navigation.py … No module named 'pyclothoids'`** bei `navigation:=true`: Ist harmlos,
der Servo läuft trotzdem.

Durch Commits gelöst:

- Roboter ohne Bodies/Joints (Physics-Variante nicht gewählt): `da82f1d`
- Newton: `/cmd_twist` ohne Wirkung (`joint_target_pos is None`): `00c58ce`
- Paketversionen im Image driften (Newton 1.5 → 1.6.1): `33c1196`

## Messwerte

Headless, 60 s Aufwärmen, 25 s Messung (`Doku/tools/sim_bench.sh`).

| Pfad / Szene | | WSL2 | Linux |
|---|---|---|---|
| PhysX, `scene_pnp_g2_op` | RTF | – | 0,59 |
| | Schritt | – | 17,0 ms |
| | GPU-Last mit / ohne Kameras | – | 93 % / 6 % |
| Newton, Substep 10 | RTF | 0,64 | 1,00 |
| | GPU-Zeit pro Schritt | 15 ms | 6,2 ms |
| | GPU-Last / VRAM | – | 65 % / 1,2 GB |

Unter WSL lief der PhysX-Pfad nicht. Newton lief unter WSL mit `scene_pnp_g2_op` (Basis frei,
GDK-Umgebung) und Newton 1.5, unter Linux mit `scene_flat_g2_sp` und Newton 1.6.1.

## Hinweise

- Die Basis 5.1.0 liegt ungenutzt unter `/isaac-sim` (Python 3.11).
- Die Stage-Verzeichnisse unter `assets/scenes/<scene>/` gehören dem Host-User, ein `chown`
  ist nicht nötig.
- `newton`-Launcher funktionieren mit der Physics-Variante `physx`. `mujoco` ist nicht nötig.
- Leistung auf diesem Rechner: PhysX ist CPU-gebunden (ca. 57 Hz statt 100), Kameras liefern
  7–8 Hz. Newton erreicht 100 Hz.
