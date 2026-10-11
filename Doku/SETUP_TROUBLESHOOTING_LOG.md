# Setup-Troubleshooting-Log

**Datum:** 2026-10-11
**Umgebung:** Windows 11 Pro · WSL2 (Ubuntu 24.04) · Docker Desktop · NVIDIA RTX 2080 Ti (Treiber 617.42, CUDA 13.4) · 24 GB RAM
**Projekt-Root in WSL:** `~/genie_sim` (Container-Mount `/workspace`)
**Container:** `geniesim3` (Image `geniesim3:latest`, Isaac Sim 5.1/6.x + ROS 2 Jazzy)

Dieses Log dokumentiert eine Troubleshooting-Session nach einem Windows-Neustart.
Ziel war, die Szene `scene_pnp_g2_op` mit GUI zu starten. Am Ende lief die Simulation
mit der Newton-Physik-Engine und einem GPU-beschleunigten OpenGL-Fenster.
Der Standard-Pfad mit Isaac Sim und RTX-Rendering ist auf diesem Setup grundsätzlich
nicht lauffähig (siehe Problem 2).

---

## Textbaustein für den Bericht

> Der folgende Abschnitt ist als Fließtext formuliert und kann (ggf. gekürzt) in die
> Arbeit übernommen werden.

Die Simulationsumgebung GenieSim wurde unter Windows 11 in einem Docker-Container
betrieben, der über Docker Desktop auf dem WSL2-Subsystem läuft. Bei der Inbetriebnahme
traten mehrere voneinander unabhängige Probleme auf, die sich teilweise gegenseitig
verdeckten.

Zunächst war das ROS-2-Overlay-Workspace in neuen Container-Shells nicht eingebunden,
sodass ROS die projekteigenen Pakete nicht fand. Dies ließ sich durch das Sourcen von
`devel/setup.bash` beheben. Ein Neubau des Workspaces war nicht nötig, da die
Build-Artefakte auf dem Host liegen und Container-Neustarts überdauern.

Das zentrale Hindernis war die Grafikschnittstelle. Isaac Sim und dessen RTX-Renderer
setzen Vulkan voraus. Unter WSL2 stellt der NVIDIA-Treiber Linux-Anwendungen jedoch nur
CUDA, OptiX und eine D3D12-Schnittstelle bereit, aber keinen nativen Vulkan-Treiber.
Docker Desktop reicht zudem nur den CUDA-Teil des Treibers in den Container durch.
Diagnostisch zeigte sich das daran, dass `nvidia-smi` und CUDA funktionierten, `vulkaninfo`
aber keinen Treiber fand. NVIDIA bestätigt, dass Isaac-Sim-Container unter WSL2 nicht
unterstützt werden. Der RTX-basierte Standard-Pfad (`launcher_ovrtx_isaac_physx`) ist auf
dieser Plattform daher nicht lauffähig. Als Ausweg wurde der Kit-freie Pfad mit der
GPU-Physik-Engine Newton (MuJoCo-Warp-Solver) gewählt, die ausschließlich CUDA benötigt
und ein OpenGL-Fenster zur Visualisierung bietet.

Weiterhin enthielt die WSL-Arbeitskopie des Repositories Windows-Zeilenenden (CRLF) in
über 5000 Textdateien, vermutlich weil sie aus einer Windows-Arbeitskopie kopiert statt
geklont worden war. Dadurch scheiterten Python-Skripte bereits am Shebang
(`/usr/bin/env: 'python3\r'`). Die Dateien wurden nach einem Abgleich mit dem
Git-Repository nach LF konvertiert.

Nachdem die Simulation lief, zeigte sich, dass der Roboter keine steuerbaren Gelenke
besaß. Ursache war eine fehlende Varianten-Auswahl in der aus dem URDF erzeugten
USD-Datei: `robot.usda` definiert ein VariantSet `Physics` mit den Varianten `physx`,
`physics`, `mujoco` und `none`, wählte aber keine aus. Ohne Auswahl wird keine
Physics-Payload eingebunden, sodass der Roboter nur als Geometrie ohne Gelenke, Massen und
Antriebe existiert. Das Konvertierungsskript `assemble_robot.py` wurde so erweitert, dass es
die in der Projektdokumentation vorgesehene Default-Variante `physx` setzt. Danach verfügte
das Modell über 46 steuerbare Gelenke mit PD-Antrieben.

Schließlich lief die Visualisierung mit nur 1–3 Bildern pro Sekunde, obwohl weder CPU noch
GPU ausgelastet erschienen. Das OpenGL-Fenster wurde per Software-Rasterizer (Mesa
`llvmpipe`) auf einem einzelnen CPU-Kern gerendert, weil die WSL-Grafikbibliotheken im
Container fehlten. Durch das Einbinden der WSL-GPU-Bibliotheken und die Auswahl des
Mesa-Treibers `d3d12` rendert OpenGL nun über DirectX 12 auf der GPU. Die Schleifenrate
stieg dadurch auf etwa 17 Hz. Der Physik-Schritt selbst benötigt dabei nur rund 0,7 ms,
das Rendern ca. 54 ms und ist damit weiterhin der begrenzende Faktor.

Für die Reproduzierbarkeit ist festzuhalten, dass der Betrieb unter WSL2 und Docker
Desktop von NVIDIA nicht offiziell unterstützt wird und Treiber- oder Docker-Updates das
Setup erneut beeinträchtigen können. Für RTX-Rendering mit Isaac Sim ist ein natives
Linux-System erforderlich.

---

## Problem 1 — `Package 'genie_sim_bringup' not found`

**Symptom:**
```
Package 'genie_sim_bringup' not found ... searching: ['/opt/ros/jazzy']
```

**Ursache:** Der Entrypoint ([docker/entrypoint.sh](../docker/entrypoint.sh)) sourct nur
`/opt/ros/jazzy/setup.bash`, nicht das Projekt-Overlay. Das gebaute Paket liegt unter
`/workspace/devel/share/genie_sim_bringup/`.

**Fix:** In jeder neuen Container-Shell:
```bash
source /workspace/devel/setup.bash
```

**Erkenntnis:** `geniesim ros build dev` ist nur bei Source-Änderungen nötig. `devel/`
liegt auf dem Host und überdauert Container-Neustarts. Die Python-Skripte sind per Symlink
installiert (`devel/lib/genie_sim_engine/*.py → source/...`), Änderungen an ihnen
wirken also ohne Rebuild.

---

## Problem 2 — Vulkan `ERROR_INCOMPATIBLE_DRIVER` (RTX-Pfad nicht lauffähig)

**Symptom:**
```
[omni.rtx] VkResult: ERROR_INCOMPATIBLE_DRIVER
[omni.rtx] vkCreateInstance failed. Vulkan 1.1 is not supported, or your driver requires an update.
[ovrtx] Failed to setup graphics!
```

**Diagnose:**
```bash
nvidia-smi                 # OK (KMD 617.42, CUDA UMD 13.4)
vulkaninfo --summary       # ERROR: libGLX_nvidia.so.0: cannot open shared object file
cat /etc/vulkan/icd.d/nvidia_icd.json   # zeigt auf libGLX_nvidia.so.0
ls /usr/lib/wsl/lib        # im WSL-Host: 23 Dateien, darunter KEIN libGLX_nvidia / Vulkan
```

**Ursache:** Der NVIDIA-Treiber für WSL liefert für Linux nur CUDA, OptiX, NVENC/NVDEC
und die D3D12-Schnittstelle (`libd3d12.so`, `libdxcore.so`), aber **keinen nativen
Vulkan-Treiber**. Das ICD-JSON im Image erwartet `libGLX_nvidia.so.0`, das es unter WSL
nicht gibt. Isaac Sim (Kit) und der ovrtx-Renderer brauchen Vulkan mit RTX-Erweiterungen.

> **Korrektur:** Eine frühere Version dieses Logs nahm an, das Problem sei durch
> `wsl --shutdown` + Docker-Neustart behebbar bzw. durch einen Treiber-Downgrade.
> Beides ist falsch. Nach dem Neustart war `vulkaninfo` unverändert fehlerhaft. Die
> genannten "funktionierenden" Treiberversionen waren nicht belegt.

**Konsequenz:** `launcher_ovrtx_isaac_physx` (und alles mit Kit-GUI oder ovrtx) läuft
unter WSL2 + Docker Desktop nicht. NVIDIA bestätigt das:
- [NVIDIA Dev Forum: Isaac Sim in WSL2](https://forums.developer.nvidia.com/t/is-it-possible-to-run-isaac-sim-in-wsl2/349609):
  *"Isaac Sim containers are currently not supported under WSL2."*
- [NVIDIA Container Toolkit Issue #233](https://github.com/NVIDIA/nvidia-container-toolkit/issues/233):
  Vulkan in Docker Desktop nicht unterstützt.

**Workaround:** Kit-freier Newton-Pfad (nur CUDA) mit OpenGL-Viewer:
```bash
ros2 launch genie_sim_bringup app.launch.py \
  scene:=scene_pnp_g2_op \
  launcher_config:=launcher_newton_mjwarp \
  headless:=false \
  physics_engine_visualizer:=newton
```
Der Launcher fragt einmal mit "Press [ENTER] to continue" nach (experimentelles Backend).

**Nebenbefund:** `assemble_robot.py` startet trotzdem eine headless Kit-App. Die Vulkan-Fehler
dort sind unkritisch, die URDF→USD-Konvertierung funktioniert ohne Renderer.

---

## Problem 3 — CRLF-Zeilenenden im WSL-Clone

**Symptom:**
```
/usr/bin/env: 'python3\r': No such file or directory
[ERROR] [genie_sim_engine_isaacsim.py-4]: process has died [... exit code 127 ...]
```

**Diagnose:**
- `devel/lib/genie_sim_engine/genie_sim_engine_isaacsim.py` ist ein **Symlink** auf die
  Source. Die Source-Datei selbst hatte CRLF. Ein ROS-Rebuild hätte daher nichts geändert.
- `~/genie_sim/.git` ist unvollständig (`git status` → "not a git repository"). Die
  WSL-Arbeitskopie wurde offenbar kopiert statt geklont.
- 5285 Textdateien hatten CRLF (davon 3816 in `source/geniesim_benchmark`).
- Abgleich mit dem Git-Repo auf `D:\Studium\BA\genie_sim` (`diff --strip-trailing-cr`):
  inhaltlich keine Unterschiede, nur Zeilenenden.

> **Korrektur:** Der früher hier empfohlene Fix `git rm --cached -r . && git reset --hard`
> funktioniert in dieser Arbeitskopie nicht (kein gültiges `.git`) und hätte sonst lokale
> Änderungen verwerfen können.

**Fix (durchgeführt):** Backup + Konvertierung aller betroffenen Dateien (ohne `.bat/.cmd/.ps1`):
```bash
tar -czf ~/genie_sim_crlf_backup_2026-10-11.tar.gz -T /tmp/crlf_convert.txt   # 11 MB
while IFS= read -r f; do sed -i 's/\r$//' "$f"; done < /tmp/crlf_convert.txt
```
Ergebnis: 0 verbleibende CRLF-Dateien.

**Empfehlung:** Die WSL-Arbeitskopie langfristig durch einen echten `git clone` mit
`core.autocrlf=false` ersetzen (siehe [SETUP.md](../SETUP.md) Abschnitt 1.2).

---

## Problem 4 — DDS-Warnings (unkritisch)

```
tev: ddsi_udp_conn_write to udp/127.0.0.1:7456 failed with retcode -3
```
CycloneDDS sendet Discovery-Pakete an Loopback-Ports ohne Empfänger. Der Container ist per
`CYCLONEDDS_URI` auf `lo` beschränkt ([docker/entrypoint.sh](../docker/entrypoint.sh)).
Harmlos, kein Fix nötig.

---

## Problem 5 — Kein GUI-Fenster

**Ursache:** Folge von Problem 2. Das Kit-/Isaac-Sim-Fenster braucht Vulkan. Mit dem
Newton-Launcher und `physics_engine_visualizer:=newton` öffnet sich ein OpenGL-Fenster
über WSLg (X11-Socket `/tmp/.X11-unix/X0`, `DISPLAY=:0`). **Gelöst** über den Workaround
aus Problem 2.

Beim ersten Start kompiliert Warp die CUDA-Kernel (mehrere Minuten, Cache unter
`~/.cache/warp`). Danach geht der Start deutlich schneller.

---

## Problem 6 — Fehlende Physics-Variant-Auswahl im Roboter-USD

**Hinweis eines Kollegen (2026-10-07):** "no physics variant selection".

**Symptom (Newton-Log):**
```
[newton-standalone] joint map: 0 controllable joint(s) from 14 total Newton joints
[newton-standalone] model — 14 bodies, 84 DOFs
```
Der Roboter war im Fenster sichtbar, hatte aber keine Gelenke. Die 14 Bodies waren nur
die freien Szenen-Objekte.

**Diagnose:** `/geniesim_assets/scenes/scene_pnp_g2_op/robot.usda` (38 Zeilen):
```usda
def Xform "robot" (
    prepend references = @./payloads/base.usda@
    append variantSets = "Physics"
)
{
    variantSet "Physics" = { "mujoco" (...) "none" "physics" (...) "physx" (...) }
}
```
Es fehlt die Zeile `variants = { string Physics = "physx" }`, die laut
[asset_consumption.md:25-36](../source/geniesim_ros/src/ros_ws/src/genie_sim_engine/docs/asset_consumption.md#L25-L36)
die Default-Auswahl setzt. Ohne Auswahl wird keine `payloads/Physics/*.usda` eingebunden.
Damit fehlen alle Joints (46 RevoluteJoints in `physics.usda`), Massen und Antriebe.

Folgefehler: Die Nachbearbeitung in `assemble_robot.py` (Collision-Policy,
Mimic-Joint-Overlay) öffnet `robot.usda` ebenfalls ohne Variante, findet keine
Physics-Prims und schreibt deshalb nichts.

Ein Neuerzeugen (Löschen von `urdf.sha256`) ergab wieder dieselbe Datei. Der
Asset-Transformer in diesem Image setzt die Auswahl also nie.

**Fix (Code-Änderung):** [assemble_robot.py](../source/geniesim_ros/src/ros_ws/src/genie_sim_engine/scripts/assemble_robot.py)
um `_ensure_default_physics_variant()` erweitert. Die Funktion setzt
`Physics = "physx"` auf dem Root-Prim, falls keine Auswahl existiert, und zwar direkt nach
dem Transformer, vor der Nachbearbeitung. Danach `urdf.sha256` gelöscht, um die
Neuerzeugung auszulösen.

**Ergebnis:**
```
[assemble_robot] authored default variant selection Physics='physx' in .../robot.usda
[assemble_robot] [collision] 'arm_l_link1': disabled 2/2 collider(s) (non-contact link)   ← Nachbearbeitung läuft jetzt
[newton-standalone] joint map: 46 controllable joint(s) from 70 total Newton joints
[newton-standalone] PD drives: 46/46 controllable DOF(s) have POSITION-mode + ke>0 + kd>0
[newton-standalone] model — 70 bodies, 130 DOFs
```
`robot.usda` hat jetzt 666 Zeilen.

**Nebenbefund:** `assemble_robot.py` schreibt den Erfolgsstempel `urdf.sha256` in einem
`finally`-Block und beendet sich mit `os._exit(0)`. Eine Exception in der Nachbearbeitung
würde also als Erfolg gewertet und das unvollständige Ergebnis gecacht.

---

## Problem 7 — 1–3 FPS, CPU und GPU kaum ausgelastet

**Diagnose:** Renderer-Abfrage per pyglet im Container:
```
RENDERER: llvmpipe (LLVM 20.1.2, 256 bits)       ← Software-Rendering auf der CPU
```
Docker Desktop injiziert nur 8 CUDA-Dateien des Treibers in den Container. Mesa braucht für
GPU-OpenGL unter WSL zusätzlich (ermittelt per `LD_DEBUG=files`):
- `/usr/lib/wsl/lib/libd3d12.so`, `libd3d12core.so`, `libdxcore.so`
- `/usr/lib/wsl/drivers/nv_dispi.inf_amd64_*/libnvwgf2umx.so` (D3D12-User-Mode-Treiber)

Der Mesa-Treiber `d3d12_dri.so` ist im Image bereits vorhanden. llvmpipe läuft
größtenteils auf einem Kern, deshalb wirkt die CPU-Gesamtlast niedrig.

**Fix:**
1. [docker/start.sh](../docker/start.sh): Im GUI-Modus unter WSL zusätzlich
   `--device /dev/dxg`, `-v /usr/lib/wsl/lib:/usr/lib/wsl/lib:ro`,
   `-v /usr/lib/wsl/drivers:/usr/lib/wsl/drivers:ro`.
2. [docker/entrypoint.sh](../docker/entrypoint.sh): Wenn `/dev/dxg` und `libd3d12.so`
   vorhanden sind, `GALLIUM_DRIVER=d3d12` setzen und `/usr/lib/wsl/lib` an
   `LD_LIBRARY_PATH` anhängen.
3. Für den laufenden Container wurden die Dateien einmalig per `docker cp` hineinkopiert.
   Die Mounts aus Punkt 1 greifen erst bei `geniesim docker down && geniesim docker up`.

**Ergebnis:**
```
RENDERER: D3D12 (NVIDIA GeForce RTX 2080 Ti)
physics= 17.1Hz (target 100)  viewport= 17.1Hz (target 30)
Step:   mean=54.41 ms   Solver: mean=0.66 ms
```
Von 1–3 FPS auf ca. 17 Hz. Der Engpass ist weiterhin das Rendern (~54 ms pro Frame),
die Physik braucht nur ~0,7 ms. Weil jeder Tick ein Render-Tick ist, ist die Physikrate an
die Bildrate gekoppelt. Ein niedrigeres `render_hz` würde mehr Physik-Schritte zwischen den
Frames erlauben (nicht getestet).

**RAM:** WSL bekommt standardmäßig 50 % des Host-RAMs (hier 11 GiB von 24 GB). Davon
waren nur ~1 GiB von Prozessen belegt, ~8,6 GiB waren Linux-Page-Cache. Windows zeigt
diesen Cache als von `VmmemWSL` belegt an. Er ist kein Engpass für die Simulation.

---

## Problem 8 — Roboter reagiert auf keine Befehle (Newton-1.5-API)

**Symptom:** Befehle auf `/cmd_4ws` (Räder) bewegen nichts. Radgeschwindigkeit und Odometrie
bleiben bei 0, obwohl QoS (BEST_EFFORT) und Topic-Typ stimmen. Im Log:
```
WARN: _apply_init_joint_pos: Control.joint_target_pos was removed in Newton 1.5; use Control.joint_target_q instead.
WARN: [mjwarp-adapter] control.joint_target_pos is None; position commands will be silently dropped
```

**Ursache:** Das Image enthält Newton 1.5.0. Dort wurden `Control.joint_target_pos` und
`joint_target_vel` in `joint_target_q` und `joint_target_qd` umbenannt. Die alten Namen sind
durch einen `RemovedAttribute`-Deskriptor ersetzt, der beim Lesen einen `AttributeError`
wirft. Die Engine liest sie per `getattr(control, "joint_target_pos", None)` und bekommt
dadurch stillschweigend `None`. `apply_commands` bricht dann ab
([control.py:104-106](../source/geniesim_ros/src/ros_ws/src/genie_sim_engine/scripts/engine/newton/control.py#L104-L106)).
Damit wurden **alle** Befehle verworfen (`/joint_command` für Arme und `/cmd_4ws` für
Räder), ebenso die Start-Pose der Arme.

**Fix (Code-Änderung):** Neues Modul
[engine/newton/_compat.py](../source/geniesim_ros/src/ros_ws/src/genie_sim_engine/scripts/engine/newton/_compat.py),
aufgerufen in `engine/newton/__init__.py`. Es ersetzt die beiden `RemovedAttribute`-Deskriptoren
durch Properties, die auf die neuen Namen zeigen. Das ist bedeutungsgleich, solange
`newton.use_coord_layout_targets = False` (Default): Dann sind die neuen Buffer weiterhin
DOF-indiziert wie vorher. Bei anderen Newton-Versionen ist das Modul ein No-op.
In `devel/` wurde der Symlink für die neue Datei von Hand angelegt (macht sonst der nächste
`geniesim ros build dev`).

Alternative wäre, die ca. 40 Verwendungen im Code umzubenennen. Der Alias ist der
kleinere Eingriff und bleibt mit älteren Newton-Versionen kompatibel.

**Nicht behoben:** `RuntimeWarning: MuJoCo dependency version mismatch ... mujoco==3.8.1
(requires ~=3.11.0)`. Bisher ohne beobachtete Auswirkung.

---

## Erweiterung — Roboter fahren lassen (vor / zurück)

**Änderung:** In
[scene_pnp_g2_op.yaml](../source/geniesim_ros/src/ros_ws/src/genie_sim_bringup/config/scene_pnp_g2_op.yaml)
`pin_base_to_world: true` → `false`. Mit `true` ist die Basis per `root_joint` an der Welt
fixiert (Substep-Modus "kinematic-control", ohne Schwerkraft und Kontakte). Mit `false`
läuft die Basis unter echter Physik ("plain: real gravity + shape contacts"). Die anderen
G2-Szenen (`scene_wbc_g2_sp`, `scene_flat_g2_sp`) nutzen bereits `false`. Backup der
Original-YAML: `~/genie_sim/scene_pnp_g2_op.yaml.bak_2026-10-11`.

**Ansteuerung:** `/cmd_4ws` (`sensor_msgs/JointState`, QoS BEST_EFFORT):
- Lenk-Joints `idx1{1,2,3,4}1_chassis_*wheel_*_joint1` → `position` [rad]
- Antriebs-Joints `idx1{1,2,3,4}2_chassis_*wheel_*_joint2` → `velocity` [rad/s]
- Bleibt der Topic länger als `cmd_4ws_timeout_s` aus, bremst die Engine die Räder.
  Es muss also kontinuierlich gesendet werden.

**Test (Räder 6 s mit +3 rad/s, dann 6 s mit −3 rad/s):**
```
after forward:   wheel vel=2.99   odom x: -0.200 → 0.021
after backward:  wheel vel=-2.99  odom x: 0.021 → -0.177
```
Die Strecke ist kurz, weil die Simulation nur mit ~17 Hz statt 100 Hz läuft (≈ 17 %
Echtzeit): 6 s Wanduhrzeit sind etwa 1 s Simulationszeit.

**Tastatur-Teleop:** [Doku/drive_base.py](drive_base.py)
(`w` vor, `s` zurück, Leertaste stopp, `+`/`-` Tempo, `q` beenden).

Der vorgesehene Weg über `/cmd_twist` → `genie_sim_controllers` (MPC-Servo) wurde nicht
genutzt: `servo_4ws.launch.py` verweist auf eine Config `servo_4ws.yaml`, die im Paket
fehlt (vorhanden ist nur `genie_g2.yaml`).

---

## Zusammenfassung

| # | Problem | Ursache | Status |
|---|---|---|---|
| 1 | Package not found | Overlay nicht gesourct | gelöst (`source devel/setup.bash`) |
| 2 | Vulkan-Fehler | WSL-Treiber hat kein Linux-Vulkan | **nicht lösbar** unter WSL2 → Newton-Workaround |
| 3 | `python3\r` | CRLF im kopierten WSL-Clone | gelöst (5285 Dateien konvertiert, Backup vorhanden) |
| 4 | DDS-Warnings | Discovery an leere Ports | harmlos |
| 5 | Kein Fenster | Folge von 2 | gelöst über Newton-GL-Viewer |
| 6 | Roboter ohne Gelenke | keine Physics-Variant-Auswahl in `robot.usda` | gelöst (Code-Fix `assemble_robot.py`) |
| 7 | 1–3 FPS | OpenGL per Software (llvmpipe) | verbessert auf ~17 Hz (d3d12-GPU-Pfad) |
| 8 | Keine Reaktion auf Befehle | Newton 1.5 hat `joint_target_pos/vel` umbenannt | gelöst (Kompatibilitäts-Alias) |
| – | Fahren vor/zurück | Basis war fixiert (`pin_base_to_world: true`) | umgestellt + Teleop-Skript |

## Geänderte Dateien

| Datei | Änderung |
|---|---|
| `source/geniesim_ros/.../scripts/assemble_robot.py` | `_ensure_default_physics_variant()` + Aufruf nach dem Transformer |
| `docker/start.sh` | WSL-GPU-Mounts (`/dev/dxg`, `/usr/lib/wsl/lib`, `/usr/lib/wsl/drivers`) im GUI-Modus |
| `docker/entrypoint.sh` | `GALLIUM_DRIVER=d3d12` + `LD_LIBRARY_PATH` unter WSL |
| `source/geniesim_ros/.../scripts/engine/newton/_compat.py` (neu) + `__init__.py` | Newton-1.5-Alias für `joint_target_pos/vel` |
| `source/geniesim_ros/.../config/scene_pnp_g2_op.yaml` | `pin_base_to_world: false` |
| `Doku/drive_base.py` (neu) | Tastatur-Teleop für `/cmd_4ws` |
| 5285 Textdateien im WSL-Clone | CRLF → LF (nur `~/genie_sim`, nicht im Git-Repo auf `D:`) |

Backups in WSL: `~/genie_sim_crlf_backup_2026-10-11.tar.gz`,
`~/assemble_robot.py.orig_2026-10-11`, `~/genie_sim_orig_2026-10-11/` (start.sh, entrypoint.sh),
im Container `/tmp/robot_backup/` (alte `robot.usda`) und `urdf.sha256.bak`.

## Lessons Learned

1. **`nvidia-smi` ist kein Grafik-Test.** Es prüft nur CUDA/NVML. Für Vulkan
   `vulkaninfo --summary`, für OpenGL den Renderer-String abfragen (`glxinfo -B` bzw. pyglet).
2. **WSL2 + Docker Desktop:** CUDA funktioniert, RTX/Vulkan nicht. OpenGL nur mit
   zusätzlichen Mounts per D3D12 auf der GPU.
3. **Symlink-Installs:** `colcon build --symlink-install` bedeutet, dass Fehler in den
   Source-Dateien direkt durchschlagen und ein Rebuild sie nicht behebt.
4. **Gecachte Generator-Ausgaben** (`urdf.sha256`) können fehlerhafte Zwischenstände
   dauerhaft konservieren. Bei Verdacht den Stempel löschen.
5. **USD-VariantSets ohne Auswahl** erzeugen keinen Fehler, nur stillschweigend fehlende
   Inhalte. Das Symptom (Roboter sichtbar, aber ohne Gelenke) ist leicht zu übersehen.

## Referenzen

- [SETUP.md](../SETUP.md) — Installations- und Lifecycle-Anleitung
- [asset_consumption.md](../source/geniesim_ros/src/ros_ws/src/genie_sim_engine/docs/asset_consumption.md) — Physics-Varianten
- [launcher_newton_mjwarp.yaml](../source/geniesim_ros/src/ros_ws/src/genie_sim_bringup/config/launcher_newton_mjwarp.yaml) — Newton-Launcher inkl. Visualizer-Optionen
- [NVIDIA Dev Forum: Isaac Sim in WSL2](https://forums.developer.nvidia.com/t/is-it-possible-to-run-isaac-sim-in-wsl2/349609)
- [NVIDIA Container Toolkit Issue #233](https://github.com/NVIDIA/nvidia-container-toolkit/issues/233)
- [CUDA on WSL User Guide](https://docs.nvidia.com/cuda/wsl-user-guide/index.html)
