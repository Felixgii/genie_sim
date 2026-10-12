# Setup-Troubleshooting-Log

**Datum:** 2026-10-11
**Umgebung:** Windows 11 Pro · WSL2 (Ubuntu 24.04) · Docker Desktop · NVIDIA RTX 2080 Ti (Treiber 617.42, CUDA 13.4) · 24 GB RAM
**Container:** `geniesim3` (Isaac Sim 5.1/6.x, ROS 2 Jazzy, Newton 1.5.0) · Projekt in WSL: `~/genie_sim` → `/workspace`

Ziel war, die Szene `scene_pnp_g2_op` mit GUI zu starten und den Roboter anzusteuern.
Ergebnis: Die Simulation läuft mit der Newton-Physik-Engine und einem GPU-beschleunigten
OpenGL-Fenster. Der Standard-Pfad mit Isaac Sim und RTX-Rendering lief in diesem Setup nicht.

---

## Textbaustein für den Bericht

> Fließtext zur (ggf. gekürzten) Übernahme in die Arbeit. Quellenangaben siehe [Quellen](#quellen).

Die Simulationsumgebung GenieSim wurde unter Windows 11 in einem Docker-Container betrieben,
der über Docker Desktop auf dem WSL2-Subsystem läuft.

Das zentrale Hindernis war die Grafikschnittstelle. Isaac Sim und dessen RTX-Renderer setzen
Vulkan voraus. Im Container fand der Vulkan-Loader jedoch keinen Treiber: Die vom
NVIDIA-Treiber unter WSL2 bereitgestellten Linux-Bibliotheken umfassten CUDA, OptiX und eine
D3D12-Schnittstelle, aber keinen Vulkan-Treiber, und Docker Desktop reichte nur den CUDA-Teil
in den Container durch. Diagnostisch zeigte sich das daran, dass `nvidia-smi` und CUDA
funktionierten, `vulkaninfo` aber keinen Treiber fand. NVIDIA unterstützt Isaac-Sim-Container
unter WSL2 offiziell nicht [1], und für Vulkan in Docker-Desktop-Containern verweist NVIDIA auf
Docker Engine [2]. Der RTX-basierte Pfad ließ sich in dieser Konfiguration daher nicht
betreiben. Stattdessen wurde der Kit-freie Pfad mit der GPU-Physik-Engine Newton
(MuJoCo-Warp-Solver) verwendet, der ausschließlich CUDA benötigt und ein OpenGL-Fenster zur
Visualisierung bietet.

Im Betrieb traten zwei Fehler in der Simulationssoftware auf. Erstens besaß der Roboter keine
steuerbaren Gelenke: Die aus dem URDF erzeugte USD-Datei definiert ein VariantSet `Physics`
(`physx`, `physics`, `mujoco`, `none`), wählte aber keine Variante aus. Ohne Auswahl wird keine
Physics-Payload eingebunden, und der Roboter existiert nur als Geometrie ohne Gelenke, Massen
und Antriebe. Das Konvertierungsskript wurde so erweitert, dass es die in der
Projektdokumentation vorgesehene Default-Variante `physx` setzt; danach verfügte das Modell über
46 steuerbare Gelenke mit PD-Antrieben. Zweitens verwarf die Engine sämtliche Steuerbefehle.
Die im Container installierte Newton-Version 1.5 hat die Steuerpuffer `joint_target_pos` und
`joint_target_vel` in `joint_target_q` und `joint_target_qd` umbenannt; die Engine las die alten
Namen und erhielt stillschweigend keinen Puffer. Ein Kompatibilitäts-Alias stellte die
Ansteuerung von Armen und Fahrwerk wieder her.

Die Visualisierung lief zunächst mit nur 1–3 Bildern pro Sekunde, weil Mesa das OpenGL-Fenster
mangels GPU-Bibliotheken im Container per Software-Rasterizer (`llvmpipe`) renderte. Nach dem
Einbinden der WSL-GPU-Bibliotheken rendert OpenGL über Mesas `d3d12`-Treiber auf der GPU. Die
Simulationsschleife erreicht damit etwa 17 Hz bei einem Sollwert von 100 Hz, also rund 17 %
Echtzeit. Der Physik-Schritt benötigt dabei nur etwa 0,7 ms, das Rendern rund 54 ms; das
Rendern ist damit der begrenzende Faktor.

Damit sich der Roboter fortbewegen kann, wurde in der Szene `pin_base_to_world` deaktiviert.
Die Basis ist dadurch nicht mehr an der Welt fixiert, sondern unterliegt Schwerkraft und
Kontakten, wie in den übrigen G2-Szenen des Projekts. Das Fahrwerk wird über Lenkwinkel und
Radgeschwindigkeiten angesteuert.

Für die Reproduzierbarkeit ist festzuhalten, dass der Betrieb unter WSL2 und Docker Desktop von
NVIDIA nicht offiziell unterstützt wird und Treiber- oder Docker-Updates das Setup erneut
beeinträchtigen können. Für RTX-Rendering mit Isaac Sim ist ein natives Linux-System der
offiziell unterstützte Weg.

---

## Wesentliche Befunde

### B1 — Kein Vulkan im Container → RTX-Pfad nicht lauffähig

**Symptom:** `[omni.rtx] VkResult: ERROR_INCOMPATIBLE_DRIVER` beim Start mit
`launcher_ovrtx_isaac_physx`, kein Isaac-Sim-Fenster.

**Eigene Messung:**
- `nvidia-smi` funktioniert, `vulkaninfo --summary` meldet `libGLX_nvidia.so.0: cannot open shared object file` / `Found no drivers`.
- Das Vulkan-ICD im Image zeigt auf `libGLX_nvidia.so.0`. In `/usr/lib/wsl/lib` (WSL-Host,
  23 Dateien) gibt es weder diese Library noch einen anderen Vulkan-Treiber, nur CUDA-, OptiX-,
  NVENC/NVDEC- und D3D12-Bibliotheken.
- Im Container waren aus dem Treiberordner nur 8 CUDA-Dateien vorhanden.

**Quellen:** NVIDIA unterstützt Isaac-Sim-Container unter WSL2 nicht [1]. Für Vulkan in
Docker-Desktop-Containern (gleiche Fehlermeldung wie hier) verweist NVIDIA auf Docker Engine [2].

**Nicht getestet:** Docker Engine direkt in WSL, Mesas Vulkan-auf-D3D12-Treiber `dzn` [3],
neuere Treiber. Ob `dzn` die von RTX benötigten Erweiterungen bietet, ist ungeprüft. Belegt ist
also nur: in diesem Setup nicht lauffähig und offiziell nicht unterstützt.

**Workaround:** Newton-Pfad (nur CUDA) mit OpenGL-Viewer, siehe [Anhang](#start).

### B2 — Roboter ohne Gelenke: fehlende Physics-Variante (Code-Fix)

**Symptom:** Roboter im Fenster sichtbar, aber `joint map: 0 controllable joint(s)`.

**Ursache:** `robot.usda` deklariert `variantSet "Physics"`, setzt aber keine Auswahl. Laut
[asset_consumption.md](../source/geniesim_ros/src/ros_ws/src/genie_sim_engine/docs/asset_consumption.md#L25-L36)
müsste `variants = { string Physics = "physx" }` gesetzt sein. Ohne Auswahl wird keine
`payloads/Physics/*.usda` eingebunden. Auch die Nachbearbeitung in `assemble_robot.py` findet
dann keine Physics-Prims. Ein Neuerzeugen ergab dieselbe Datei, der Transformer im Image setzt
die Auswahl also nie. (Hinweis kam von einem Kollegen: "no physics variant selection".)

**Fix:** [assemble_robot.py](../source/geniesim_ros/src/ros_ws/src/genie_sim_engine/scripts/assemble_robot.py):
neue Funktion `_ensure_default_physics_variant()`, setzt `Physics = "physx"` direkt nach dem
Transformer. Danach `urdf.sha256` löschen, damit der Roboter neu erzeugt wird.

**Ergebnis:** `joint map: 46 controllable joint(s)`, `PD drives: 46/46`, 70 Bodies.

**Nebenbefund:** `assemble_robot.py` schreibt den Erfolgsstempel `urdf.sha256` im
`finally`-Block. Eine Exception in der Nachbearbeitung würde als Erfolg gecacht.

### B3 — Alle Steuerbefehle verworfen: Newton-1.5-API (Code-Fix)

**Symptom:** `/cmd_4ws` und `/joint_command` ohne Wirkung. Im Log:
`control.joint_target_pos is None; position commands will be silently dropped`.

**Ursache:** Newton 1.5.0 hat `Control.joint_target_pos/vel` in `joint_target_q/qd` umbenannt.
Die alten Namen werfen beim Lesen einen `AttributeError`, `getattr(..., None)` liefert `None`, und
`apply_commands` bricht ab ([control.py:104-106](../source/geniesim_ros/src/ros_ws/src/genie_sim_engine/scripts/engine/newton/control.py#L104-L106)).

**Fix:** [engine/newton/_compat.py](../source/geniesim_ros/src/ros_ws/src/genie_sim_engine/scripts/engine/newton/_compat.py)
(aufgerufen in `engine/newton/__init__.py`) leitet die alten Namen auf die neuen um. Das ist
bedeutungsgleich, solange `newton.use_coord_layout_targets = False` (Default). Bei älteren
Newton-Versionen ist das Modul ein No-op. Alternative wäre, ~40 Codestellen umzubenennen.

**Offen:** Warnung `mujoco==3.8.1 (requires ~=3.11.0)`, bisher ohne beobachtete Auswirkung.

### B4 — Performance: Software-Rendering → GPU-OpenGL

**Symptom:** 1–3 FPS, CPU und GPU scheinbar kaum ausgelastet.

**Ursache:** Mesa renderte mit `llvmpipe` (Software, im Wesentlichen ein CPU-Kern), weil im
Container die WSL-Grafikbibliotheken fehlten. Mesas `d3d12`-Treiber braucht
`/usr/lib/wsl/lib/libd3d12*.so`, `libdxcore.so` und `libnvwgf2umx.so` aus dem
NVIDIA-Treiberordner (ermittelt per `LD_DEBUG=files`).

**Fix:** [docker/start.sh](../docker/start.sh) mountet im GUI-Modus unter WSL `/dev/dxg`,
`/usr/lib/wsl/lib` und `/usr/lib/wsl/drivers`. [docker/entrypoint.sh](../docker/entrypoint.sh)
setzt dann `GALLIUM_DRIVER=d3d12` und ergänzt `LD_LIBRARY_PATH`.

**Ergebnis:**

| | vorher | nachher |
|---|---|---|
| OpenGL-Renderer | `llvmpipe` | `D3D12 (NVIDIA GeForce RTX 2080 Ti)` |
| Schleifenrate | 1–3 Hz | ~17 Hz (Soll 100 Hz) |
| Physik-Schritt | – | ~0,7 ms |
| Render-Schritt | – | ~54 ms |

Jeder Tick ist ein Render-Tick, die Physikrate hängt also an der Bildrate. Ein niedrigeres
`render_hz` könnte mehr Physik-Schritte pro Bild erlauben (nicht getestet).

### B5 — Szenen-Änderung: mobile Basis

**Änderung:** [scene_pnp_g2_op.yaml](../source/geniesim_ros/src/ros_ws/src/genie_sim_bringup/config/scene_pnp_g2_op.yaml):
`pin_base_to_world: true` → `false`. Vorher war die Basis per `root_joint` fixiert (Substep
"kinematic-control", ohne Schwerkraft und Kontakte). Jetzt läuft sie unter voller Physik, wie in
`scene_wbc_g2_sp` und `scene_flat_g2_sp`.

**Ansteuerung:** `/cmd_4ws` (`sensor_msgs/JointState`, QoS BEST_EFFORT). Die Lenk-Joints
`*_joint1` bekommen eine Position [rad], die Antriebs-Joints `*_joint2` eine Geschwindigkeit
[rad/s]. Ohne laufende Nachrichten bremst die Engine nach `cmd_4ws_timeout_s`.

**Test:** 6 s mit +3 rad/s, dann 6 s mit −3 rad/s → Radgeschwindigkeit 2,99 rad/s,
Odometrie x: −0,200 → 0,021 → −0,177 m. Kurze Strecke wegen ~17 % Echtzeit.

---

## Anhang: Betrieb

### Start

Im Container (`geniesim docker into`):
```bash
source /workspace/devel/setup.bash
ros2 launch genie_sim_bringup app.launch.py scene:=scene_pnp_g2_op \
  launcher_config:=launcher_newton_mjwarp headless:=false physics_engine_visualizer:=newton
# einmal ENTER (Hinweis auf experimentellen Launcher)
```
Fahren, im zweiten Terminal: `python3 /workspace/Doku/drive_base.py`
(`w` vor, `s` zurück, Leertaste stopp, `+`/`-` Tempo, `q` beenden).

Beim ersten Start kompiliert Warp die CUDA-Kernel (mehrere Minuten, danach gecacht).

### Kleinere Probleme

| Problem | Ursache | Lösung |
|---|---|---|
| `Package 'genie_sim_bringup' not found` | Overlay nicht gesourct | `source /workspace/devel/setup.bash` in jeder neuen Shell |
| `/usr/bin/env: 'python3\r'` | CRLF in 5285 Dateien, WSL-Kopie kopiert statt geklont (`.git` defekt) | nach LF konvertiert (Backup vorhanden); langfristig neu klonen mit `core.autocrlf=false` |
| `ddsi_udp_conn_write ... retcode -3` | CycloneDDS-Discovery an leere Loopback-Ports | harmlos |
| Kein Isaac-Sim-Fenster | Folge von B1 | Newton-GL-Viewer |
| Python-Änderung ohne Wirkung? | – | Skripte sind per Symlink installiert, kein Rebuild nötig. Neue Dateien brauchen einen Symlink in `devel/` bzw. `geniesim ros build dev` |

### Geänderte Dateien

| Datei | Änderung |
|---|---|
| `source/geniesim_ros/.../scripts/assemble_robot.py` | Default-Physics-Variante (B2) |
| `source/geniesim_ros/.../scripts/engine/newton/_compat.py` (neu), `__init__.py` | Newton-1.5-Alias (B3) |
| `docker/start.sh`, `docker/entrypoint.sh` | WSL-GPU-Mounts, `GALLIUM_DRIVER=d3d12` (B4); Mounts greifen erst nach `geniesim docker down && up` |
| `source/geniesim_ros/.../config/scene_pnp_g2_op.yaml` | `pin_base_to_world: false` (B5) |
| `Doku/drive_base.py` (neu) | Tastatur-Teleop |

Backups in WSL: `~/genie_sim_crlf_backup_2026-10-11.tar.gz`, `~/assemble_robot.py.orig_2026-10-11`,
`~/genie_sim_orig_2026-10-11/`, `~/genie_sim/scene_pnp_g2_op.yaml.bak_2026-10-11`.

### Lessons Learned

1. `nvidia-smi` prüft nur CUDA. Für Grafik `vulkaninfo --summary` bzw. den OpenGL-Renderer-String abfragen.
2. Stille Fehler sind die teuersten: Eine fehlende Varianten-Auswahl und ein umbenanntes
   API-Attribut erzeugten keine Fehlermeldung, sondern nur fehlende Funktion.
3. Gecachte Generator-Ausgaben (`urdf.sha256`) können fehlerhafte Zwischenstände konservieren.

---

## Quellen

Abgerufen am 2026-10-11. Die Zitate stammen aus der Zusammenfassung beim Abruf. **Vor der
Übernahme in die Arbeit den genauen Wortlaut auf der Originalseite prüfen.**

| # | Quelle | Belegt |
|---|---|---|
| [1] | NVIDIA Developer Forums: *Is it possible to run Isaac Sim in WSL2?* — https://forums.developer.nvidia.com/t/is-it-possible-to-run-isaac-sim-in-wsl2/349609 | NVIDIA-Antwort sinngemäß: *"Isaac Sim containers are currently not supported under WSL2. This is due to GPU and RTX feature limitations inside WSL2."* |
| [2] | GitHub, NVIDIA/nvidia-container-toolkit, Issue #233: *Docker Desktop - Vulkan Drivers not found* — https://github.com/NVIDIA/nvidia-container-toolkit/issues/233 | Gleiche Fehlermeldung (`libGLX_nvidia.so.0: cannot open shared object file`). Ein Collaborator sinngemäß: *"currently Docker Desktop is not supported [...] please give it a try with Docker engine."* |
| [3] | NVIDIA Developer Forums: *Vulkan Fails to Detect NVIDIA GPU on WSL2 Ubuntu 24.04 – dzn Driver Files Missing* — https://forums.developer.nvidia.com/t/vulkan-fails-to-detect-nvidia-gpu-on-wsl2-ubuntu-24-04-dzn-driver-files-missing-tested-on-multiple-systems/342142 | Vulkan in WSL2 läuft über Mesas D3D12-basierten `dzn`-Treiber. Fehlt dieser, fällt Vulkan auf `llvmpipe` zurück. (Nutzerbericht, keine offizielle NVIDIA-Aussage) |
| [4] | NVIDIA: *CUDA on WSL User Guide* — https://docs.nvidia.com/cuda/wsl-user-guide/index.html | Offizielle Doku zu CUDA unter WSL2; in WSL keinen Linux-Treiber installieren, der Windows-Treiber stellt CUDA bereit. (Nur über die Suche gesichtet, nicht im Detail gelesen) |

Projektinterne Referenzen: [SETUP.md](../SETUP.md),
[asset_consumption.md](../source/geniesim_ros/src/ros_ws/src/genie_sim_engine/docs/asset_consumption.md),
[launcher_newton_mjwarp.yaml](../source/geniesim_ros/src/ros_ws/src/genie_sim_bringup/config/launcher_newton_mjwarp.yaml).
