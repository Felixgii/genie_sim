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

### B6 — 2D-Laserscan ohne RTX (2026-10-12)

**Ziel:** `sensor_msgs/LaserScan` für Nav2 (Costmap) und einen späteren Learned Planner: 360°,
~10 Hz, bis 10 m, 0,3 m über Boden. Der Scan soll die aktuelle Kollisionsgeometrie pro Step
treffen, damit auch bewegte Objekte (später Fußgänger) erscheinen. Auf dem Newton-Pfad gibt es
keine Kamera- oder Lidar-Topics, weil RTX hier nicht läuft (B1).

**Grundlage (im installierten Paket geprüft):** Newton 1.5 exportiert in `newton/sensors.py`
`SensorContact`, `SensorFrameTransform`, `SensorIMU` und `SensorTiledCamera`, aber kein Lidar.
`SensorTiledCamera` ist ein Warp-Raytracer (nur CUDA). Er liefert `depth` (Trefferabstand) und
`shape_index`. Strahlursprung und -richtung lassen sich pro Pixel frei vorgeben. Ein 1×360-„Bild“
mit horizontalen Strahlen ergibt damit einen Laserscan. Getroffen werden die Shapes der
Modell-BVH (`Model.bvh_build_shapes(shape_flags=...)`). `Model.bvh_refit_shapes(state)` passt sie
an die aktuellen `body_q` an.

**Umsetzung:** neues Modul
[laser_scan.py](../source/geniesim_ros/src/ros_ws/src/genie_sim_engine/scripts/engine/newton/laser_scan.py)
im Engine-Prozess. Ein separater Node käme an die GPU-Zustände nicht heran, und die Posen der
Szenenobjekte werden nicht publiziert. Es folgt dem Muster der bestehenden Debug-Publisher:
- **Schalter:** Block `newton.sensors.laser_scan` in der Szene-YAML. Ohne den Block wird das
  Modul nicht geladen.
- **Einhängepunkte:** Erzeugung in `setup/runtime.py`, Aufruf pro Tick in `tick_extras`
  (`engine_base.py`).

Details:
- BVH neu gebaut mit `ShapeFlags.COLLIDE_SHAPES`: Der Scan sieht Kollisions-, nicht Render-Geometrie.
- Scanner-Pose = `base_link`-Pose auf den Boden projiziert (x, y, Yaw) plus `height`. Das ist
  dieselbe Projektion, mit der die Engine `odom → base_footprint` publiziert. Neuer statischer
  Frame `base_footprint → base_scan` (z = 0,3 m).
- Takt 10 Hz nach **Wanduhr**, Stempel = Wanduhr wie bei den übrigen Engine-Topics. Mit
  Sim-Zeit-Takt kämen bei RTF ≈ 0,18 nur ~1,8 Hz an.
- Eigentreffer: Beim ersten Scan wird jeder Strahl, der zuerst den Roboter trifft, hinter
  dessen Collider gestartet (+2 cm). Spätere Roboter-Treffer (z.B. ein Arm in der Ebene) werden
  als „kein Echo“ (+inf) gemeldet.
- Parameter (YAML): `topic`, `frame_id`, `parent_frame`, `height`, `num_rays`, `range_min`
  (0 = automatisch), `range_max`, `rate_hz`, `dead_zone_margin`. QoS: Sensor-Data (Best Effort).
- Diagnose: `GENIESIM_LASER_SCAN_DEBUG=1` listet einmalig alle treffbaren Shapes mit
  Welt-AABB, dazu die Roboter-Collider in Bodennähe.

**Gefundene Lücke in Newton 1.5 (Workaround im Modul):** Der Raytracer nimmt den
Mesh-Schnitt-Pfad nur für `GeoType.MESH`. Alle anderen Typen gehen an
`raycast.ray_intersect_shape`, das laut Docstring nur Primitive behandelt. Konvexe Hüllen
(`CONVEX_MESH`, der übliche USD-Collider, z.B. die Flaschen) werden deshalb **nie getroffen**.
Newton macht für `HFIELD` bereits dasselbe Remapping. Das Modul gibt dem Raytracer ein eigenes
Typ-Array, in dem `CONVEX_MESH` als `MESH` geführt wird. `model.shape_type`, das die Kollision
nutzt, bleibt unverändert. Der Workaround greift auf ein privates Attribut zu
(`_SensorTiledCamera__render_context`). Fehlt es in einer späteren Newton-Version, gibt das Modul
eine Warnung aus und läuft ohne konvexe Hüllen weiter. Außerdem gilt: Fehlschüsse haben
`depth = 0.0` (Default-`ClearData`), nicht negativ wie im Docstring beschrieben. Das Modul
erkennt sie deshalb am `shape_index`.

**Kollisionsgeometrie auf 0,3 m (wichtig für die Interpretation):**
- **Szene `scene_pnp_g2_op`:** Auf Scan-Höhe gibt es **keinen** Collider. Vom Regal
  `supermarket_shelf_002` sind nur die vier Böden Collider (z = 0,09–0,14 / 0,54–0,59 / 0,99–1,04 /
  1,43–1,48 m), dazu Flaschen ab 1,05 m und der Boden. Es gibt keine Wände und keinen Tisch. Der Scan
  zeigt in der Originalszene deshalb überall +inf, und das ist korrekt.
- **Roboter:** Im unteren Bereich hat er nur die vier Radzylinder (z = 0–0,14 m,
  x ≈ −0,30…+0,30 m und y ≈ ±0,24 m um `base_link`) und `body_link2` (ab 0,48 m) als Collider.
  **Zwischen 0,14 und 0,48 m hat das Chassis keine Kollisionsgeometrie.**
- **Toter Bereich:** vorne / links / hinten / rechts jeweils **0,0 cm**. Die Strahlen starten im
  Scanner-Zentrum, und es gibt keine Eigentreffer. So steht es auch im Log
  (`laser_scan: dead zone (none -- no robot collider at scan height ...)`). Bekommt der Roboter
  später Collider auf Scan-Höhe, wird der tote Bereich beim Start automatisch berechnet und
  geloggt.

**Testaufbau:** temporär, nur in der WSL-Kopie, danach wieder entfernt:
- Zwei statische Wände über den vorhandenen Mechanismus `newton.entries` (`kind: box`):
  Innenfläche vorne bei x = 1,75 m, links bei y = 1,45 m.
- [scan_test_plugin.py](scan_test_plugin.py) als Scene-Plugin. Es führt die freie Flasche
  `iros_beverage_bottle_002` kinematisch auf 0,3 m Höhe neben dem Roboter entlang y = −2,5 … −0,8 m
  (Sinus, 6 s Sim-Zeit), indem es `joint_q`/`body_q` nach jedem Step schreibt.

Ein Plugin mit `on_build` hätte sich angeboten, aber dann verlangt die Engine einen
Partikel-Solver (`setup/model.py`).

**Testergebnisse:**

| Test | Ergebnis |
|---|---|
| `ros2 topic hz /scan` | 9,98 Hz (Fenster 12–22 Scans) |
| Typ / Frame | `sensor_msgs/msg/LaserScan`, `frame_id: base_scan`, 360 Strahlen, `range_max` 10 m; TF `base_footprint → base_scan` = (0, 0, 0,300) |
| Bekannte Abstände, Roboter am Start (x = −0,2) | 0°: 1,950 m (soll 1,75 − (−0,2) = 1,950) · +90°: 1,450 m (soll 1,450) |
| Fahrt auf die Wand zu (`/cmd_4ws`, 10 rad/s) | r(0°) 1,950 → 1,449 m, Abweichung zu 1,75 − x_odom: ±0,7 mm im Stand, max. −7 mm während der Fahrt (Abtastversatz Scan/Odom) |
| Bewegtes Objekt (20 Proben über 1,5 Hübe) | Range bei −90° folgt der Sollposition über 0,77–2,46 m, Abweichung meist 2 mm, max. 11 mm. Bei 0,8 m treffen 3 Strahlen die 5 cm breite Flasche, ab ~1,8 m einer |
| Drehung (Yaw −157°) | Scanpunkte im `odom`-Frame (über TF) liegen weiter exakt auf der Wandfläche, das Yaw-Vorzeichen ist also korrekt |
| RViz | LaserScan-Display (Best Effort) subskribiert, keine TF-/Drop-Meldungen im RViz-Log. Einen Screenshot erlaubt WSLg nicht (XWayland verweigert `GetImage`). Ersatz: eigene Draufsicht `/scan` in `odom` über TF, Punkte deckungsgleich mit Wänden und Flasche |

**Echtzeitfaktor** (je 30 s `/rtf`, ~500 Werte, Roboter im Stillstand, GL-Fenster offen):

| Lauf | /rtf Mittel (Median) | Render | `tick_extras` |
|---|---|---|---|
| Vorher (Original, ohne Modul) | 0,1795 (0,1846) | ~53 ms | – |
| A/B mit Testwänden: Scan an | 0,1796 (0,1833) | 44,5 ms | 10,0 ms |
| A/B mit Testwänden: Scan aus (`enabled: false`) | 0,1856 (0,1901) | 52,7 ms | 0,01 ms |
| Nachher (Endzustand, Scan an) | 0,1811 (0,1852) | 44,6 ms | 9,4 ms |

Ein Effekt ist **nicht von der Streuung zwischen Läufen zu unterscheiden**: Zwei Läufe ohne Scan
lagen bei 0,1795 und 0,1856. Der Scan kostet ~10 ms pro Tick in `tick_extras`, Render wird im
Gegenzug ~8 ms schneller. Das Zurücklesen (`numpy()`) synchronisiert die GPU und wartet dabei
größtenteils auf Arbeit, auf die sonst der Render-Schritt gewartet hätte. Falls das später
stört, wäre ein asynchrones Zurücklesen wie in `async_mirror.py` möglich. Dann käme der Scan
einen Tick später.

**Grenzen:**
- Kein Sensorrauschen, keine Strahlaufweitung, keine Intensitäten, keine Bewegungsverzerrung
  innerhalb eines Scans (alle 360 Strahlen zum selben Zeitpunkt).
- Nur Kollisionsgeometrie. Visuelle Details ohne Collider sieht der Scan nicht.
- Echte 2D-Ebene: Alles über oder unter 0,3 m ist unsichtbar. Beim Fahrtest blieb der Roboter
  nach 0,5 m am untersten Regalboden hängen (z = 0,09–0,14 m), den der Scan prinzipbedingt nicht
  sieht. Für die Nav2-Costmap fehlt dieses Hindernis also.
- Eine einzige BVH pro Modell: Eine spätere Depth-Kamera mit `SensorTiledCamera` sähe ebenfalls
  nur Kollisionsgeometrie, solange der Scan aktiv ist.
- Der Workaround für konvexe Hüllen hängt an einem privaten Newton-Attribut (siehe oben).

**Nebenbefunde (nicht behoben):**
- **Roboter fährt durch Testwand:** Bei der Rückwärtsfahrt mit Lenkeinschlag fuhr der Roboter
  durch die statische Testbox (`newton.entries`, `kind: box`), obwohl das Log „collision-group
  unification … can collide“ meldet. Nicht untersucht. Das ist relevant, sobald Fußgänger
  physisch mit dem Roboter interagieren sollen.
- **Veraltete Scene-Caches:** `assemble_scene.py` schreibt `newton_scene.usda` und
  `newton_solvers.json` in `/geniesim_assets/scenes/<scene>/` nur, wenn `newton.entries`
  existiert, löscht sie aber nicht, wenn die Einträge wieder entfernt werden. Die Engine lädt die
  alte Datei weiter (`setup/stage.py:571`). Die Test-Artefakte wurden danach von Hand gelöscht.
- **Testshell:** `docker exec … bash -c` liest `/etc/profile.d/geniesim.sh` nicht. Ohne
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` und `CYCLONEDDS_URI` (Loopback, kein Multicast) sieht
  man zwar Topics über den Daemon (`ros2 topic list`), bekommt aber keine Daten. So kamen
  anfangs keine Werte auf `/rtf`, und `tf2_echo` fand `base_link` nicht. Lösung: Login-Shell
  `docker exec -u isaac-sim geniesim3 bash -lc '…'` bzw. `geniesim docker into`.

### B7 — Chassis-Servo: Restbewegung nach Twist-Timeout (Code-Fix, 2026-10-13)

**Symptom:** Servo standalone (`genie_sim_chassis_servo_node`, `genie_g2.yaml`, `OptimalServo`).
Erst `/cmd_twist` mit `linear.x=0.2`, dann senden stoppen, dann `angular.z=0.5`. Der Roboter
dreht sich nicht auf der Stelle, sondern fährt eine Vorwärtskurve. Mit einem 0-Twist
dazwischen funktioniert es.

**Messung (vorher):** In den ersten `/cmd_4ws`-Ticks nach dem zweiten Befehl liegen die
Radgeschwindigkeiten bei 2,66/3,06 rad/s, also beim alten Vorwärtswert 2,857 rad/s
(0,2 m/s / 0,07 m). Danach bleibt der Servo dauerhaft in einer Vorwärtskurve hängen
(Lenkung ±0,30/±0,19 statt ±0,81 rad). Odometrie: +0,105 m vorwärts in 4 s bei vx≈0,21 m/s.
Mit 0-Twist dazwischen: −0,002 m, Spin-Stellung ±0,808 rad.

**Ursache:** Nach `twist_timeout_ms` (1000 ms) liefert der Servo `nullopt`. Es geht kein
`/cmd_4ws` mehr raus, und die Engine bremst die Räder (`cmd_4ws_timeout_s` = 0,1 s).
`ServoCore::prev_cmd_` behält aber die letzten Radgeschwindigkeiten. Beim nächsten Twist
startet `OptimalServo` mit diesen Werten
([optimal_servo.cpp:37-41](../source/geniesim_ros/src/ros_ws/src/genie_sim_ros_control/genie_sim_controllers/src/plugins/optimal_servo.cpp#L37-L41)),
die Lenkwinkel dagegen mit dem Ist-Zustand. Weil die Geschwindigkeiten über 1e-2 liegen,
wird in `getDirectCmd` die Regel „erst lenken, dann fahren“ übersprungen. Pro Tick darf sich die
Geschwindigkeit nur um 0,2 rad/s ändern, deshalb landet der MPC in der Vorwärtskurve.

**Fix:** [servo_core.cpp](../source/geniesim_ros/src/ros_ws/src/genie_sim_ros_control/genie_sim_controllers/src/servo_core.cpp)
`ServoCore::tick`: Wenn der Twist abgelaufen ist **und** der Servo `nullopt` liefert, werden
`prev_cmd_.drive_speeds` auf 0 gesetzt. Die Lenkwinkel bleiben stehen. Der Fix greift für alle
Servo-Modi und beide Wrapper (`servo_node`, `chassis_servo_controller`, gemeinsame
`libgenie_sim_controllers_servo_core.so`). Die Bedingung `nullopt` ist Absicht:
`SelftestServo` und `ParkingServo` senden auch ohne Twist. Würde dort `prev` jeden Tick genullt,
würde `ServoCore::limit()` den Selftest auf 0,2 rad/s pro Tick deckeln.
Build: `geniesim ros build dev` im Container (C++, Symlink-Install allein reicht nicht).

**Ergebnis (nachher, RTF 0,14–0,18):**

| Test | vorher | nachher |
|---|---|---|
| x=0.2 → 3 s Pause → z=0.5 | +0,105 m vorwärts, Vorwärtskurve | −0,001 bis −0,002 m, Spin-Stellung (3 Läufe) |
| x=0.2 → z=0.5 ohne Pause | – | erste Ticks identisch zu vorher (2,66/3,06 …), max. `/cmd_4ws`-Lücke 11 ms → Fix-Zweig nicht aktiv |
| rückwärts / vorwärts | −0,117 / +0,114 m | −0,107 / +0,097 m |
| seitwärts −y (Räder schon quer) | −0,078 m | −0,096 m |
| seitwärts +y aus dem Stand, 20 s | – | ~4 s Umlenken, dann +0,49 m |
| GeneralServo, SpinServo, ParkingServo | – | unverändert funktionsfähig (Parking hält Kreuzstellung, sendet weiter) |

**Offen (schon vorher vorhanden, nicht durch den Fix verursacht):**
- **Kurve aus der Spin-Stellung:** `vx=0.15, wz=0.3` direkt nach einer Drehung endet in einer
  falschen Gleichgewichtslage (Lenkung −0,13/0,33/0,92/−1,29). Das passiert genauso beim
  durchgehenden Wechsel ohne Pause, also auf dem unveränderten Codepfad. Nach dem Fix gilt das auch
  für den Pause-Fall. Vorher landete der Pause-Fall dort zufällig richtig. Aus anderen
  Startstellungen (z. B. nach 0-Twist) wird die Kurve korrekt angefahren.
- **Seitwärts aus dem Stand braucht Anlaufzeit (kein Fehler):** Die Lenkung braucht von 0 auf
  ±1,571 rad etwa 4 s Wanduhrzeit. Bis dahin hält der MPC die Fahrgeschwindigkeit bei 0, danach
  fährt der Roboter mit 2,14 rad/s (0,15 m/s) seitwärts (+0,49 m in 20 s, mit Fix). Ein erster
  4-s-Test war dafür zu kurz und wurde zunächst fälschlich als „keine Bewegung“ gewertet.
  Langtest Kurve zur Kontrolle: Die falsche Lage bleibt 18 s konstant, die Ist-Lenkung entspricht
  dem Befehl. Das ist also keine langsame Konvergenz.
- **Hänger am Lenkanschlag (nur ohne Fix beobachtet):** Nach −y-Fahrt + Pause blieb der
  Original-Servo bei ±1,571 rad stehen, auch Drehen und 0-Twist halfen nicht. Mit Fix trat das
  in derselben Sequenz nicht auf.
- `use_drive_ltd` (in `genie_g2.yaml` aus): Der Tracking-Differentiator hält nach dem Timeout
  ebenfalls den alten Zustand und wird durch den Fix nicht zurückgesetzt.
- Testumgebung: Nach mehreren Läufen stand der Roboter bei x≈0,22 m an einem Hindernis
  (Vorderräder blockiert). Die Tests oben laufen deshalb jeweils ab einem frischen Sim-Start.

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
| Topics sichtbar, aber keine Daten (`ros2 topic echo` stumm, `tf2_echo`: frame does not exist) | `docker exec … bash -c` ist keine Login-Shell, DDS-Umgebung aus `/etc/profile.d/geniesim.sh` fehlt (B6) | `docker exec -u isaac-sim geniesim3 bash -lc '…'` oder `geniesim docker into` |
| Entfernte `newton.entries` wirken weiter | Veraltete `newton_scene.usda`/`newton_solvers.json` im Scene-Cache (B6) | Dateien in `/geniesim_assets/scenes/<scene>/` löschen |
| Änderungen auf D: kommen im Container nicht an | Container mountet die WSL-Kopie `~/genie_sim`, nicht das Git-Repo auf D: | Dateien kopieren und **nach LF konvertieren** (D: hat CRLF durch `autocrlf`) |

### Geänderte Dateien

| Datei | Änderung |
|---|---|
| `source/geniesim_ros/.../scripts/assemble_robot.py` | Default-Physics-Variante (B2) |
| `source/geniesim_ros/.../scripts/engine/newton/_compat.py` (neu), `__init__.py` | Newton-1.5-Alias (B3) |
| `docker/start.sh`, `docker/entrypoint.sh` | WSL-GPU-Mounts, `GALLIUM_DRIVER=d3d12` (B4); Mounts greifen erst nach `geniesim docker down && up` |
| `source/geniesim_ros/.../config/scene_pnp_g2_op.yaml` | `pin_base_to_world: false` (B5) |
| `Doku/drive_base.py` (neu) | Tastatur-Teleop |
| `source/geniesim_ros/.../scripts/engine/newton/laser_scan.py` (neu) | 2D-Laserscan (B6) |
| `source/geniesim_ros/.../scripts/engine/newton/engine_base.py`, `setup/runtime.py` | Einhängepunkte Laserscan (B6, je wenige Zeilen) |
| `source/geniesim_ros/.../config/scene_pnp_g2_op.yaml` | Block `newton.sensors.laser_scan` (B6) |
| `source/geniesim_ros/.../genie_sim_engine/AGENTS.md` | Datei-Layout um `laser_scan.py` ergänzt |
| `Doku/scan_test_plugin.py` (neu) | Test-Plugin bewegtes Objekt (B6), nur temporär aktiv |
| `source/geniesim_ros/.../genie_sim_controllers/src/servo_core.cpp` | `prev_cmd_.drive_speeds` bei Twist-Timeout auf 0 (B7) |

Backups in WSL: `~/genie_sim_crlf_backup_2026-10-11.tar.gz`, `~/assemble_robot.py.orig_2026-10-11`,
`~/genie_sim_orig_2026-10-11/`, `~/genie_sim/scene_pnp_g2_op.yaml.bak_2026-10-11`,
`~/servo_core.cpp.orig_2026-10-13`.

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
