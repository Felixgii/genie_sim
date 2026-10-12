# GenieSim — Setup & Container-Lifecycle

Praxis-Anleitung für den täglichen Umgang mit dem GenieSim-Container unter WSL2 + Docker Desktop.

**Voraussetzungen (müssen auf dem Host da sein):**

- WSL2 (Ubuntu)
- Docker Desktop (mit WSL2-Backend)
- NVIDIA-GPU + Treiber
- Git

> **Variante & Container-Name:**
>
> - `geniesim docker` → Container `geniesim3` (Isaac Sim 5.1 — standard)
> - `geniesim docker5.1` → Container `geniesim3`
> - `geniesim docker4.5` → Container `geniesim2`
>
> In den Lifecycle-Befehlen unten `<container>` durch deinen Namen ersetzen (meist `geniesim3`).

---

## 1. Erststart auf einem neuen PC — Schritt für Schritt

### 1.1 Repo klonen (in native WSL-FS, NICHT auf `/mnt/d/`)

```bash
cd ~
git clone <repo-url> genie_sim
cd ~/genie_sim
```

### 1.2 Git-Konfiguration (gegen CRLF-Chaos)

```bash
git config core.autocrlf false
git config core.eol lf
```

### 1.3 Git LFS für große Dateien

```bash
sudo apt install -y git-lfs
git lfs install
git lfs pull
```

### 1.4 `uv` installieren (Python-Toolchain)

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env
```

### 1.5 Virtuelles Environment anlegen

```bash
uv venv --python 3.12 .venv
source .venv/bin/activate
```

### 1.6 `direnv` einrichten — venv automatisch aktivieren

```bash
sudo apt install -y direnv
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc
source ~/.bashrc

# .envrc für dieses Projekt anlegen und freigeben
echo 'source .venv/bin/activate' > .envrc
direnv allow
```

Ab jetzt reicht `cd ~/genie_sim` — direnv aktiviert das venv automatisch.

### 1.7 `geniesim` CLI installieren

```bash
uv pip install -e source/geniesim_cli/
```

Prüfen:

```bash
which geniesim
# → /home/felix/genie_sim/.venv/bin/geniesim
```

### 1.8 Assets runterladen (mehrere GB, 15-40 min)

```bash
uv pip install -U "huggingface_hub[cli]"
hf download agibot-world/GenieSimAssets \
  --repo-type dataset \
  --local-dir source/geniesim_assets
uv pip install -e source/geniesim_assets
```

> **Resumable:** `hf download` kann unterbrochen und einfach erneut gestartet werden.

### 1.9 Image bauen (20-40 min)

```bash
geniesim docker build
```

### 1.10 Container starten (5-10 min — entrypoint macht pip install -e)

```bash
geniesim docker up
```

Warte, bis `GENIESIM_READY` im Terminal erscheint.

### 1.11 Rein in den Container

```bash
geniesim docker into
```

Du bist drin, wenn der Prompt sich ändert zu `isaac-sim@...:/workspace$`.

---

## 2. Täglicher Lifecycle — Pause & Resume ohne Rebuild

### ⚠️ Wichtig: `down` ≠ `stop`

| Befehl                         | Was passiert                                          | Nächster Start       | Dauer Start |
| ------------------------------ | ----------------------------------------------------- | -------------------- | ----------- |
| `docker stop <container>`      | Prozesse beendet, **Container bleibt**                | `docker start`       | ~1-2 min    |
| `docker pause <container>`     | Prozesse eingefroren (SIGSTOP), **RAM bleibt belegt** | `docker unpause`     | sofort      |
| `geniesim docker down`         | Container **gelöscht** (Image bleibt)                 | `geniesim docker up` | ~5-10 min   |
| `geniesim docker build` + `up` | Image komplett neu                                    | —                    | 30-60 min   |

**Faustregel:** Für normale Pausen `docker stop` — NICHT `geniesim docker down`.

### Session beenden

```bash
docker stop geniesim3
```

### Session fortsetzen

```bash
cd ~/genie_sim                        # direnv lädt venv automatisch
docker start geniesim3
docker logs -f geniesim3              # warten bis GENIESIM_READY, dann Ctrl-C
geniesim docker into
```

### Kurze Pause (RAM-schonender als stop, aber hält Speicher belegt)

```bash
docker pause geniesim3        # einfrieren
docker unpause geniesim3      # weitermachen (sofort, kein entrypoint)
```

### Nach PC-Neustart

1. Docker Desktop starten (startet WSL mit)
2. In WSL:
   ```bash
   cd ~/genie_sim                   # direnv aktiviert venv
   docker start geniesim3
   docker logs -f geniesim3         # warten bis GENIESIM_READY
   geniesim docker into
   source /workspace/devel/setup.bash
   ```
3. sim starten:
   ```
   source /workspace/devel/setup.bash
   ros2 launch genie_sim_bringup app.launch.py scene:=scene_pnp_g2_op \
   launcher_config:=launcher_newton_mjwarp headless:=false
   physics_engine_visualizer:=newton
   ```
4. test nav satrten\_

   ```
   source /workspace/devel/setup.bash
   python3 /workspace/Doku/drive_base.py

   ```

---

## 3. Wenn der Container doch frisch sein muss

### Container neu, Image bleibt (~5-10 min)

Nach Änderungen an `docker/entrypoint.sh` oder wenn in-Container-State kaputt ist:

```bash
geniesim docker down
geniesim docker up
```

### Vollständiger Rebuild (30-60 min)

Nur bei Änderungen am Dockerfile oder Base-Image:

```bash
geniesim docker down
geniesim docker build
geniesim docker up
```

### Platz zurückgewinnen

```bash
geniesim docker down
docker image prune                 # ungenutzte Images weg
docker volume prune                # ungenutzte Volumes weg (Vorsicht: Caches)
docker system df                   # Übersicht wie viel wo liegt
```

---

## 4. Szene starten (im Container)

```bash
# im Container (geniesim docker into)
ros2 launch genie_sim_bringup app.launch.py \
  scene:=scene_pnp_g2_op \
  launcher_config:=launcher_ovrtx_isaac_physx \
  headless:=false
```

Falls der ROS-Workspace noch nicht gebaut ist:

```bash
geniesim ros build dev
```

---

## 5. Zeit-Erwartungen

| Schritt                      | native WSL-FS (`~/`) | Windows-FS (`/mnt/d/`) |
| ---------------------------- | -------------------- | ---------------------- |
| `git lfs pull`               | 1-3 min              | 5-10 min               |
| `hf download`                | 15-40 min            | 20-60 min              |
| `geniesim docker build`      | 20-40 min            | gleich                 |
| `geniesim docker up` (first) | 5-10 min             | 30-60 min              |
| `docker start` (resume)      | 1-2 min              | 2-5 min                |
| `geniesim ros build dev`     | 10-15 min            | 30-60 min              |

**Starke Empfehlung:** Repo in native WSL-FS (`~/genie_sim`) klonen, nicht auf `/mnt/d/`.

---

## 6. Troubleshooting

### `geniesim: command not found`

venv ist nicht aktiv. Fix:

```bash
cd ~/genie_sim          # direnv lädt venv
# oder manuell:
source .venv/bin/activate
```

Falls direnv nichts macht, ist der Hook noch nicht in bashrc geladen:

```bash
source ~/.bashrc
cd ~/genie_sim
```

### `direnv: /home/felix/genie_sim/.envrc is blocked`

```bash
direnv allow
```

### `.envrc:1: .venv/bin/activate: No such file or directory`

venv existiert noch nicht — Schritte 1.4-1.7 oben nachholen. Nach dem Erstellen:

```bash
direnv reload
```

### `geniesim_assets is not pip-installed (editable) on the host`

Assets fehlen oder sind nicht installiert. Schritt 1.8 oben.

### `/usr/bin/env: 'bash\r': No such file`

CRLF-Zeilenenden in einem Shell-Script (Windows-Problem). Fix auf dem Host:

```bash
git config core.autocrlf false
git config core.eol lf
git rm --cached -r .
git reset --hard
```

Wenn der Fehler aus einem Prebuilt-Image kommt, reicht `geniesim docker up` — das bind-mountet die saubere Host-Datei über die CRLF-Image-Version.

### `Container <name> exists (state: exited)` bei `geniesim docker up`

Container ist noch da von letzter Session. **Nicht `down`** — stattdessen:

```bash
docker start geniesim3
```

### Shell ohne CLI (Notfall-Zugang)

```bash
docker exec -it -u $(id -u):$(id -g) -w /workspace geniesim3 bash -l
```
