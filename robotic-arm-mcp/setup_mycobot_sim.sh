#!/usr/bin/env bash
# setup_mycobot_sim.sh — Idempotent setup for myCobot 280 Pi simulation
# ROS 2 Jazzy + Gazebo Harmonic on Ubuntu 24.04 (Noble) with NVIDIA GPU
set -euo pipefail

YELLOW='\033[1;33m'
GREEN='\033[1;32m'
RED='\033[1;31m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

# ── 1. Repositorio ROS 2 ────────────────────────────────────────────────────
info "1/7 — Configurando repositorio de ROS 2 Jazzy..."

ROS_KEYRING=/usr/share/keyrings/ros-archive-keyring.gpg
if [ ! -f "$ROS_KEYRING" ]; then
    sudo apt-get update -qq
    sudo apt-get install -y -qq curl gnupg lsb-release software-properties-common
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o "$ROS_KEYRING"
    info "Llave GPG de ROS 2 instalada."
else
    info "Llave GPG de ROS 2 ya existe, saltando."
fi

ROS_LIST=/etc/apt/sources.list.d/ros2.list
if [ ! -f "$ROS_LIST" ]; then
    echo "deb [arch=$(dpkg --print-architecture) signed-by=$ROS_KEYRING] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | sudo tee "$ROS_LIST" > /dev/null
    info "Repositorio ROS 2 agregado."
else
    info "Repositorio ROS 2 ya configurado, saltando."
fi

sudo apt-get update -qq

# ── 2. Instalación de ROS 2 Jazzy ───────────────────────────────────────────
info "2/7 — Instalando ROS 2 Jazzy Desktop y paquetes de simulación..."

PKGS=(
    ros-jazzy-desktop
    ros-dev-tools
    ros-jazzy-ros-gz
    ros-jazzy-moveit
    ros-jazzy-ros2-control
    ros-jazzy-ros2-controllers
    ros-jazzy-xacro
    ros-jazzy-joint-state-publisher-gui
)

# Instalar solo los que faltan
TO_INSTALL=()
for pkg in "${PKGS[@]}"; do
    if ! dpkg -s "$pkg" &>/dev/null; then
        TO_INSTALL+=("$pkg")
    fi
done

if [ ${#TO_INSTALL[@]} -gt 0 ]; then
    sudo apt-get install -y "${TO_INSTALL[@]}"
    info "Paquetes ROS 2 instalados: ${TO_INSTALL[*]}"
else
    info "Todos los paquetes ROS 2 ya están instalados."
fi

# ── 3. Dependencias de hardware (pymycobot, serial) ─────────────────────────
info "3/7 — Instalando dependencias de hardware..."

sudo apt-get install -y -qq python3-pip python3-serial

if ! python3 -c "import pymycobot" &>/dev/null; then
    pip3 install --user --break-system-packages pymycobot
    info "pymycobot instalado."
else
    info "pymycobot ya instalado."
fi

# ── 4. Workspace y clonado ──────────────────────────────────────────────────
info "4/7 — Configurando workspace ~/mycobot_ws..."

WS=~/mycobot_ws
mkdir -p "$WS/src"

# El repo UoMMScRobotics/maniupulator-jazzy es solo documentación, no contiene paquetes ROS.
# El repositorio correcto es el oficial de Elephant Robotics, rama humble (compatible con Jazzy).
REPO_DIR="$WS/src/mycobot_ros2"
if [ ! -d "$REPO_DIR/.git" ]; then
    git clone --branch humble https://github.com/elephantrobotics/mycobot_ros2.git "$REPO_DIR"
    info "Repositorio mycobot_ros2 (rama humble) clonado."
else
    info "Repositorio ya existe, haciendo pull..."
    git -C "$REPO_DIR" pull --ff-only || warn "Pull falló, usando versión existente."
fi

# rosdep init (idempotente)
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
    info "rosdep inicializado."
else
    info "rosdep ya inicializado."
fi

# Corregir permisos de ~/.ros si fueron creados como root (por sudo rosdep init)
if [ -d ~/.ros ] && [ "$(stat -c '%U' ~/.ros)" = "root" ]; then
    warn "~/.ros es propiedad de root, corrigiendo permisos..."
    sudo chown -R "$USER:$USER" ~/.ros
    info "Permisos corregidos."
fi

rosdep update --rosdistro=jazzy

# Sourcing ROS 2 para que rosdep install funcione
# set +u porque setup.bash de ROS referencia variables no inicializadas (AMENT_TRACE_SETUP_FILES)
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

info "Instalando dependencias del workspace con rosdep..."
rosdep install --from-paths "$WS/src" --ignore-src -r -y --rosdistro=jazzy || \
    warn "Algunas dependencias de rosdep no se pudieron resolver (puede ser normal)."

# ── 5. Compilación ──────────────────────────────────────────────────────────
info "5/7 — Compilando workspace con colcon..."

cd "$WS"
colcon build --symlink-install 2>&1 | tail -20
info "Compilación completada."

# ── 6. Post-instalación (.bashrc) ───────────────────────────────────────────
info "6/7 — Configurando ~/.bashrc..."

BASHRC=~/.bashrc

add_to_bashrc() {
    local line="$1"
    if ! grep -qF "$line" "$BASHRC"; then
        echo "$line" >> "$BASHRC"
        info "Agregado a .bashrc: $line"
    else
        info "Ya en .bashrc: $line"
    fi
}

add_to_bashrc "source /opt/ros/jazzy/setup.bash"
add_to_bashrc "source ~/mycobot_ws/install/setup.bash"

# ── 7. Verificación de GPU ──────────────────────────────────────────────────
info "7/7 — Verificación de GPU NVIDIA..."

if command -v nvidia-smi &>/dev/null; then
    echo ""
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv
    echo ""
    # Verificar que el renderer OpenGL use NVIDIA
    if command -v glxinfo &>/dev/null; then
        RENDERER=$(glxinfo | grep "OpenGL renderer" || true)
        if echo "$RENDERER" | grep -qi nvidia; then
            info "OpenGL usa la GPU NVIDIA — Gazebo usará la RTX 5070."
        else
            warn "OpenGL NO reporta NVIDIA: $RENDERER"
            warn "Gazebo podría no usar la GPU. Verifica con: sudo prime-select nvidia"
        fi
    else
        warn "glxinfo no instalado. Instala con: sudo apt install mesa-utils"
    fi
else
    error "nvidia-smi no encontrado. Verifica los drivers NVIDIA."
fi

echo ""
info "═══════════════════════════════════════════════════════════"
info "  Setup completo. Abre una nueva terminal o ejecuta:"
info "    source ~/.bashrc"
info "  Luego prueba el stack de simulación del repo:"
info "    source ~/Documents/Repos/RoboticArm/venv_ai/bin/activate"
info "    python ~/Documents/Repos/RoboticArm/robotic-arm-mcp/sim/sim_driver_node.py"
info "  Y en otra terminal:"
info "    source ~/Documents/Repos/RoboticArm/venv_ai/bin/activate"
info "    python ~/Documents/Repos/RoboticArm/robotic-arm-mcp/sim/sim_viewer.py"
info "  O con MoveIt2:"
info "    ros2 launch mycobot_280_moveit2 demo.launch.py"
info "═══════════════════════════════════════════════════════════"
