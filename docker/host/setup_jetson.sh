#!/bin/bash
# One-time setup of the robot computer (Jetson AGX Orin, JetPack 7.2.x, Ubuntu 24.04).
# Everything ROS runs in containers; this only prepares the host. Safe to run again.
#
#   sudo ./setup_jetson.sh <robot-network-interface> [jetson-ip/prefix]
#   e.g. sudo ./setup_jetson.sh eth0 192.168.56.2/24      (list interfaces: ip -br link)
#
# Afterwards, as your normal user: log out and back in (docker group), then run
# seed_site_config.sh once the image has been pulled. See docs/DEPLOYMENT.md, section 3.
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "run with sudo" >&2
  exit 1
fi
IFACE=${1:?usage: sudo $0 <robot-network-interface> [ip/prefix]}
ADDRESS=${2:-192.168.56.2/24}
USER_NAME=${SUDO_USER:?run with sudo from your normal user, not as root}

step() { echo; echo "==> $*"; }

step "Checking this is JetPack 7 (Jetson Linux R39)"
if ! grep -q "R39" /etc/nv_tegra_release 2>/dev/null; then
  echo "expected R39 in /etc/nv_tegra_release, found: $(head -1 /etc/nv_tegra_release 2>/dev/null || echo none)" >&2
  echo "flash JetPack 7.2.x first (docs/DEPLOYMENT.md, section 1)" >&2
  exit 1
fi

step "Hostname"
hostnamectl set-hostname ur30-jetson

step "Static address ${ADDRESS} on ${IFACE} for the robot network (no gateway: internet stays on Wi-Fi)"
if nmcli -t -f NAME con show | grep -qx robot; then
  nmcli con modify robot ifname "${IFACE}" ipv4.method manual ipv4.addresses "${ADDRESS}" \
    ipv4.gateway "" ipv6.method disabled
else
  nmcli con add type ethernet ifname "${IFACE}" con-name robot ipv4.method manual \
    ipv4.addresses "${ADDRESS}" ipv6.method disabled
fi
nmcli con up robot || echo "could not bring up 'robot' yet (cable unplugged?); it will start when linked"

step "Docker"
if ! command -v docker >/dev/null; then
  curl -fsSL https://get.docker.com | sh
fi
usermod -aG docker "${USER_NAME}"

step "Docker log rotation"
mkdir -p /etc/docker
if [[ ! -f /etc/docker/daemon.json ]]; then
  cat > /etc/docker/daemon.json <<'EOF'
{
  "log-driver": "json-file",
  "log-opts": { "max-size": "50m", "max-file": "5" }
}
EOF
  systemctl restart docker
else
  echo "/etc/docker/daemon.json exists, left unchanged. NVIDIA's runtime settings may be in it."
fi

step "Time sync"
apt-get update
apt-get install -y chrony

step "No automatic upgrades of the board support package or the OS under a running robot"
apt-mark hold 'nvidia-l4t-*' >/dev/null
systemctl disable --now unattended-upgrades 2>/dev/null || true

step "No desktop: frees CPU and memory. Undo with: sudo systemctl set-default graphical.target"
systemctl set-default multi-user.target

step "Folders for per-robot files and recordings"
mkdir -p /etc/ur30 /var/lib/ur30
chown "${USER_NAME}": /etc/ur30 /var/lib/ur30

step "Power mode"
nvpmodel -q || true
cat <<'EOF'

Done. Still to do by hand:
  1. Pick the maximum power mode from the list above:  sudo nvpmodel -m <id>
     and lock the clocks at boot if you want stable timing:  sudo jetson_clocks
  2. Log out and back in, so that docker works without sudo.
  3. Pull the image and run seed_site_config.sh.
EOF
