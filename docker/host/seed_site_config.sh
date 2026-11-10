#!/bin/bash
# Copy the starting files out of the image onto the host, so they always match its version:
#   - the Orbbec udev rule (the camera cannot be opened without it), and
#   - templates for /etc/ur30 (site parameters and the camera mount).
# Existing files in /etc/ur30 are never overwritten: they hold this robot's measurements.
#
#   ./seed_site_config.sh <image>      e.g. ./seed_site_config.sh ghcr.io/pvrohin/ur30_ibvs:v0.1.0
set -euo pipefail

IMAGE=${1:?usage: $0 <image>}
SHARE=/opt/ros/jazzy/share
WS_SHARE=/ws/install

from_image() { docker run --rm --entrypoint cat "${IMAGE}" "$1"; }

echo "==> udev rule for the Orbbec camera"
from_image "${SHARE}/orbbec_camera/udev/99-obsensor-libusb.rules" \
  | sudo tee /etc/udev/rules.d/99-obsensor-libusb.rules >/dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "    installed; unplug and replug the camera"

seed() {
  local source=$1 target=$2
  if [[ -e ${target} ]]; then
    echo "    ${target} exists, kept"
  else
    from_image "${source}" > "${target}"
    echo "    ${target} created from the template: edit it"
  fi
}
echo "==> /etc/ur30"
seed "${WS_SHARE}/ur30_real_bringup/share/ur30_real_bringup/config/ur30_ibvs_real.yaml" \
  /etc/ur30/ibvs_site.yaml
seed "${WS_SHARE}/ur30_cell_description/share/ur30_cell_description/config/camera_mount_nominal.yaml" \
  /etc/ur30/camera_mount.yaml
cat <<'EOF'

The templates are placeholders. Before the arm moves under IBVS:
  - ibvs_site.yaml: measure the marked values, then set site_configured: true
  - camera_mount.yaml: CAD transform, then the hand-eye calibration result
  - kinematics.yaml: from ur_calibration; then point KINEMATICS_FILE in .env at it
EOF
