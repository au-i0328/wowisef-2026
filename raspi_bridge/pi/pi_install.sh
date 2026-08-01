#!/usr/bin/env bash
# pi_install.sh
# ----------------------------------------------------------------
# Installs the climbing robot bridge on a Raspberry Pi Zero 2 W:
#   - Python deps (websockets, opencv, pyserial)
#   - System services (hostapd + dnsmasq configured by pi_ap_setup.sh)
#   - Two systemd services that autostart the bridge + video stream
# ----------------------------------------------------------------
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
RUN_USER="${SUDO_USER:-pi}"
RUN_GROUP="$(id -gn "$RUN_USER")"

if [[ $EUID -ne 0 ]]; then
  echo "Re-run as root: sudo $0" >&2
  exit 1
fi

echo "[*] System packages"
apt-get update -y
# libatlas-base-dev / libjpeg-dev are not strictly required — modern
# opencv-python-headless wheels ship their own native deps — but install them
# if the repo still has them (older Debian/Raspbian) for faster numpy/sklearn.
EXTRA_PKGS=()
for pkg in libatlas-base-dev libjpeg-dev; do
    if apt-cache show "$pkg" >/dev/null 2>&1; then
        EXTRA_PKGS+=("$pkg")
    else
        echo "    (skip) $pkg not in repo on this distro"
    fi
done
apt-get install -y python3 python3-pip python3-venv \
                   hostapd dnsmasq avahi-daemon \
                   "${EXTRA_PKGS[@]}" \
                   v4l-utils

echo "[*] Python venv -> /opt/climbingrobot"
install -d -o "$RUN_USER" -g "$RUN_GROUP" /opt/climbingrobot
python3 -m venv /opt/climbingrobot/venv
# shellcheck disable=SC1091
source /opt/climbingrobot/venv/bin/activate
pip install --upgrade pip
pip install --no-cache-dir \
    "websockets>=12" \
    "opencv-python-headless>=4.6" \
    "pyserial>=3.5"

echo "[*] Copying bridge scripts"
install -m 755 "$REPO_DIR/pi_serial_bridge.py" /opt/climbingrobot/pi_serial_bridge.py
install -m 755 "$REPO_DIR/pi_video_stream.py"   /opt/climbingrobot/pi_video_stream.py
chown -R "$RUN_USER:$RUN_GROUP" /opt/climbingrobot

echo "[*] Systemd units"
cat >/etc/systemd/system/serial-bridge.service <<EOF
[Unit]
Description=Climbing Robot Serial -> WebSocket bridge
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${RUN_USER}
WorkingDirectory=/opt/climbingrobot
ExecStart=/opt/climbingrobot/venv/bin/python /opt/climbingrobot/pi_serial_bridge.py
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

cat >/etc/systemd/system/video-stream.service <<EOF
[Unit]
Description=Climbing Robot USB webcam streamer
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${RUN_USER}
WorkingDirectory=/opt/climbingrobot
ExecStart=/opt/climbingrobot/venv/bin/python /opt/climbingrobot/pi_video_stream.py --device /dev/video0 --width 640 --height 480 --fps 20
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable --now serial-bridge.service
systemctl enable --now video-stream.service

echo "[*] All set. Now configure hostapd: sudo bash $REPO_DIR/pi_ap_setup.sh"
echo "    Hint: reboot to make sure the AP comes up cleanly."
