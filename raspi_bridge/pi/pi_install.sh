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
# NB: libatlas-base-dev / libjpeg-dev used to be helpful for numpy + opencv
# wheels, but on Debian trixie the dev packages are gone or renamed and the
# opencv-python-headless wheels we use ship their own native deps. Skip them.
apt-get install -y --no-install-recommends \
    python3 python3-pip python3-venv \
    hostapd dnsmasq avahi-daemon \
    v4l-utils

# Make sure the run user can talk to the Arduino (ttyACM*) and to the camera.
# dialout -> /dev/ttyACM* / /dev/ttyUSB*; video -> /dev/video*.
for grp in dialout video; do
    if getent group "$grp" >/dev/null; then
        usermod -aG "$grp" "$RUN_USER" || true
    fi
done

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
Group=${RUN_GROUP}
SupplementaryGroups=dialout video
WorkingDirectory=/opt/climbingrobot
# --auto-test = fall back to mock serial if the Arduino never shows up
# --wait-serial=-1 = wait indefinitely for the real Arduino
# EnvironmentFile lets admin override:  TEST_MODE=1 in the env file forces
# mock mode immediately.
EnvironmentFile=-/etc/default/climbingrobot
ExecStart=/opt/climbingrobot/venv/bin/python /opt/climbingrobot/pi_serial_bridge.py \\
    --auto-test --wait-serial=-1 --wait-device=-1
Restart=on-failure
RestartSec=5
StartLimitIntervalSec=120
StartLimitBurst=8

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
Group=${RUN_GROUP}
SupplementaryGroups=video
WorkingDirectory=/opt/climbingrobot
EnvironmentFile=-/etc/default/climbingrobot
ExecStart=/opt/climbingrobot/venv/bin/python /opt/climbingrobot/pi_video_stream.py \\
    --device /dev/video0 --width 640 --height 480 --fps 20 \\
    --auto-test --wait-device=-1
Restart=on-failure
RestartSec=5
StartLimitIntervalSec=120
StartLimitBurst=8

[Install]
WantedBy=multi-user.target
EOF

# /etc/default/climbingrobot lets admins force mock mode at boot.
cat >/etc/default/climbingrobot <<EOF
# Set to 1 to force both Pi services into pure --test mode at startup.
# Leave at 0 (default) to keep waiting for real hardware and fall back
# automatically if it never appears.
TEST_MODE=0
EOF

systemctl daemon-reload
systemctl enable --now serial-bridge.service
systemctl enable --now video-stream.service

echo "[*] All set. Now configure hostapd: sudo bash $REPO_DIR/pi_ap_setup.sh"
echo "    Hint: reboot to make sure the AP comes up cleanly."
echo "    Hardware-less tip:"
echo "        sudo sed -i 's/^TEST_MODE=.*/TEST_MODE=1/' /etc/default/climbingrobot"
echo "        sudo systemctl restart serial-bridge video-stream"
