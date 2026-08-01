#!/usr/bin/env bash
# pi_ap_setup.sh
# ----------------------------------------------------------------
# Configure the Pi Zero 2 W's external WLAN adapter (typically wlan1
# when running on a Pi with builtin wifi + USB dongle, but supports
# wlan0 as well) as a WiFi access point hosting the ClimbingRobot
# network.  This script is idempotent: re-running it will not change
# settings that have already been written.
#
# Defaults (edit at the top to change):
#   AP_SSID     = ClimbingRobot
#   AP_PASSWORD = climb12345
#   AP_CHANNEL  = 5        # 1..11
#   AP_IP       = 192.168.4.1/24
#   AP_IFACE    = wlan1    # the external adapter; falls back to wlan0
#   WIFI_MODE   = g        # 802.11g/n widely compatible
# ----------------------------------------------------------------
set -euo pipefail

AP_SSID="ClimbingRobot"
AP_PASSWORD="climb12345"
AP_CHANNEL="5"
AP_IP="192.168.4.1"
AP_NETMASK="255.255.255.0"
AP_NET="${AP_IP%.*}.0/24"
AP_DHCP_START="192.168.4.10"
AP_DHCP_END="192.168.4.100"
AP_IFACE="${AP_IFACE:-wlan1}"

WIFI_MODE="g"

if [[ $EUID -ne 0 ]]; then
  echo "Re-run as root: sudo $0" >&2
  exit 1
fi

# Detect the hostapd-capable interface. Some Pi OS setups bring up
# wlan0 (broadcom). Else, use a USB Atheros/RTL8812AU adapter.
if ! ip link show "$AP_IFACE" &>/dev/null; then
  for cand in wlan1 wlan0; do
    if ip link show "$cand" &>/dev/null; then
      AP_IFACE="$cand"
      break
    fi
  done
fi
echo "[*] Using interface: $AP_IFACE"

echo "[*] Installing hostapd + dnsmasq"
apt-get update -y
apt-get install -y hostapd dnsmasq

# Disable NetworkManager management of the AP interface
mkdir -p /etc/NetworkManager/conf.d
cat >/etc/NetworkManager/conf.d/ignore-ap-iface.conf <<EOF
[keyfile]
unmanaged-devices=interface-name:${AP_IFACE}
EOF

echo "[*] Static IP for $AP_IFACE"
cat >/etc/network/interfaces.d/${AP_IFACE} <<EOF
allow-hotplug ${AP_IFACE}
iface ${AP_IFACE} inet static
    address ${AP_IP}
    netmask ${AP_NETMASK}
EOF

# Bring up the interface with the new address
ifdown "$AP_IFACE" 2>/dev/null || true
ifup "$AP_IFACE" 2>/dev/null || true

echo "[*] hostapd config -> /etc/hostapd/hostapd.conf"
cat >/etc/hostapd/hostapd.conf <<EOF
interface=${AP_IFACE}
driver=nl80211
ssid=${AP_SSID}
hw_mode=${WIFI_MODE}
channel=${AP_CHANNEL}
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=${AP_PASSWORD}
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
EOF

# Tell hostapd where to read its config
grep -q '^DAEMON_CONF=' /etc/default/hostapd 2>/dev/null \
  || echo 'DAEMON_CONF="/etc/hostapd/hostapd.conf"' >>/etc/default/hostapd
sed -i 's|^#DAEMON_CONF=.*|DAEMON_CONF="/etc/hostapd/hostapd.conf"|' /etc/default/hostapd

# Point /etc/hostapd/hostapd.conf as the default config in hostapd main
grep -q '^DAEMON_CONF=' /etc/default/hostapd && \
  sed -i 's|^DAEMON_CONF=.*|DAEMON_CONF="/etc/hostapd/hostapd.conf"|' /etc/default/hostapd

echo "[*] dnsmasq config -> /etc/dnsmasq.d/climbingrobot.conf"
cat >/etc/dnsmasq.d/climbingrobot.conf <<EOF
interface=${AP_IFACE}
bind-interfaces
domain-needed
bogus-priv
dhcp-range=${AP_DHCP_START},${AP_DHCP_END},12h
dhcp-option=3,${AP_IP}
dhcp-option=6,${AP_IP}
EOF

# Stop and disable conflicting services
systemctl stop wpa_supplicant 2>/dev/null || true
systemctl disable wpa_supplicant 2>/dev/null || true
systemctl unmask hostapd || true
systemctl enable hostapd
systemctl enable dnsmasq

echo "[*] Optional: enable mDNS so climbing-robot.local resolves"
if ! command -v avahi-daemon >/dev/null; then
  apt-get install -y avahi-daemon
fi
systemctl enable avahi-daemon
sed -i 's/^#publish-addresses=yes/publish-addresses=yes/' /etc/avahi/avahi-daemon.conf 2>/dev/null || true

echo "[*] Restarting services"
systemctl restart dnsmasq
systemctl restart hostapd
systemctl restart avahi-daemon || true

sleep 1
echo "[*] Done. SSID=${AP_SSID}  Password=${AP_PASSWORD}  IP=${AP_IP}"
ip addr show "$AP_IFACE" | grep inet || true