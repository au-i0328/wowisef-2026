#!/usr/bin/env bash
# pi_tailscale_setup.sh
# ----------------------------------------------------------------
# Install Tailscale on the Pi and bring it up on your tailnet.
#
# Headless-Pi flow: you generate an auth key in the Tailscale admin
# console first, then pass it here. The script runs `tailscale up`
# non-interactively so no browser is needed on the Pi.
#
# Steps (one-time):
#   1. On a machine with a browser, go to https://login.tailscale.com/admin
#      Settings -> Keys -> Generate auth key.
#        - Reusable: ON  (so other machines can reuse the same key flow)
#        - Expiry: 90 days  (long enough not to babysit, short enough
#                             to rotate on a calendar reminder)
#        - Tags: leave empty (or add tag:robot if you use ACLs)
#      Copy the key (starts with tskey-auth-...).
#
#   2. On your laptop:
#        brew install tailscale   # macOS via Homebrew
#        sudo tailscale login     # signs into the same tailnet
#        tailscale status         # confirm your Mac is listed
#
#   3. On the Pi:
#        sudo TAILSCALE_AUTHKEY=tskey-auth-... \
#             bash pi_tailscale_setup.sh
#
#   4. Verify:
#        tailscale status         # from your Mac -- Pi should appear
#        ping -c 3 <pi-tailnet-ip> # should be <50 ms over home WiFi
#
# After this:
#   * The Pi gets a stable 100.x.y.z (visible via `tailscale ip` on the
#     Pi) and the dashboard can reach it from anywhere on the tailnet:
#       python3 mac_dashboard.py --host 100.x.y.z
#   * For the home-network validation you specifically asked for, no
#     further networking is needed. When we add LTE later, this script
#     also accepts `--advertise-routes 192.168.4.0/24` so the Pi can
#     act as a gateway for the AP clients; that's NOT enabled by
#     default here.
# ----------------------------------------------------------------
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
RUN_USER="${SUDO_USER:-pi}"

# --no-tailscale early-exit: this must come BEFORE the root check so
# the operator can dry-run it without sudo. It documents the
# operator's intended workflow and skips any apt or `tailscale up`
# work. Mirrors the `--no-tailscale` flag on each Python program: the
# operator takes responsibility for the network path themselves
# (usually the home AP or a directly-trusted LAN).
if [[ "${1:-}" == "--no-tailscale" || "${NO_TAILSCALE:-0}" == "1" ]]; then
  echo "[*] --no-tailscale set: Tailscale will NOT be installed or started"
  echo "    The Pi will be reachable only over its existing interfaces"
  echo "    (the ClimbingRobot AP at 192.168.4.1, or its home-WiFi IP)."
  echo
  echo "    The Python programs all accept --no-tailscale too:"
  echo "      python3 mac_dashboard.py --host 192.168.4.1 --no-tailscale \\"
  echo "                              --auth-token \"\"   # skipped"
  echo "      python3 gamepad_to_pi.py --host 192.168.4.1 --no-tailscale"
  echo "      sudo python3 pi_serial_bridge.py    --no-tailscale   # on the Pi"
  echo "      sudo python3 pi_video_stream.py     --no-tailscale   # on the Pi"
  echo
  exit 0
fi

if [[ $EUID -ne 0 ]]; then
  echo "Re-run as root: sudo $0" >&2
  exit 1
fi

if [[ -z "${TAILSCALE_AUTHKEY:-}" ]]; then
  echo "ERROR: TAILSCALE_AUTHKEY env var is empty." >&2
  echo "Generate a key at https://login.tailscale.com/admin/settings/keys" >&2
  echo "then re-run:  sudo TAILSCALE_AUTHKEY=tskey-auth-... bash $0" >&2
  exit 1
fi

echo "[*] Installing Tailscale (apt)"
# Tailscale ships its own apt repo; install it idempotently.
if ! command -v tailscale >/dev/null; then
  curl -fsSL https://tailscale.com/install.sh | sh
else
  echo "    tailscale already installed: $(tailscale version | head -n1)"
fi

echo "[*] Ensuring tailscaled is enabled"
systemctl enable --now tailscaled
sleep 2  # let the daemon bind its socket

echo "[*] tailscale up (non-interactive)"
# We pre-flight the arg list once and capture the exact command, so
# that on failure the operator can copy-paste it into a shell and see
# the *real* Tailscale error rather than retyping. Also: if this Pi
# is already on a tailnet, `tailscale up` warns that the auth key
# is being ignored -- pass --force-reauth only when re-keying.
TAILSCALE_UP_ARGS=(
  --auth-key="${TAILSCALE_AUTHKEY}"
  --accept-routes
  # Disable Tailscale SSH on the robot Pi: it's not needed and it
  # shrinks the attack surface. `--ssh` is a *bare boolean* (the
  # value-style `--ssh=off` is a parse error in current Tailscale;
  # `--ssh=false` works too but spelling it out is verbose).
  --ssh=false
  # Skip Tailscale's automatic iptables/netfilter magic. Without this
  # Tailscale rewrites FORWARD/POSTROUTING rules and breaks the AP
  # stack (hostapd + dnsmasq). The bridge binds 0.0.0.0 anyway, so
  # we don't need Tailscale to do our firewalling for us.
  --netfilter-mode=off
  --operator="${RUN_USER}"
)
# If we appear to already be on a tailnet from a previous install,
# the auth-key would be ignored silently. Surface that before we
# hang waiting for a connect.
if tailscale status --json 2>/dev/null | python3 -c \
    "import sys,json; s=sys.stdin.read(); print(json.loads(s).get('BackendState',''))" \
    | grep -qE '^(Running|Starting)$'; then
  echo "    ! Pi already on a tailnet (BackendState Running/Starting)."
  echo "    If you intend to *rotate* the auth key, append --force-reauth:"
  echo "      tailscale up ${TAILSCALE_UP_ARGS[*]} --force-reauth"
fi

if ! tailscale up "${TAILSCALE_UP_ARGS[@]}"; then
  cat >&2 <<EOF
[!] tailscale up failed. The exact command was:
    tailscale up ${TAILSCALE_UP_ARGS[*]}
Re-run that manually on the Pi to see the full error.
EOF
  exit 1
fi

echo
echo "[*] Tailscale status"
tailscale status || true
echo
echo "[*] This Pi's tailnet IP(s):"
tailscale ip -4 || true

cat <<EOF

[ok] Next steps
  1. From your Mac, run:  tailscale status
     You should see this Pi listed with a 100.x.y.z address.
  2. From your Mac, run:  ping -c 3 100.x.y.z
     This is your home-network baseline latency. LTE will add ~50 ms.
  3. Generate a bearer token for the bridge:
        sudo bash ${REPO_DIR}/pi_auth_setup.sh
  4. Point the dashboard at the tailnet IP:
        python3 mac_dashboard.py --host 100.x.y.z

EOF