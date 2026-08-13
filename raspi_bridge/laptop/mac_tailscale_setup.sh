#!/usr/bin/env bash
# mac_tailscale_setup.sh
# ----------------------------------------------------------------
# Install Tailscale on a macOS laptop and pull the bridge bearer
# token from the Pi.
#
# This script is informational/automated; on macOS most of it is
# interactive (login requires a browser cookie). Use the App Store
# version if you want a tray icon, or the CLI if you want everything
# in a terminal.
#
# Steps:
#   1. Install Tailscale:
#        brew install --cask tailscale
#      or download from https://tailscale.com/download/mac
#      Then:  open -a Tailscale   # launches the menu-bar app
#             sudo tailscale login # or click "Log in..." in the menu
#
#   2. Confirm you're on the same tailnet as the Pi:
#        tailscale status
#        # Pi should be listed with a 100.x.y.z address
#
#   3. Pull the bridge bearer token from the Pi (run on the Mac):
#        mkdir -p ~/.config/climbingrobot
#        scp pi@<pi-lan-or-tailnet-ip>:/etc/climbingrobot/auth_token \
#           ~/.config/climbingrobot/auth_token
#        chmod 600 ~/.config/climbingrobot/auth_token
#
#   4. Verify the dashboard can connect over Tailscale:
#        export BRIDGE_AUTH_TOKEN="$(cat ~/.config/climbingrobot/auth_token)"
#        python3 mac_dashboard.py --host <pi-tailnet-ip> \
#                                --auth-token "$BRIDGE_AUTH_TOKEN"
#      Or just rely on the env var (script does this for you below).
#
# Why this isn't a single shell script: Homebrew casks need a UI
# session to install. This file documents the manual steps in one
# place so the README can just point here.
# ----------------------------------------------------------------
set -euo pipefail

if [[ "$(uname -s)" != "Darwin" ]]; then
  echo "This script is meant for macOS. On Linux: https://tailscale.com/download/linux" >&2
  exit 1
fi

# --no-tailscale early-exit: skip the Tailscale install/scptoken-pull
# flow and print the directly-trusted-LAN workflow. Mirrors the
# --no-tailscale flag on each Python program.
if [[ "${1:-}" == "--no-tailscale" || "${NO_TAILSCALE:-0}" == "1" ]]; then
  CONFIG_DIR="${HOME}/.config/climbingrobot"
  echo "[*] --no-tailscale set: Tailscale will NOT be installed on this Mac"
  echo "    The bridge auth token will NOT be pulled -- none is required when"
  echo "    running in --no-tailscale mode."
  echo
  echo "    Use this workflow instead:"
  echo "      python3 mac_dashboard.py  --host 192.168.4.1 --no-tailscale"
  echo "      python3 gamepad_to_pi.py --host 192.168.4.1 --no-tailscale"
  echo "    or, if the Pi has been joined to your LAN (no AP):"
  echo "      python3 mac_dashboard.py  --host <pi-lan-ip>  --no-tailscale"
  echo
  exit 0
fi

CONFIG_DIR="${HOME}/.config/climbingrobot"
mkdir -p "${CONFIG_DIR}"

echo "[*] Installing Tailscale via Homebrew"
if command -v brew >/dev/null; then
  brew install --cask tailscale || true
else
  echo "    ! Homebrew not found. Install from https://tailscale.com/download/mac instead."
fi

# Tailscale's macOS install runs as a privileged helper; the user
# normally logs in via the menu-bar app. CLI login requires `sudo`.
echo
echo "[*] Tailnet login"
echo "    Run one of:"
echo "      sudo tailscale login     # CLI; opens browser"
echo "    or click 'Log in...' in the Tailscale menu-bar app."
echo
echo "    After login, verify with:  tailscale status"

echo
echo "[*] Pulling the bridge bearer token from the Pi"
if [[ -z "${PI_HOST:-}" ]]; then
  read -r -p "    Pi hostname or IP (default: pi.local): " PI_HOST
  PI_HOST="${PI_HOST:-pi.local}"
fi

# scp preserves the file's mode on most systems but the receiving
# machine's umask can strip it -- chmod explicitly.
scp "${PI_HOST}:/etc/climbingrobot/auth_token" "${CONFIG_DIR}/auth_token"
chmod 600 "${CONFIG_DIR}/auth_token"

# Tell the user how to consume it.
cat <<EOF

[ok] Token saved to ${CONFIG_DIR}/auth_token

  Use it like this:
    export BRIDGE_AUTH_TOKEN="\$(cat ${CONFIG_DIR}/auth_token)"
    python3 mac_dashboard.py --host <pi-tailnet-ip>

  Or pass it inline:
    python3 mac_dashboard.py --host <pi-tailnet-ip> \\
        --auth-token "\$(cat ${CONFIG_DIR}/auth_token)"

  The token is also accepted by gamepad_to_pi.py via the same flag /
  env var, so the gamepad sender picks it up automatically.

EOF

echo "[*] Quick sanity check from your shell"
echo "    tailscale status                          # Pi on the tailnet"
echo "    ping -c 3 \$(tailscale ip -4 | head -n1)   # 100.x baseline"
echo "    curl -sI http://<pi-tailnet-ip>:8080/health   # video OK"