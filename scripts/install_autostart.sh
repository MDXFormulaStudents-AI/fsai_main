#!/bin/bash
# MDX FSAI — one-time autostart installer
# Run once with: sudo bash scripts/install_autostart.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[1/4] Installing pcan module auto-load..."
cp "$SCRIPT_DIR/pcan.conf" /etc/modules-load.d/pcan.conf

echo "[2/4] Installing systemd services..."
cp "$SCRIPT_DIR/fsai-can.service"   /etc/systemd/system/fsai-can.service
cp "$SCRIPT_DIR/fsai-stack.service" /etc/systemd/system/fsai-stack.service

echo "[3/4] Making launcher script executable..."
chmod +x "$SCRIPT_DIR/start_fsai.sh"

echo "[4/4] Enabling services..."
systemctl daemon-reload
systemctl enable fsai-can.service
systemctl enable fsai-stack.service

echo ""
echo "Done. Services will start automatically on next boot."
echo "To start now without rebooting:"
echo "  sudo systemctl start fsai-can"
echo "  sudo systemctl start fsai-stack"
echo ""
echo "To watch live logs:"
echo "  journalctl -u fsai-stack -f"
