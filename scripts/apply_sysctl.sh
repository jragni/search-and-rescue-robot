#!/usr/bin/env bash
# Apply DDS network buffer tuning required for LiDAR visualization
# over rosbridge/Fast-DDS/Zenoh. Must be run on BOTH host and robot.
# Per README.md prereqs.
set -euo pipefail

sudo sysctl -w net.core.wmem_max=2147483647
sudo sysctl -w net.core.wmem_default=2147483647
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.rmem_default=2147483647

# Persist across reboots
SYSCTL_FILE=/etc/sysctl.d/60-ros2-net-buffers.conf
sudo tee "$SYSCTL_FILE" >/dev/null <<'EOF'
net.core.wmem_max=2147483647
net.core.wmem_default=2147483647
net.core.rmem_max=2147483647
net.core.rmem_default=2147483647
EOF

echo "Network buffers applied + persisted to $SYSCTL_FILE"
