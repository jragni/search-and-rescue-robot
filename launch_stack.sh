#!/usr/bin/env bash
# launch_stack.sh — Bring up the full Transbot telemetry + tunnel stack in one tmux session.
#
# Windows:
#   0 container   — runs docker_run.sh (keeps the `ros2` container alive)
#   1 bringup     — sensors/drivers/EKF/SLAM + rosbridge (inside container)
#   2 webrtc      — ros_webrtc camera signaling (inside container)
#   3 cloudflare  — quick tunnel exposing nginx:8000 to the internet
#
# Prereqs on host: docker (image ros_2_image built), nginx on :8000 (robot-teleop),
#                  cloudflared, tmux. rosbridge listens on 9090 inside container.
#
# Usage:  ./launch_stack.sh          # start (or attach if already running)
#         ./launch_stack.sh stop     # tear down session + container

set -euo pipefail

SESSION="transbot"
REPO_DIR="$HOME/search-and-rescue-robot"
CONTAINER="ros2"
CAMERA_TOPIC="/camera/color/image_raw"
TUNNEL_TARGET="http://127.0.0.1:8000"

# In-container repo path (matches docker_run.sh volume mount)
CTR_REPO="/home/ubuntu/search-and-rescue-robot"

# ── stop subcommand ──────────────────────────────────────────────────────────
if [[ "${1:-}" == "stop" ]]; then
    echo "Stopping stack..."
    docker stop "$CONTAINER" 2>/dev/null || true
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    echo "Done."
    exit 0
fi

# ── attach if already running ────────────────────────────────────────────────
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already exists — attaching."
    exec tmux attach-session -t "$SESSION"
fi

# ── helper: wait until the container is up and ros2 is callable ───────────────
wait_for_container() {
    echo "Waiting for container '$CONTAINER' to be ready..."
    for _ in $(seq 1 30); do
        if docker exec "$CONTAINER" bash -lc 'command -v ros2 >/dev/null 2>&1'; then
            echo "Container ready."
            return 0
        fi
        sleep 1
    done
    echo "WARNING: container not ready after 30s — windows may need a manual restart."
    return 1
}

# ── Window 0: container ───────────────────────────────────────────────────────
# docker_run.sh runs interactively (-it) and keeps the container alive while this
# pane lives. All other windows docker-exec into it.
tmux new-session -d -s "$SESSION" -n container \
    "cd '$REPO_DIR' && ./docker_run.sh"

wait_for_container || true

# ── Window 1: bringup (sensors + rosbridge) ──────────────────────────────────
tmux new-window -t "$SESSION" -n bringup \
    "docker exec -it $CONTAINER bash -lc 'cd $CTR_REPO && source ./onboard_source.sh && ros2 launch transbot_bringup bringup.launch.py'"

# Give bringup a head start so the camera topic exists before webrtc subscribes.
sleep 8

# ── Window 2: webrtc camera signaling ────────────────────────────────────────
tmux new-window -t "$SESSION" -n webrtc \
    "docker exec -it $CONTAINER bash -lc 'ros2 run ros_webrtc webrtc_server --ros-args -p image_topic:=$CAMERA_TOPIC'"

# ── Window 3: cloudflare quick tunnel ────────────────────────────────────────
tmux new-window -t "$SESSION" -n cloudflare \
    "cloudflared tunnel --url $TUNNEL_TARGET"

# ── Attach ───────────────────────────────────────────────────────────────────
# Land on the cloudflare window so the public URL is visible immediately.
tmux select-window -t "$SESSION:cloudflare"
exec tmux attach-session -t "$SESSION"
