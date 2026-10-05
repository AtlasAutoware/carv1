#!/usr/bin/env bash
# pilot_tunnel.sh — open the pilot page at http://localhost:8081/ through SSH (run on the laptop).
# Browsers only expose WebCodecs on secure pages (HTTPS or localhost). On plain http://10.42.0.1:8080/
# the page cannot decode the camera's H.264 and falls back to MJPEG; through this tunnel it can.
# IPQoS=ef puts the laptop->car side (driving commands) in the Wi-Fi video queue instead of the
# background queue that OpenSSH uses for non-interactive sessions.
#   tools/pilot_tunnel.sh [car]        default car: 10.42.0.1 (AtlasCar hotspot); 192.168.55.1 over USB-C
CAR="${1:-10.42.0.1}"
PORT=8081
if ! curl -s -m 2 -o /dev/null "http://localhost:$PORT/status"; then
    ssh -f -N -o BatchMode=yes -o ExitOnForwardFailure=yes -o ServerAliveInterval=2 \
        -o ServerAliveCountMax=3 -o IPQoS=ef -L "$PORT:127.0.0.1:8080" "atlas@$CAR" || exit 1
fi
echo "pilot page: http://localhost:$PORT/"
xdg-open "http://localhost:$PORT/" >/dev/null 2>&1 || true
