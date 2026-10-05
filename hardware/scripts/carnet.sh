#!/usr/bin/env bash
# carnet.sh — pick how the car gets on a network (run on the Jetson, needs sudo).
#
#   carnet.sh status                      both adapters: what each is on, signal, IPs
#   carnet.sh hotspot                     be the AP "AtlasCar", car = 10.42.0.1. On the USB TP-Link T3U when it is
#                                         plugged in: 2.4 GHz ch 6, 20 MHz, WPA3-SAE (long range). Without the T3U:
#                                         the onboard card, 5 GHz ch 149, WPA2 (short range).
#   carnet.sh hotspot5                    the T3U hotspot on 5 GHz ch 149 instead (more bandwidth, about half the range)
#   carnet.sh hotspot24                   same as hotspot (kept so older notes still work)
#   carnet.sh client <SSID> [password]    join an existing WiFi / mesh (eero, Orbi, Deco, ...) on the onboard card; the
#                                         car roams between mesh nodes, so range = the mesh's footprint. Find the car
#                                         with `carnet.sh status` or http://ubuntu.local:8080/ (mDNS).
#   carnet.sh home <SSID> <password> [ip]   permanent home router on the onboard card: fixed IP (default 192.168.0.250)
#   carnet.sh tether                      use a phone plugged in over USB as the uplink (phone: enable USB tethering),
#                                         for the cellular/Tailscale path (see docs/REMOTE.md)
#
# Two radios since 2026-10-05: the T3U (driver: hardware/scripts/install_t3u_driver.sh) hosts the hotspot, so the
# onboard card is free to join a router or mesh for internet at the same time. Without the T3U the onboard card is
# the only radio and hotspot/client modes exclude each other. Channel 36 is not used any more: the onboard card boots
# with no country set and marks 36 "No IR", so a 5 GHz hotspot there never started; 149 is allowed either way.
set -e
ONBOARD=wlP1p1s0
t3u_if() {   # the T3U's interface (rtw_8822bu from lwfinger/rtw88, or the in-tree rtw88_8822bu)
    for n in /sys/class/net/*; do
        d=$(readlink -f "$n/device/driver" 2>/dev/null)
        case "${d##*/}" in rtw_8822bu|rtw88_8822bu) echo "${n##*/}"; return;; esac
    done
}
T3U=$(t3u_if)
IF=$ONBOARD       # station modes (client/home) always use the onboard card
cmd="${1:-status}"

need_root() { [ "$(id -u)" = 0 ] || exec sudo -E "$0" "$@"; }

mk_hotspot() {   # name ifname band channel key-mgmt pmf
    if ! nmcli -t -f NAME connection show | grep -qx "$1"; then
        nmcli connection add type wifi ifname "$2" con-name "$1" ssid AtlasCar mode ap \
            802-11-wireless.band "$3" 802-11-wireless.channel "$4" ipv4.method shared ipv6.method disabled \
            wifi-sec.key-mgmt "$5" wifi-sec.pmf "$6" wifi-sec.proto rsn wifi-sec.pairwise ccmp wifi-sec.group ccmp \
            wifi-sec.psk "${ATLASCAR_PSK:?set ATLASCAR_PSK=<password> the first time}" \
            connection.autoconnect yes connection.autoconnect-priority 10 >/dev/null
    fi
}

case "$cmd" in
  status)
    for i in $T3U $ONBOARD; do
        nmcli -t -f DEVICE,STATE,CONNECTION device | grep "^$i:" || true
        iw dev "$i" info 2>/dev/null | grep -E "type|channel" | sed 's/^/   /' || true
    done
    [ -n "$T3U" ] && iw dev "$T3U" station dump 2>/dev/null | grep -E "Station|signal:|tx bitrate" | sed 's/^/   /'
    [ -z "$T3U" ] && echo "(no T3U plugged in)"
    nmcli -t -f ACTIVE,SSID,SIGNAL,CHAN,FREQ device wifi list ifname "$ONBOARD" 2>/dev/null | grep '^yes' || true
    echo "IPs: $(hostname -I)"
    ip route show default | head -1 || true
    ;;
  hotspot|hotspot24|hotspot5)
    need_root "$@"
    if [ -n "$T3U" ]; then
        band=bg; ch=6; [ "$cmd" = hotspot5 ] && { band=a; ch=149; }
        mk_hotspot AtlasCar "$T3U" "$band" "$ch" sae 3
        nmcli connection modify AtlasCar connection.interface-name "$T3U" 802-11-wireless.band "$band" \
            802-11-wireless.channel "$ch" connection.autoconnect yes connection.autoconnect-priority 10
        nmcli connection down AtlasCarOnboard >/dev/null 2>&1 || true
        nmcli connection up AtlasCar >/dev/null && echo "hotspot AtlasCar on the T3U ($T3U), band $band ch $ch: car = 10.42.0.1"
    else
        mk_hotspot AtlasCarOnboard "$ONBOARD" a 149 wpa-psk 0
        nmcli connection modify AtlasCarOnboard connection.interface-name "$ONBOARD" 802-11-wireless.band a \
            802-11-wireless.channel 149
        nmcli connection up AtlasCarOnboard >/dev/null && \
            echo "no T3U plugged in: hotspot AtlasCar on the onboard card, 5 GHz ch 149 (WPA2): car = 10.42.0.1"
    fi
    ;;
  client)
    need_root "$@"
    ssid="${2:?usage: carnet.sh client <SSID> [password]}"
    if [ -n "${3:-}" ]; then
        nmcli device wifi connect "$ssid" password "$3" ifname "$IF" >/dev/null
    else
        nmcli connection up "$ssid" >/dev/null || nmcli device wifi connect "$ssid" ifname "$IF" >/dev/null
    fi
    # low latency for teleop: no WiFi power save, roam aggressively between mesh nodes
    nmcli connection modify "$ssid" 802-11-wireless.powersave 2 connection.autoconnect-priority 20 2>/dev/null || true
    iw dev "$IF" set power_save off 2>/dev/null || true
    echo "joined $ssid: car IP $(hostname -I | awk '{print $1}')  (also http://ubuntu.local:8080/ via mDNS)"
    ;;
  home)
    # Permanent home network: a dedicated router (e.g. AtlasNet) on the onboard card. Fixed IP so the pilot page
    # has a stable URL; with the T3U plugged in the AtlasCar hotspot stays up on the T3U at the same time.
    #   carnet.sh home <SSID> <password> [ip]        default ip 192.168.0.250 (gateway/dns = x.x.x.1)
    need_root "$@"
    ssid="${2:?usage: carnet.sh home <SSID> <password> [ip]}"; pw="${3:?password}"; ip="${4:-192.168.0.250}"
    gw="${ip%.*}.1"
    nmcli connection delete "$ssid" >/dev/null 2>&1 || true
    join() {   # $1 = key-mgmt (sae for WPA3-Personal, wpa-psk for WPA2)
        nmcli connection delete "$ssid" >/dev/null 2>&1 || true
        nmcli connection add type wifi ifname "$IF" con-name "$ssid" ssid "$ssid" \
            wifi-sec.key-mgmt "$1" wifi-sec.psk "$pw" \
            connection.autoconnect yes connection.autoconnect-priority 20 \
            802-11-wireless.powersave 2 \
            ipv4.method manual ipv4.addresses "$ip/24" ipv4.gateway "$gw" ipv4.dns "$gw" >/dev/null
        nmcli connection up "$ssid" >/dev/null 2>&1
    }
    if join sae; then echo "joined $ssid (WPA3)"
    elif join wpa-psk; then echo "joined $ssid (WPA2)"
    else echo "could not join $ssid: check the password, or set the router to WPA2/WPA3 mixed"; exit 1; fi
    iw dev "$IF" set power_save off 2>/dev/null || true
    echo "car = $ip  (autoconnect priority 20)"
    echo "pilot page: http://$ip:8080/"
    ;;
  tether)
    need_root "$@"
    dev=$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="ethernet" && $1 ~ /^(usb|enx|enP.*u)/ {print $1; exit}')
    [ -n "$dev" ] || { echo "no USB-tethered phone found (enable USB tethering on the phone first)"; exit 1; }
    nmcli device connect "$dev" >/dev/null && echo "tethered via $dev: $(ip -4 addr show "$dev" | grep -oE 'inet [0-9.]+')"
    ;;
  *) sed -n '2,20p' "$0"; exit 1 ;;
esac
