#!/usr/bin/env bash
# install_t3u_driver.sh — driver for the TP-Link Archer T3U (RTL8812BU) on the Jetson (run on the car).
# NVIDIA's L4T kernel (JetPack 7.2.1, 6.8.12-tegra) is built without rtw88, so the T3U has no driver
# out of the box. This builds rtw88 from github.com/lwfinger/rtw88 (tested at a56bcd2) against the
# running kernel's headers and installs only the four modules the T3U needs (needs sudo).
# Rerun after any kernel update (JetPack upgrade): the modules are tied to the kernel version.
#   1. get the source (the car has no internet; from the laptop):
#        git clone --depth 1 https://github.com/lwfinger/rtw88
#        rsync -a --exclude .git rtw88/ atlas@192.168.55.1:~/rtw88-lwfinger/
#   2. on the car:  bash hardware/scripts/install_t3u_driver.sh [~/rtw88-lwfinger]
#   3. then:        carnet.sh hotspot      (puts AtlasCar on the T3U)
set -e
SRC="${1:-$HOME/rtw88-lwfinger}"
KVER=$(uname -r)
[ -f "$SRC/Makefile" ] || { echo "no rtw88 source at $SRC (see the header of this script)"; exit 1; }
echo "building rtw88 for $KVER (about 40 s)..."
make -C "$SRC" -j"$(nproc)" > /tmp/rtw88_build.log 2>&1 || { tail -20 /tmp/rtw88_build.log; exit 1; }
sudo install -d "/lib/modules/$KVER/updates/rtw88"
sudo install -m644 "$SRC"/rtw_core.ko "$SRC"/rtw_usb.ko "$SRC"/rtw_8822b.ko "$SRC"/rtw_8822bu.ko \
    "/lib/modules/$KVER/updates/rtw88/"
sudo depmod -a
# This kernel cannot read compressed firmware, and linux-firmware ships rtw8822b_fw.bin.zst only.
sudo install -m644 "$SRC/firmware/rtw8822b_fw.bin" /lib/firmware/rtw88/rtw8822b_fw.bin
sudo tee /etc/modprobe.d/atlas-t3u.conf > /dev/null <<'EOF'
# Atlas car: TP-Link Archer T3U (RTL8812BU) on rtw88 (lwfinger/rtw88).
# Stay in USB 2 mode: USB 3 radiates noise into the 2.4 GHz band the car link uses.
options rtw_usb switch_usb_mode=n
# US Wi-Fi rules from boot (FCC channel set and transmit-power tables).
options cfg80211 ieee80211_regdom=US
EOF
sudo iw reg set US
sudo modprobe rtw_8822bu
echo "T3U driver installed for $KVER. Next: carnet.sh hotspot"
