#!/usr/bin/env bash
set -euo pipefail

# === USER SETTINGS (edit to taste) ============================================
SSID="ROSNet"            # Hotspot name
PASSWORD="ros2followme"  # >= 8 chars
BAND="bg"                # "bg" = 2.4GHz (most compatible) ; "a" = 5GHz
CHANNEL="6"              # 2.4GHz: 1/6/11 ; 5GHz: 36/40/44/48 usually safe
SUBNET_CIDR="10.42.0.1/24"  # Laptop/clients get 10.42.0.x via DHCP
CON_NAME="ros-ap"        # NetworkManager connection name
ROS_DOMAIN_ID_VAL="0"    # All devices must match for DDS discovery
RMW_IMPL="rmw_fastrtps_cpp"  # or "rmw_cyclonedds_cpp" if you prefer Cyclone
# ==============================================================================

echo "[*] Checking prerequisites…"
if ! command -v nmcli >/dev/null 2>&1; then
  echo "NetworkManager (nmcli) not found. Installing…"
  sudo apt-get update
  sudo apt-get install -y network-manager
fi

# Ensure NetworkManager manages wlan0 (Ubuntu 22.04 usually does by default)
echo "[*] Making sure NetworkManager manages Wi-Fi…"
sudo systemctl enable NetworkManager.service
sudo systemctl start NetworkManager.service

# Optional: Turn off Wi-Fi powersave for stability
echo "[*] Disabling Wi-Fi powersave…"
sudo mkdir -p /etc/NetworkManager/conf.d
sudo tee /etc/NetworkManager/conf.d/99-wifi-powersave.conf >/dev/null <<'EOF'
[connection]
wifi.powersave = 2   # 2 = disabled
EOF
sudo systemctl restart NetworkManager

# Clean up any old connection with same name
if nmcli -t -f NAME con show | grep -Fxq "${CON_NAME}"; then
  echo "[*] Removing existing NM connection '${CON_NAME}'…"
  sudo nmcli con delete "${CON_NAME}" || true
fi

# Create AP connection
echo "[*] Creating AP connection '${CON_NAME}' (SSID='${SSID}', band=${BAND}, channel=${CHANNEL})…"
sudo nmcli con add type wifi ifname wlan0 con-name "${CON_NAME}" ssid "${SSID}"
sudo nmcli con modify "${CON_NAME}" \
  802-11-wireless.mode ap \
  802-11-wireless.band "${BAND}" \
  802-11-wireless.channel "${CHANNEL}" \
  802-11-wireless.hidden no \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "${PASSWORD}" \
  ipv4.method shared \
  ipv6.method ignore

# Pin the hotspot IP (NetworkManager's shared mode uses 10.42.0.1 by default;
# we set it explicitly to be predictable across boots)
sudo nmcli con modify "${CON_NAME}" +ipv4.addresses "${SUBNET_CIDR}"

# Autoconnect on boot
sudo nmcli con modify "${CON_NAME}" connection.autoconnect yes

# Bring it up now (so you can test without reboot)
echo "[*] Bringing up hotspot now…"
sudo nmcli con up "${CON_NAME}"

# Small ROS 2 env profile for all shells (networking knobs)
echo "[*] Installing /etc/profile.d/ros2_network.sh …"
sudo tee /etc/profile.d/ros2_network.sh >/dev/null <<EOF
# ROS 2 networking defaults for hotspot use
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VAL}
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=${RMW_IMPL}
EOF

# Optional: firewall allowances for DDS/mDNS when UFW is enabled
if command -v ufw >/dev/null 2>&1; then
  echo "[*] UFW detected — allowing common ROS 2 discovery ports (optional)…"
  # mDNS
  sudo ufw allow 5353/udp comment 'mDNS for ROS 2'
  # Fast DDS discovery (7400-7500/udp). Adjust or comment out if using CycloneDDS.
  sudo ufw allow 7400:7500/udp comment 'FastDDS discovery'
fi

# Systemd unit to ensure AP is up at boot even if NetworkManager races
echo "[*] Creating systemd unit to force AP up after boot…"
sudo tee /etc/systemd/system/ros-ap.service >/dev/null <<EOF
[Unit]
Description=Bring up ${CON_NAME} Wi-Fi hotspot (NetworkManager)
After=network-online.target NetworkManager.service
Wants=network-online.target
Requires=NetworkManager.service

[Service]
Type=oneshot
ExecStart=/usr/bin/nmcli con up ${CON_NAME}
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable ros-ap.service

# Helpful printout
echo
echo "=== Hotspot configured ==============================================="
echo " SSID:     ${SSID}"
echo " Password: ${PASSWORD}"
echo " Band:     ${BAND}  Channel: ${CHANNEL}"
echo " Subnet:   ${SUBNET_CIDR}  (Pi will be gateway; clients ~10.42.0.x)"
echo " ROS 2:    ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VAL}, RMW=${RMW_IMPL}, ROS_LOCALHOST_ONLY=0"
echo " Service:  ros-ap.service (enabled at boot)"
echo "======================================================================"
echo
echo "Next steps:"
echo " 1) Reboot: sudo reboot"
echo " 2) Connect your laptop to SSID '${SSID}'."
echo " 3) Test: on Pi -> ros2 topic pub /chatter std_msgs/String \"data: 'hello'\" -r 1"
echo "           on laptop -> ros2 topic echo /chatter"
