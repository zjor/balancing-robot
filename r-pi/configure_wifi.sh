#!/bin/bash

read -p "Enter image dir [/Volumes/boot]: " IMAGE_DIR
IMAGE_DIR=${IMAGE_DIR:-/Volumes/boot}

read -p "SSID: " SSID
read -p "Password: " PASSWORD

cd ${IMAGE_DIR}
set -x

touch ssh

:>wpa_supplicant.conf

cat <<EOT >> wpa_supplicant.conf
country=CZ
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
 
network={
    ssid="${SSID}"
    psk="${PASSWORD}"
}
EOT