# Balancing Robot

## Configuring headless Pi

1. Connect SD card to USB
2. `touch ssh` in the root directory
3. Create `wpa_supplicant.conf` file in the root directory with the following content

```
country=CZ
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
 
network={
    ssid="SSID"
    psk="PASSWORD"
}

```

4. Plug SD card to Pi and boot, after some time connect using
- `ssh pi@raspberrypi.local`
- `ssh pi@rpi-zero-a.local`
- `ssh pi@rpi-zero-b.local`

default password: raspberry

## Shutting down R Pi

`sudo shutdown -h now`

