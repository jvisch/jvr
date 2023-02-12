# Install of Raspberry Pi

## Install OS: Ubuntu server

ROS is only supported on Ubuntu, so we need to install Ubuntu and not
the Raspberry Pi OS.

For the install the Raspberry Pi Imager is used
<https://www.raspberrypi.com/software/>.\
![Raspberry Pi Imager](./img/rpi-imager.png)

This install is based on [ROS2
Humble](https://docs.ros.org/en/humble/index.html) and [Raspberry Pi \#
model
B+](https://www.raspberrypi.com/products/raspberry-pi-3-model-b-plus/)

1.  Choose Storage

    Check the system requirements of the ROS-distribution you're want to
    use and the version of the Raspberry Pi it will be installed on.

    'Other general-purpose OS' -\> 'Ubuntu' -\> 'Ubuntu Server 22.04.1
    LTS (64-bit)'

    ![Other general-purpose OS](./img/choose-os-1.png)
    ![Ubuntu](./img/choose-os-2.png) ![Ubuntu Server 22.04.1 LTS
    (64-bit)](./img/choose-os-3.png)

2.  Choose Storage

    Select the SD-card on which the OS must be installed.

3.  Set advanced Options

    -   Set hostname: `my-robot` (or any name you want)

        This will be the name of the robot on the network. If you use
        more robots, make sure the names are unique within your network.

    -   Enable SSH: `check`

        I'm not using a desktop-like interface on the pi (for
        performance reasons), to connect to the pi SSH is used.

        -   Password
            -   Set user: `user-name` ([do not use
                `pi`](https://www.raspberrypi.com/news/raspberry-pi-bullseye-update-april-2022/))
            -   Set password: `*********`
        -   or; authentication key

    -   Configure wireless LAN: `check`

        I'm not using ethernet cables. Using WiFi with ROS is a bit
        tricky and unstable, but I'm trying to solve that. The robot
        must be able to drive around *on it's own*.

        -   SSID: `your wifi ssid`
        -   Password: `******`
        -   Wireless LAN country: `your country`

    -   Locale settings

        These will be set during the ROS install and setup.

Hit *Write* to write the image to the SD-card. After the write, put the
SD-card in the Raspberry Pi and start it.

## Configure Robot

### login with SSH

1.  Find the IP of the Raspberry Pi
    -   On your router (log in and find it)
    -   using `arp`:
        -   Windows `arp -a | findstr b8-27-eb`
        -   Linux: `arp -na | grep -i b8:27:eb`
2.  log in on the RPi with ssh `ssh <username>@<ip-of-rpi>`

### Update RPi

On the RPi:

1.  `sudo apt update`

2.  `sudo apt full-upgrade`

    This will take a while, follow instructions (if given).

3.  `sudo reboot now`

    Always reboot (just to make sure all updates and configurations are
    finished).

### Enable firewall (of course)

!! First add SSH to the firewall rules, otherwise the SSH connection
will be terminated and you will no longer be able to connect via SSH.

1.  `sudo ufw allow ssh`
2.  `sudo ufw enable`

### Stop unattended upgrades (optional)

The [unattended upgrades
package](https://packages.ubuntu.com/jammy/unattended-upgrades) upgrades
the OS with security issues on startup or on time interval. This can be
very slow in the RPi, especially if the RPi is rebooted frequently. This
can be disabled (but you have check for updates frequently).

1.  `sudo dpkg-reconfigure unattended-upgrades`

    select *no* for disabling autostart unattended-upgrades on booting.

2.  `sudo systemctl stop unattended-upgrades`

3.  `sudo apt remove unattended-upgrades`

### Make RPi discoverable by hostname on network (optional)

Local networks usually uses DHCP for handing out IP's. With
[Samba](https://packages.ubuntu.com/jammy/samba) the RPi can be discored
by hostname. RPi uses default Avahi, on my network this only works
stable if on the network card IPv6 is *disabled*. With Samba IPv6 seems
to work consistenly.

1.  `sudo apt install samba`
2.  `sudo ufw allow samba`
3.  `sudo service smbd restart` or reboot the RPi

## Install ROS (ROS2 Humble) on Robot

1.  Follow instructions [Installation Ubuntu
    (Debian)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

2.  Install `ros-humble-ros-base` on the RPi (not the *desktop*) on the
    RPi.

3.  Install also `sudo apt install ros-dev-tools` for building the
    JVR-packages.

4.  Add ROS source to shell startup script

    `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`

## Configure firewall on Robot

1.  `sudo ufw allow in proto udp to 224.0.0.0/4`
2.  `sudo ufw allow in proto udp from 224.0.0.0/4`
3.  `sudo ufw allow 7400:7430/tcp`
4.  `sudo ufw allow 7400:7430/udp`
5.  `sudo ufw allow 11811`

-   stap 1 t/m 4 zijn voor ROS (detectie op netwerk)
-   stap 5 is voor Fast DDS Discory server

## Pyton packages

- `pip install adafruit-circuitpython-pca9685`
- `pip install Flask`


```{=html}
<!-- ## Notes

-   GPIO (pinnen gebruiken)

    -   installeer `sudo apt install python3-rpi.gpio`
    -   geef de gebruiker rechten op GPIO
        -   groep gpio toevoegen `sudo addgroup gpio`
        -   gebruiker toevoegen `sudo usermod -a -G gpio $USER`

-   Op windows ook firewall instellen dat Ros er bij kan:

    1.  op windows defender
    2.  inbound rules -\> add new rule
    3.  select `port`
    4.  allow rang `7400-7429` (dat is voor domain id 1)

-   /dev/i2c/ is niet te benaderen voor ander gebruikers, doe:

    -   `sudo apt install i2c-tools`
    -   ~`sudo groupadd i2c (group may exist already)`~
    -   \~`sudo chown :i2c /dev/i2c-1` (or i2c-0)\~
    -   ~`sudo chmod g+rw /dev/i2c-1`~
    -   `sudo usermod -aG i2c $USER`
    -   `sudo reboot now` (or logout) -->
```
