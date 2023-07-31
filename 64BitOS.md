# Pi Setup
## Flashing SD Card
Currently we are using the 64 Bit Raspberry Pi OS Lite May 2023 release with Debian version 11.

Flash the SD card as normal using the `rpi-imager`. I find it helpful to use the `rpi-imager` to enable `ssh` and setup my username and password on the pi
## Installing Kernel
We are using the Preempt RT kernel. The newest 64 bit raspberry pi Preempt RT kernel I could find is 5.15 from: https://github.com/kdoren/linux/releases/tag/rpi_5.15.65-rt49
The installation instructions are from: https://github.com/kdoren/linux/wiki/Installation-of-kernel-from-deb-package-(Raspberry-Pi-OS)

- Copy the `.deb` file onto the pi
- Go into su using `sudo su`
- On the pi run `apt install ./ linux-image-5.15.65-rt49-v8+_5.15.65-1_arm64.deb `
- On the pi run `KERN=5.15.65-rt49-v8+`
- Then run:
```
mkdir -p /boot/$KERN/o/
cp -d /usr/lib/linux-image-$KERN/overlays/* /boot/$KERN/o/
cp -dr /usr/lib/linux-image-$KERN/* /boot/$KERN/
[[ -d /usr/lib/linux-image-$KERN/broadcom ]] && cp -d /usr/lib/linux-image-$KERN/broadcom/* /boot/$KERN/
touch /boot/$KERN/o/README
mv /boot/vmlinuz-$KERN /boot/$KERN/
mv /boot/initrd.img-$KERN /boot/$KERN/
mv /boot/System.map-$KERN /boot/$KERN/
cp /boot/config-$KERN /boot/$KERN/
cat >> /boot/config.txt << EOF

[all]
kernel=vmlinuz-$KERN
# initramfs initrd.img-$KERN
os_prefix=$KERN/
overlay_prefix=o/$(if [[ "$KERN" =~ 'v8' ]]; then echo -e "\narm_64bit=1"; fi)
[all]
EOF
```
- Check to make sure everything is all set by rebooting the pi and running `uname -a
`. You should expect the kernel version to be 5.15 with Preempt RT

## Installing the kodlab mjbots sdk
- scp `utils/performace_governer.sh`, `utils/setup-system.py`, and `utils/set-wifi.py`
- Ensure the pi is connected to the WWW and run `sudo python3 setup-system.py`. This will install a number of dependencies for the kodlab mjbots sdk.
- To change the access point created by the PI run `sudo python3 setup-wifi.py` and enter your desired SSID, wifi password, and IP address

# Laptop Setup
Modified from: https://github.com/abhiTronix/raspberry-pi-cross-compilers/wiki/Cross-Compiler-CMake-Usage-Guide-with-rsynced-Raspberry-Pi-64-bit-OS#cross-compiler-cmake-usage-guide-with-rsynced-raspberry-pi-64-bit-os
## Toolchain
- Download the toolchain from download from: https://sourceforge.net/projects/raspberry-pi-cross-compilers/files/Bonus%20Raspberr[…]2064-Bit%20Cross-Compiler%20Toolchains/Bullseye/GCC%2010.2.0/
- Extract the toolchain into `\opt\`
- Check that `/opt/cross-pi-gcc-10.2.0-64/` exists
- Update your `~/.bashrc` with: `export PATH=$PATH:/opt/cross-pi-gcc-10.2.0-64/bin`

## rootfs
- Create the folder `$HOME/raspberrypi/rootfs`
- rsync the rootfs onto your computer with `rsync -avr --delete pi@<ip>:/{lib,usr,opt} $HOME/raspberrypi/rootfs`
- Fix the symlinks in the rootfs with 
```
wget https://raw.githubusercontent.com/abhiTronix/rpi_rootfs/master/scripts/sysroot-relativelinks.py
sudo chmod +x sysroot-relativelinks.py
./sysroot-relativelinks.py $HOME/raspberrypi/rootfs
```
- Anytime you change the rootfs you must re run the `sysroot-relativelinks.py`
- Lastly add the following to your `~/.bashrc`: `export RASPBERRY_VERSION=4`