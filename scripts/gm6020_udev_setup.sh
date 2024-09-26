#!/bin/sh

echo "#udev id for gm6020
SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", ATTRS{serial}==\"0001\", SYMLINK+=\"gm6020\"" > /tmp/85-gm6020-udev.rules


sudo sh -c "cat /tmp/85-gm6020-udev.rules > /etc/udev/rules.d/85-gm6020-udev.rules"

sudo udevadm control --reload-rules
sudo adduser $USER dialout

echo ""
echo "gm6020 udev setup has finished."

