# How to fix usb names: udev rules

Explanations:
- Tutorial that does not work good but explains great: [https://medium.com/@darshankt/setting-up-the-udev-rules-for-connecting-multiple-external-devices-through-usb-in-linux-28c110cf9251](https://medium.com/@darshankt/setting-up-the-udev-rules-for-connecting-multiple-external-devices-through-usb-in-linux-28c110cf9251)
- This solution works better but is very short: https://askubuntu.com/questions/1021547/writing-udev-rule-for-usb-device

My explanation:

Create a rule file in udev:
```
cd /etc/udev/rules.d
sudo gedit ttyUSB.rules
```

I wrote this in mine (capital letters, spaces and `"` positions are important), modify it according to your needs:
```
SUBSYSTEM=="tty",SUBSYSTEMS=="usb",DRIVERS=="usb",ATTRS{serial}=="FT7W9D4H",SYMLINK+="ttyUSB_mz0"
SUBSYSTEM=="tty",SUBSYSTEMS=="usb",DRIVERS=="usb",ATTRS{serial}=="FT7W9E7B",SYMLINK+="ttyUSB_mz1"
SUBSYSTEM=="tty",SUBSYSTEMS=="usb",DRIVERS=="usb",ATTRS{serial}=="FT7W9D4A",SYMLINK+="ttyUSB_mz2"
SUBSYSTEM=="tty",SUBSYSTEMS=="usb",DRIVERS=="usb",ATTRS{serial}=="FT763GV5",SYMLINK+="ttyUSB_mz3"
```

Result: the usb device with the serial number `FT7W9D4H` will always be assigned to `/dev/ttyUSB_mz0` ...
