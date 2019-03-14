# Logitech R400

This is a driver for the Logitech R400 pointer.

I took some inspiration from this [link](http://www.orangecoat.com/how-to/read-and-decode-data-from-your-mouse-using-this-pyusb-hack)

## ROS communication

This node simply publishes a **~state** topic of the type **logitech_r400/LogitechR400.msg**.

## Install

### Proper pyusb version
*TODO use rosdep instead*
You'll need pyusb 1.0+ for this driver to work. To install it:

```
sudo pip install --upgrade --pre pyusb
```

### Config setup
First, find out pointer's vendor and product values. To do that, in the console run `dmesg -w` and plug in the pointer's dongle to the usb port. A message similar to the one below should appear:

```
[64802.439385] usb 1-5.3: new low-speed USB device number 21 using xhci_hcd
[64802.604109] usb 1-5.3: New USB device found, idVendor=2571, idProduct=4101
[64802.604116] usb 1-5.3: New USB device strings: Mfr=1, Product=2, SerialNumber=0
[64802.604120] usb 1-5.3: Product: HAS HS304
[64802.604124] usb 1-5.3: Manufacturer: HAS  
[64802.612420] input: HAS   HAS HS304 as /devices/pci0000:00/0000:00:14.0/usb1/1-5/1-5.3/1-5.3:1.0/0003:2571:4101.003A/input/input90
[64802.671856] hid-generic 0003:2571:4101.003A: input,hidraw1: USB HID v1.11 Keyboard [HAS   HAS HS304] on usb-0000:00:14.0-5.3/input0
[64802.675745] input: HAS   HAS HS304 as /devices/pci0000:00/0000:00:14.0/usb1/1-5/1-5.3/1-5.3:1.1/0003:2571:4101.003B/input/input91
[64802.735835] hid-generic 0003:2571:4101.003B: input,hidraw2: USB HID v1.11 Mouse [HAS   HAS HS304] on usb-0000:00:14.0-5.3/input1
```

In the example above, you can see there are `idVendor` and `idProduct` values present, these are the ones you're interested in.

Now, make sure you're in the `plugdev` group and create a udev file:

```
sudo nano /etc/udev/rules.d/89-logitech-r400.rules
```

Add the following udev rule to that file:

```
ATTRS{idProduct}=="<product_value>", ATTRS{idVendor}=="<vendor_value>", MODE="666", GROUP="plugdev"
```
where `<product_value>` and `<vendor_value>` are values found before.

Now, reload the rules without rebooting:
```
sudo udevadm trigger
```
Finally, go to `config.yaml` file in `config` directory and use previously found product and vendor values again. 

Information based on the following [wiki](http://www.tincantools.com/wiki/Accessing_Devices_without_Sudo).