#!/usr/bin/python
import usb.core
import usb.util

def is_our_WGS(tdev):
   if tdev.idVendor != 0x16c0:
      return False
   if tdev.idProduct != 0x05e4:
     return False
   ldev=usb.legacy.Device(tdev)
   dh=ldev.open()
   name=u'WZab Digital Wireless Guitar System 1'
   version=0x0001
   if tdev.iProduct:
     dname=dh.getString(tdev.iProduct,255,0)
   else:
     dname=""
   dver=tdev.bcdDevice
   return (dver == version and \
           dname == name)

# We assume, that our device is the first one with 
# the VUSB shared idVendor and idProdect, 
# and with proper name and version
devs = usb.core.find(find_all=True, custom_match=is_our_WGS)
for dev in devs:
    #Configuration of switches
    # Our control message transfer must have the following form
    # 1st argument - bmRequestType - 0x40
    # 2nd argument - bRequest 
    #   - 0x8n to configure packet sent when n-th switch gets pressed
    #   - 0xcn to configure packet sent when n-th switch gest released
    # 3rd argument - wIndex - first 2 bytes of packet 0xB0B1 (B0-first byte, B1-second byte)
    # 4th argument - wValue - next 2 bytes of packet 0xB2B3 (B2-third byte, B3-fourth byte)
    # The example below configures switches 0 and 1 to send program changes 1 and 2 when pressed
    dev.ctrl_transfer(0x40,0x80,0x0cc0,0x0100)
    dev.ctrl_transfer(0x40,0x81,0x0cc0,0x0200)
    # Two transfers below configure switches 3 and 4 to emulate the control, which can be used
    # e.g. to control "looper" in guitarix (the looper starts recording when you press the button 3
    # and stops recording, when you press the button 4)
    dev.ctrl_transfer(0x40,0x83,0x0bb0,0x507f)
    dev.ctrl_transfer(0x40,0x84,0x0bb0,0x5000)
    # Two transfers below configure potentiometer 0 to work as master volume controller 
    dev.ctrl_transfer(0x40,0xb3,0x0bb0,0x0700)
 f len(devs)==0:
  print "Device not found!"

