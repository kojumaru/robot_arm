import usb.core, usb.util

dev = usb.core.find(idVendor=0x256f, idProduct=0xc635)
try:
    dev.set_configuration()
except:
    pass
print('SpaceMouse detected. Move it around...')
for i in range(500):
    try:
        data = dev.read(0x81, 16, timeout=100)
        if data and data[0] != 0:
            print(f'ID={data[0]} data={list(data[:8])}')
    except usb.core.USBError as e:
        if 'timeout' in str(e).lower():
            continue
        print(f'Error: {e}')
        break
