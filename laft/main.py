import usb.core
import sys
import array


def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)


VID = 0x8F83
PID = 0x2309


def connect(vid: int, pid: int) -> usb.core.Device:
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        raise RuntimeError(f"device {vid:04x}:{pid:04x} not found.")
    return dev


def main():
    try:
        dev = connect(VID, PID)

        if not dev.get_active_configuration():
            dev.set_configuration()
        cfg = dev.get_active_configuration()

        intf = cfg[(0, 0)]
        ep_in = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress)
            == usb.util.ENDPOINT_IN,
        )
        ep_out = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress)
            == usb.util.ENDPOINT_OUT,
        )
        ep_out.write("wahh")
        s = ""
        for c in ep_in.read(64):
            s += chr(c)
        print(s)
    except usb.core.USBError as e:
        eprint(f"error: {e}")
        sys.exit(1)
    except RuntimeError as e:
        eprint(f"error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
