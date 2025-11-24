import argparse
from array import array
import usb.core
import sys

# constants for colored output
GRAY = "\033[0;30m"
CYAN = "\033[0;36m"
RED = "\033[1;31m"
BLUE = "\033[0;34m"
YELLOW = "\033[0;33m"
GREEN = "\033[1;32m"
ENDCOLOR = "\033[0m"


def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)


VID = 0x8F83
PID = 0x2309

NUM_TRACES = 16


def connect(vid: int, pid: int) -> usb.core.Device:
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        raise RuntimeError(f"device {vid:04x}:{pid:04x} not found.")
    return dev


def trace_request(size: int) -> array:
    return array("B", [0x01, (size & 0xFF00) >> 8, size & 0x00FF] + 5 * [0])


def main():
    parser = argparse.ArgumentParser(
        prog="laft",
        description="Interact with the logic analyzer built with a raspberry pi pico 2 and buddy boards",
        epilog=f"{RED}<3{ENDCOLOR}",
    )

    parser.add_argument(
        "-s",
        "--size",
        type=int,
        help="the number of bits to read from each trace",
        required=True,
    )

    args = parser.parse_args()

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

        req = trace_request(args.size)
        ep_out.write(req)
        resp = []
        for i in range(0, (63 + args.size * NUM_TRACES // 8) // 64):
            print("reading")
            print(ep_in.read(64))
    except usb.core.USBError as e:
        eprint(f"error: {e}")
        sys.exit(1)
    except RuntimeError as e:
        eprint(f"error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
