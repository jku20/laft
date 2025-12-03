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


def trace_request_tx(size: int, ep_in, ep_out) -> list[list[bool]]:
    # send request
    req = trace_request(size)
    ep_out.write(req)

    # read response packets
    resp = []
    for i in range(0, (63 + size * NUM_TRACES // 8) // 64):
        resp += list(ep_in.read(64))

    out = []
    # unflatten the flattened list
    for i in range(size):
        b1 = i * 2
        b2 = i * 2 + 1

        bits = []
        for i in range(8):
            bits.append(resp[b1] & 1 << i)
        for i in range(8):
            bits.append(resp[b2] & 1 << i)

        out.append(bits)
    return out


def freq_request(freq: int) -> array:
    assert freq < (1 << 32)
    return array(
        "B",
        [
            0x02,
            0x00,
            0x00,
            0x00,
            0x000000FF & freq,
            (0x0000FF00 & freq) >> 8,
            (0x00FF0000 & freq) >> 16,
            (0xFF000000 & freq) >> 24,
        ],
    )


def freq_request_tx(freq: int, ep_in, ep_out):
    req = freq_request(freq)
    ep_out.write(req)


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

    parser.add_argument(
        "-f",
        "--frequency",
        type=int,
        help="the sampling frequency of the logic analyzer",
        required=False,
    )

    args = parser.parse_args()

    try:
        # Set up a USB Connection.
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

        # Set the frequency to listen at if it exist, else this will be just the default on the MCU
        if args.frequency:
            freq_request_tx(args.frequency, ep_in, ep_out)

        # Request the data from the trigger condition
        resp = trace_request_tx(args.size, ep_in, ep_out)
        print(resp)
    except usb.core.USBError as e:
        eprint(f"error: {e}")
        sys.exit(1)
    except RuntimeError as e:
        eprint(f"error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
