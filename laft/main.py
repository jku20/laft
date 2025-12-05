import argparse
import queue
import threading
from array import array
import usb.core
import sys

import dearpygui.dearpygui as dpg

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


def read_data_request(size: int, ep_in, timeout: int = 1000) -> list[list[int]]:
    # read response packets
    resp = []
    for i in range(0, (63 + size * NUM_TRACES // 8) // 64):
        resp += list(ep_in.read(64, timeout=timeout))

    out = [[] for i in range(16)]
    # unflatten the flattened list
    for i in range(size):
        b1 = i * 2
        b2 = i * 2 + 1

        for i in range(8):
            out[i].append(resp[b1] & 1 << i)
        for i in range(8):
            out[i + 8].append(resp[b2] & 1 << i)
    return out


def trace_request(size: int) -> array:
    return array("B", [0x01, 0x00, (size & 0xFF00) >> 8, size & 0x00FF] + 4 * [0])


def trace_request_tx(size: int, ep_in, ep_out) -> list[list[int]]:
    # send request
    req = trace_request(size)
    ep_out.write(req)

    return read_data_request(size, ep_in)


def rising_edge_trigger_request(size: int, pin: int) -> array:
    return array("B", [0x04, pin & 0x0F, size & 0xFF00, size & 0x00FF] + 4 * [0])


def rising_edge_trigger_request_tx(
    size: int, pin: int, ep_in, ep_out
) -> list[list[int]]:
    # Send req.
    req = rising_edge_trigger_request(size, pin)
    ep_out.write(req)

    return read_data_request(size, ep_in, timeout=10000)


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
        required=False,
    )

    parser.add_argument(
        "-f",
        "--frequency",
        type=int,
        help="set the sampling frequency of the logic analyzer",
        required=False,
    )

    parser.add_argument(
        "-p",
        "--pin",
        type=int,
        help="set the pin to trigger on",
        required=False,
    )

    parser.add_argument(
        "--headless",
        action="store_true",
        help="run send an individual request instead of running as a gui",
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
    except usb.core.USBError as e:
        eprint(f"error: {e}")
        sys.exit(1)
    except RuntimeError as e:
        eprint(f"error: {e}")
        sys.exit(1)

    if args.headless:
        # We a command line tool
        # Set the frequency to listen at if it exist, else this will be just the default on the MCU
        if args.frequency is not None:
            freq_request_tx(args.frequency, ep_in, ep_out)

        # Request the data from the trigger condition
        if args.pin is not None:
            resp = rising_edge_trigger_request_tx(args.size, args.pin, ep_in, ep_out)
        else:
            resp = trace_request_tx(args.size, ep_in, ep_out)

        print(resp)
    else:
        # We running the GUI
        dpg.create_context()
        dpg.create_viewport(title="laft", width=600, height=600)

        SAMPLES = 500
        EXPAND = 50
        update = [True]

        def toggle_update():
            update[0] = not update[0]
            if update[0]:
                dpg.set_item_label("update_button", "Stop Capture")
            else:
                dpg.set_item_label("update_button", "Start Capture")

        def update_data(resp):
            for i in range(16):
                xaxis = [j for j in range(SAMPLES * EXPAND)]
                yaxis = [0 for j in range(SAMPLES * EXPAND)]
                for j in range(SAMPLES):
                    for k in range(EXPAND):
                        yaxis[j * EXPAND + k] = resp[i][j]
                dpg.set_value(f"series_{i}", [xaxis, yaxis])

        with dpg.window(tag="laft"):
            dpg.add_button(
                label="Stop Capture", tag="update_button", callback=toggle_update
            )

            for i in range(16):
                # plot for a waveform
                with dpg.plot(
                    label=f"Waveform {i}",
                    height=50,
                    width=-1,
                    no_frame=True,
                    no_title=True,
                    no_mouse_pos=True,
                    no_menus=True,
                ):
                    dpg.add_plot_axis(
                        dpg.mvXAxis,
                        label="x",
                        no_label=True,
                        tag=f"xaxis_{i}",
                        no_highlight=True,
                        no_tick_marks=True,
                        no_tick_labels=True,
                    )
                    dpg.add_plot_axis(
                        dpg.mvYAxis,
                        tag=f"yaxis_{i}",
                        no_label=True,
                        no_highlight=True,
                        no_tick_labels=True,
                        no_tick_marks=True,
                        no_gridlines=True,
                        lock_min=True,
                        lock_max=True,
                    )
                    dpg.set_axis_limits_constraints(f"xaxis_{i}", 0, SAMPLES * EXPAND)

                    # series 1
                    dpg.add_line_series(
                        [j for j in range(SAMPLES * EXPAND)],
                        [0 for j in range(SAMPLES * EXPAND)],
                        parent=f"yaxis_{i}",
                        tag=f"series_{i}",
                        shaded=True,
                    )

        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("laft", True)

        dataq = queue.Queue()
        cmdq = queue.Queue()

        # The worker is controlled by commands passed by cmdq
        # 0 is shutdown, 1 is get data, 2 is wait
        def populate_q_worker():
            while True:
                cmd = cmdq.get()
                if cmd == 0:
                    break
                elif cmd == 1:
                    if dataq.empty():
                        print("capturing")
                        resp = trace_request_tx(SAMPLES, ep_in, ep_out)
                        dataq.put(resp)
                elif cmd == 2:
                    pass

        worker = threading.Thread(target=populate_q_worker, daemon=True)
        worker.start()
        while dpg.is_dearpygui_running():
            if update[0]:
                try:
                    resp = dataq.get_nowait()
                    update_data(resp)
                except queue.Empty:
                    cmdq.put(1)
            else:
                cmdq.put(2)
            dpg.render_dearpygui_frame()
        cmdq.put(0)
        worker.join()
        dpg.destroy_context()


if __name__ == "__main__":
    main()
