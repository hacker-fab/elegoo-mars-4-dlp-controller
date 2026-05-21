#!/usr/bin/env python3
"""
DLPC6540 I2C smoke test for Raspberry Pi.

Sends "Get Controller Info" (Opcode 0x00, Destination 4) and decodes the
13-byte response. If the controller name comes back as readable ASCII,
I2C and the controller firmware are both alive.

Protocol facts from DLPU110B (DLPC6540 Programmer's Guide):

  Section 15.2 "I2C Target":
    - Default 8-bit write address = 0x34  -> 7-bit address = 0x1A
    - Read uses repeated-start: write opcode bytes, Sr, read N bytes

  Section 16.1 "Command Packet":
    Command Header byte:
      bit 0-2: Destination     (1 = common, 4 = system commands)
      bit 3  : Opcode length   (0 = 1 byte, 1 = 2 bytes)
      bit 4  : Length present  (data length field follows opcode)
      bit 5  : Checksum present
      bit 6  : Reply requested (write commands only)
      bit 7  : Read command    (1 = read, 0 = write)

    For "Get Controller Info" we want the simplest possible packet:
      header = 0b1000_0100 = 0x84   (read=1, dest=4)
      opcode = 0x00
    No length field, no checksum -> 2-byte write total.

  Section 16.2 "Response Packet":
    Response Header byte mirrors the command header but bit 7 means BUSY,
    not Read. If busy is set, the rest of the response is garbage and the
    host must retry. Section 16.5.2.

  Section 4.6 "HOST_IRQ/SYSTEM_BUSY":
    GPIO_58 on the DLPC6540 is open-drain; LOW = ready. The host SHOULD
    wait for LOW before sending the first command. This is optional but
    documented. Wire it to a Pi input with --busy-gpio if you have it.

Wiring (DLPC6540 pin ball numbers from datasheet Table 5-9):
    Pi SDA (GPIO2, pin 3)  <-> IIC0_SDA  (D29)
    Pi SCL (GPIO3, pin 5)  <-> IIC0_SCL  (E27)
    Pi GND                  -> board GND
    Pullups >= 1 kOhm to 3.3V on both lines (Pi internal 1.8k usually OK)
    Optional: any Pi GPIO   <- GPIO_58 (SYSTEM_BUSY) on the DLPC6540

Setup:
    sudo raspi-config            # Interface Options -> I2C -> Enable
    sudo apt install -y i2c-tools
    pip install smbus2 gpiozero
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    from smbus2 import SMBus, i2c_msg
except ImportError:
    sys.exit("pip install smbus2")

# --- Constants from DLPU110B ----------------------------------------------

DLPC_ADDR = 0x1A  # 7-bit; datasheet quotes 0x34 as 8-bit write

DEST_COMMON = 1
DEST_SYSTEM = 4

HDR_READ = 1 << 7
HDR_BUSY = 1 << 7  # same bit, different meaning in response
HDR_ERROR = 1 << 6
HDR_CHECKSUM_PRESENT = 1 << 5
HDR_LENGTH_PRESENT = 1 << 4
HDR_DEST_MASK = 0x07

# Opcodes used here. See Table 19-5 (Controller Info) and 19-4 (Mode).
OP_MODE = 0x00  # Destination 1: returns 1-byte mode info
OP_CONTROLLER_INFO = 0x00  # Destination 4: returns 13 bytes

# --- Bus operations -------------------------------------------------------


def scan(bus: SMBus) -> list[int]:
    """Probe every legal 7-bit address; return responders."""
    found = []
    for addr in range(0x03, 0x78):
        try:
            bus.write_quick(addr)
            found.append(addr)
        except OSError:
            pass
    return found


def send_read_command(
    bus: SMBus,
    addr: int,
    destination: int,
    opcode: int,
    response_len: int,
    *,
    inter_delay_s: float = 0.01,
    retries: int = 5,
) -> bytes:
    """Issue a read-style command and return the data bytes (header stripped).

    Uses a single i2c_rdwr() so the bus issues a repeated-START between
    write and read, which is what the protocol specifies. Retries if the
    response header's busy bit (bit 7) is set.
    """
    header = HDR_READ | (destination & HDR_DEST_MASK)
    write = i2c_msg.write(addr, [header, opcode])
    # +1 for the response header byte the controller prepends
    read = i2c_msg.read(addr, response_len + 1)

    last_resp_header: int | None = None
    for attempt in range(retries):
        bus.i2c_rdwr(write, read)
        resp = bytes(read)
        resp_header = resp[0]
        last_resp_header = resp_header

        if resp_header & HDR_BUSY:
            # Controller still processing; back off and retry the whole txn.
            time.sleep(inter_delay_s * (attempt + 1))
            continue

        if resp_header & HDR_ERROR:
            err_code = resp[1] if len(resp) > 1 else 0xFF
            raise RuntimeError(f"Controller returned error code {err_code} (see Table 16-5 in DLPU110B). Response header = 0x{resp_header:02X}")

        return resp[1:]

    raise TimeoutError(f"Controller stayed busy after {retries} retries (last response header = 0x{last_resp_header:02X})")


# --- Decoders -------------------------------------------------------------


def decode_controller_info(data: bytes) -> tuple[int, str]:
    """Table 19-5: bytes 0-3 = Controller ID (LE), bytes 4-12 = Name."""
    if len(data) < 13:
        raise ValueError(f"expected 13 bytes, got {len(data)}: {data.hex()}")
    controller_id = int.from_bytes(data[0:4], "little")
    name = data[4:13].rstrip(b"\x00").decode("ascii", errors="replace")
    return controller_id, name


def decode_mode(data: bytes) -> str:
    """Table 19-4: bit 0 = app mode (0 bootloader / 1 main), bit 1 = single/multi."""
    if not data:
        raise ValueError("empty response")
    b = data[0]
    app = "Main Application" if b & 0x01 else "Bootloader"
    cfg = "Multiple controllers" if b & 0x02 else "Single controller"
    return f"{app}, {cfg}"


# --- Optional: SYSTEM_BUSY GPIO wait --------------------------------------


def wait_busy_low(pin: int, timeout_s: float = 5.0) -> None:
    """Block until DLPC6540 GPIO_58 (open-drain, active-high BUSY) goes LOW."""
    try:
        from gpiozero import DigitalInputDevice
    except ImportError:
        sys.exit("pip install gpiozero (or omit --busy-gpio)")

    busy = DigitalInputDevice(pin, pull_up=True)  # external open-drain
    start = time.monotonic()
    while busy.value:
        if time.monotonic() - start > timeout_s:
            raise TimeoutError(f"SYSTEM_BUSY (GPIO_58) stuck HIGH for {timeout_s}s. Per DLPU110B 4.6 this means the controller boot is hung — check PWRGOOD, power rails, and Boot Hold Reason.")
        time.sleep(0.01)


# --- Main -----------------------------------------------------------------


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--bus", type=int, default=1, help="Pi I2C bus (default 1)")
    p.add_argument("--addr", type=lambda x: int(x, 0), default=DLPC_ADDR, help=f"7-bit slave address (default 0x{DLPC_ADDR:02X})")
    p.add_argument("--busy-gpio", type=int, default=None, help="Pi GPIO wired to DLPC6540 GPIO_58 SYSTEM_BUSY")
    p.add_argument("--no-scan", action="store_true", help="Skip the bus scan; go straight to command")
    args = p.parse_args()

    if args.busy_gpio is not None:
        print(f"Waiting for SYSTEM_BUSY low on Pi GPIO {args.busy_gpio}...")
        wait_busy_low(args.busy_gpio)
        print("  ready.")

    with SMBus(args.bus) as bus:
        if not args.no_scan:
            print(f"Scanning I2C bus {args.bus}...")
            devices = scan(bus)
            print("  found: " + (", ".join(f"0x{a:02X}" for a in devices) or "nothing"))
            if args.addr not in devices:
                print(f"  WARNING: 0x{args.addr:02X} did not ACK the scan.")
                print("  Check PWRGOOD, pullups, GND, and SDA/SCL not swapped.")
                # Continue anyway — write_quick is not perfectly reliable.

        print(f"\nGet Controller Info (opcode 0x00, dest 4) from 0x{args.addr:02X}:")
        data = send_read_command(bus, args.addr, DEST_SYSTEM, OP_CONTROLLER_INFO, 13)
        cid, name = decode_controller_info(data)
        print(f"  Controller ID: 0x{cid:08X}")
        print(f"  Name: {name!r}")
        print(f"  Raw: {data.hex()}")

        print("\nGet Mode (opcode 0x00, dest 1):")
        data = send_read_command(bus, args.addr, DEST_COMMON, OP_MODE, 1)
        print(f"  {decode_mode(data)}  (raw: 0x{data[0]:02X})")

    print("\nOK — I2C round-trip works end-to-end.")


if __name__ == "__main__":
    main()
