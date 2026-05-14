#!/usr/bin/env python3
"""Stepper dance — drives the stage through a sequence of relative moves."""

import time
import serial
from stage import GrblStage


# PORT = "/dev/tty.usbserial-XXXX"  # change to your port (COM6 on Windows, /dev/ttyUSB0 on Linux)
PORT = "/dev/tty.usbmodem1401"
BAUD = 115200


def main():
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        stage = GrblStage(
            controller_target=ser,
            enable_homing=False,
            enable_tiling=False,
            autofocus_offset=0,
        )
        time.sleep(1.0)

        # all moves in microns (1000 µm = 1 mm)
        moves = [
            # square
            {"x": 2000, "y": 0},
            {"x": 0, "y": 2000},
            {"x": -2000, "y": 0},
            {"x": 0, "y": -2000},
            # diagonal X's
            {"x": 1500, "y": 1500},
            {"x": -1500, "y": -1500},
            {"x": 1500, "y": -1500},
            {"x": -1500, "y": 1500},
            # z bob
            {"z": 500},
            {"z": -500},
            {"z": 500},
            {"z": -500},
            # spiral-out
            {"x": 1000, "y": 0},
            {"x": 0, "y": 1000},
            {"x": -1500, "y": 0},
            {"x": 0, "y": -1500},
            {"x": 2000, "y": 0},
            {"x": 0, "y": 2000},
            {"x": -2000, "y": -2000},  # back-ish
        ]

        for i, m in enumerate(moves, 1):
            print(f"step {i}/{len(moves)}: {m}")
            stage.move_relative(m)
            time.sleep(0.3)


if __name__ == "__main__":
    main()
