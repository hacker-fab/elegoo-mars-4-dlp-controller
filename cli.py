import argparse
import RPi.GPIO as GPIO
import smbus  # I2C
import spidev  # SPI
from UV_projector.controller import DLPC1438, Mode

if __name__ == "__main__":
    parser = argparse.ArgumentParser("litho cli")
    parser.add_argument("filename")
    parser.add_argument("--brightness", type=int, default=100, choices=range(0, 101), metavar="0-100", help="UV LED brightness on a scale from [0, 100]")
    parser.add_argument("--exposure-frames", type=int, default=200, help="exposure duration in frames (~60fps)")
    parser.add_argument("-x", "--x-offset", type=int, default=0)
    parser.add_argument("-y", "--y-offset", type=int, default=0)
    args = parser.parse_args()

    LED_PWM = int(args.brightness * 1023 / 100)

    GPIO.setmode(GPIO.BCM)

    try:
        # Initialize I2C (SMBus) on channel 1
        i2c = smbus.SMBus(1)

        # Initialise SPI (bus 0, with CE0 as chip select pin)
        spi = spidev.SpiDev()
        spi.open(0, 0)
        spi.max_speed_hz = 125000000  # FPGA/DCLP1438 limit: 50 MB/s; 125MHz seems limit for Pi zero 1W
        spi.mode = 3

        DMD = DLPC1438(i2c, spi)

        # external print mode
        DMD.configure_external_print(LED_PWM=LED_PWM)
        DMD.switch_mode(Mode.EXTERNALPRINT)

        # intialise FPGA buffers to zero
        DMD.set_background(intensity=0, both_buffers=True)

        # do the exposure
        try:
            DMD.send_image_to_buffer(args.filename, args.x_offset, args.y_offset)
        except FileNotFoundError:
            print(f"Error: Image not found: {args.filename}")
            exit(1)
        except AssertionError as e:
            print(f"Error: Invalid image dimensions: {e}")
            exit(1)
        DMD.swap_buffer()
        DMD.expose_pattern(exposed_frames=args.exposure_frames)
        DMD.switch_mode(Mode.STANDBY)

    finally:
        GPIO.cleanup()
