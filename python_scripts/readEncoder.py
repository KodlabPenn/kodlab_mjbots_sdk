import time

import spidev

spi_ch = 1

# Enable SPI
spi = spidev.SpiDev(0, spi_ch)
spi.max_speed_hz = 1200000
spi.mode = 0b01
usleep = lambda x: time.sleep(x/1000000.0)

def read_enc():
    register_msg = [0xFF, 0xFF]
    error_msg = [0b01000000,0b00000001]
    spi.xfer2(register_msg)
    usleep(1)
    nop_msg = [0x00, 0x00]
    reply = spi.xfer2(nop_msg)

    angle = 0
    for n in reply:
        angle = (angle << 8) + n
    print(bin(angle).count("1"))
    angle = angle & 0x3FFF

    return angle/16384 * 360

try:
    while True:
        enc = read_enc()
        print("enc:", round(enc, 3))
        time.sleep(1)

finally:
    spi.close()
