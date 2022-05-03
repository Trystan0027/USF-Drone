from machine import Pin, SPI
import struct

from nrf24l01 import NRF24L01

led = Pin(25, Pin.OUT)                # LED
btn = Pin(28, Pin.IN, Pin.PULL_DOWN)  # button press
csn = Pin(15, mode=Pin.OUT, value=1)  # chip select not
ce  = Pin(14, mode=Pin.OUT, value=0)  # chip enable

pipes = (b"\xe1\xf0\xf0\xf0\xf0", b"\xd2\xf0\xf0\xf0\xf0")


def setup():
    spi = machine.SPI(0, baudrate=48000, polarity=0, phase=0, bits=8, sck=Pin(2), mosi=Pin(3), miso=Pin(4))
    print(SPI(0))
    nrf = NRF24L01(SPI(0), csn, ce, payload_size=4)
    print(SPI(0))
    nrf.open_tx_pipe(b"\xd2\xf0\xf0\xf0\xf0") # tx is transfer
    nrf.open_rx_pipe(1, b"\xe1\xf0\xf0\xf0\xf0")# rx is recieve

    led.value(0)
    return nrf
            
def auto_ack(nrf):
    nrf.reg_write(0x01, 0b11111000)  # enable auto-ack on all pipes


nrf = setup()
auto_ack(nrf)

while True:
    state = 0
    if state != btn.value():
        state = btn.value()
        led.value(state)
            
        print("tx", state)
        try:
            nrf.send(struct.pack("i", state))
        except OSError:
            print('message lost')
        state = 0
        led.value(state)

