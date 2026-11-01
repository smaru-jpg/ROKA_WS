# ROKA section-1 IFF LED firmware

## Fixed hardware contract

- Controller: Arduino Nano compatible ATmega328P board
- LED: 16-pixel WS2812/NeoPixel, `NEO_GRB + NEO_KHZ800`
- Data: Nano D6 to NeoPixel DIN
- Serial: 115200 baud over the Nano USB serial adapter
- Commands: `E` all red, `F` all green, `B` green/red halves, `N` off

Connect the Nano and NeoPixel grounds. Do not power 16 pixels through a GPIO
pin. Use a suitable regulated 5 V supply for the pixels and connect its ground
to the Nano ground. Verify the installed strip's power rating before setting
full brightness.

## Compile and upload

Install the AVR core and library once:

```bash
arduino-cli core update-index
arduino-cli core install arduino:avr@1.8.8
arduino-cli lib install "Adafruit NeoPixel@1.15.5"
```

Compile:

```bash
cd ~/roka_ws
arduino-cli compile --fqbn arduino:avr:nano \
  src/mission_vision/firmware/roka_iff_neopixel
```

After connecting the Nano, find its stable path and upload:

```bash
ls -l /dev/serial/by-id/
LED_PORT=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0
arduino-cli upload --port "$LED_PORT" --fqbn arduino:avr:nano \
  src/mission_vision/firmware/roka_iff_neopixel
```

If a clone uses the old Nano bootloader, compile and upload with
`--board-options cpu=atmega328old`. Do not guess this option; use the setting
that matches the actual Nano.

## Standalone protocol check

Stop section 1 before opening the same serial port. Opening a Nano serial port
can reset the board, so wait two seconds before sending the first command.

```bash
LED_PORT=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0
stty -F "$LED_PORT" 115200 raw -echo
sleep 2
printf E >"$LED_PORT"  # red
printf F >"$LED_PORT"  # green
printf B >"$LED_PORT"  # half green / half red
printf N >"$LED_PORT"  # off
```

The port name above belongs to the recorded Deco PC adapter. On another PC or
after replacing the Nano/USB adapter, use the path actually shown under
`/dev/serial/by-id`.

After uploading, the drive-free integrated D435/IFF/LED test is:

```bash
roka vision spring-led
```
