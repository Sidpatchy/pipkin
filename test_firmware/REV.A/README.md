# Pipkin hardware diagnostic firmware

PlatformIO/Arduino test project for Pipkin REV.A with the XIAO nRF54L15 and
E28-2G4M27SX (SX1281) radio module.

## What it does

- Blinks the XIAO's active-low built-in LED as a liveness indicator.
- Resets and checks the SX1281 over SPI, including standby and packet-type
  commands. It does not start RX or transmit RF.
- Scans J4's external I2C bus at 100 kHz and reports acknowledged addresses.
  Use `Wire1` for J4; the standard `Wire` pins conflict with the radio.

The schematic shows no onboard I2C peripherals. An empty scan is expected
unless a device is connected to J4. The optional 4.7 kΩ pull-ups R17/R18 are
marked DNP, so the external bus must provide pull-ups.

## Pin mapping

| Signal | XIAO pin |
| --- | --- |
| SX1281 NSS | D4 |
| SX1281 RESET | D2 |
| SX1281 BUSY | D3 |
| SX1281 DIO1 | D5 |
| SPI SCK / MISO / MOSI | D8 / D9 / D10 |
| J4 SCL / SDA (`Wire1`) | D11 / D12 |
| RF switch RX_EN / TX_EN | D0 / D1, held low |

## Build, flash, and monitor

From this directory:

```sh
pio run
pio run -t upload
pio device monitor
```

The target uses the `xiao_nrf54l15` board definition and nRF54 Arduino core
v1.0.23. Serial monitor speed is 115200 baud.
