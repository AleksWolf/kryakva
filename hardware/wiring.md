# Electronics wiring

All electronics except the IMU and the PCB sit in the head and meet at the 40-pin pin header of the Radxa Zero 3W.
Diagram: [wiring.png](wiring.png), pin header viewed from the pin side. Colors in the diagram are the wire colors.

## Zero 3W pin header

| Pin | To | Wire color |
|---|---|---|
| 1 | IMU 3.3 V | red |
| 2 | amplifier VIN (5 V) | red |
| 3 | IMU SDA | blue |
| 4 | UBEC +5 (Zero power) | red |
| 5 | IMU SCL | green |
| 6 | UBEC − | black |
| 8 | UC-01 R | yellow |
| 10 | UC-01 T | white |
| 12 | amplifier BCLK (and microphone SCK) | purple |
| 13 | amplifier SD (enable) | gray |
| 14 | common ground | black |
| 17 | ToF VIN (3.3 V) and microphone VDD | red |
| 29 | ToF SCL | green |
| 31 | ToF SDA | blue |
| 35 | amplifier LRC (and microphone WS) | orange |
| 38 | microphone SD | gray |
| 40 | amplifier DIN | brown |

Red wires to pins 2 and 4 carry 5 V, to pins 1 and 17 carry 3.3 V. Feed the ToF only 3.3 V.

## Ground

The grounds of UC-01, ToF, IMU, microphone and amplifier are joined together and go to pin 14 by one wire.

## Modules

- **UC-01** (servo bus). Level switch at 3.3 V. Wires R, T, G are Dupont, to the UC-01 pin block and to the Zero pin header.
  Port 1 is the head servo chain. Port 2 is power from the PCB (cable No. 12 from [cables.md](cables.md)). While UC-01 works with the Zero, USB-C is free.
- **UBEC 5 V / 3 A** (Zero power). Input IN+ and IN− goes into the UC-01 power screw terminal (6–15 V). Output: +5 → pin 4, − → pin 6.
- **IMU LSM6DSV16X** sits in the trunk, on `power_support`; its wires run through the neck. Jumpers on the module: AD0 → GND, CS → VCC.
  Colors: VCC red, GND black, SDA blue, SCL green.
- **ToF VL53L8CX**. The jumpers are required: SPI_I2C_N and NCS to each other and to GND. With them the module is tested on the duck. MISO, INT, LPn stay free.
- **I²S microphone**. Jumper L/R → GND (left channel). SCK and WS are soldered to BCLK and LRC of the amplifier on the back side of its board; VDD to VIN of the ToF module.
- **Amplifier MAX98357A**. The speaker wires are soldered to the amplifier board. GAIN stays free. SD goes to pin 13: the `duck-amp` service turns the amplifier on only while a sound plays ([software/zero_setup.md](../software/zero_setup.md)).
- **Camera**: ribbon cable into the Zero camera connector, bypassing the pin header.

## Sensor orientation

Seen from the duck: the 8×8 ToF frame is rotated 270° clockwise, the camera frame 90° clockwise, no mirroring.
IMU: the sensor X axis points left, Y up, Z forward.

## Soldering

- **PCB.** 5 AMP 2.0-3P headers, open side toward the front of the duck. To the BAT− and BAT+ pads: 16 (18) AWG wires from the battery contacts
  on power_support. The cable with a connector on one end: power and ground wires to the neck power pads.
- **UC-01.** Desolder the factory servo connectors and the straight pin header; solder in the right-angle pin header and two AMP 2.0-3P headers.

## Servo power

NP-F550 battery → PCB ([power_board/README.md](power_board/README.md)) → two leg chains and, through cable No. 13, the head chain.
