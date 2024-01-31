# Teensy 4.0 - OV7670 3D Camera
# Notes

## CSI-2
- Camera Serial Interface 2
- Defines data transmission and control interface between peripheral (camera) and host processor.
- Focuses on protocol for tranferring image data and basic image sensor control through Camera Control Interface (CCI)

## DSI
- Define data transfer protocol and interface (display).

## D-PHY
- Defines transmission medium (electrical conductors), in/output ciruits and clocking mechanisms that captures bits from serial bit streams.
- ### Link Basics:
- Line: Wire connecting pin from peripheral to processor.
- Lane: A pair of wire that carries a signal and its inverse (to cancel out noise).
- Link: Connection between peripheral and processor containing a clock lane and at least one data lane.