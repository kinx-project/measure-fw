HID processing latency measurement
==================================

This repository contains firmware for the [NXP FRDM-MK66F evaluation/development
board](https://www.nxp.com/products/processors-and-microcontrollers/arm-based-processors-and-mcus/kinetis-cortex-m-mcus/k-seriesperformancem4/k6x-ethernet/freedom-development-platform-for-kinetis-k66-k65-and-k26-mcus:FRDM-K66F)
to measure USB HID (Human Interface Device) processing latency.

The FRDM-K66F can be obtained for about 60 USD from various electronics stores
such as Mouser, Farnell or DigiKey.

The board features an NXP Kinetis K66 microcontroller capable of running at 180
MHz, and a USB 2.0 High Speed port. These features are helpful for doing precise
measurements.

Measurement
-----------

* The firmware reads the SW3 switch in a busy loop.
* When a button press was detected, a new measurement is started, i.e.:
** A CAPS_LOCK key press is simulated for 50 ms.
** The time of the next USB start-of-frame (SOF) interrupt is recorded.
** The time of the next HID set-report request is recorded.

Once the SW3 debounce time of 500ms elapsed, the measurement is converted from
the ARM cycle counter (DWT_CYCCNT register) to μs and printed to the CDC ACM
serial port in an easy-to-parse format.

The `sof=` value indicates the time which has elapsed between the debounced SW3
button press and the next USB start-of-frame interrupt (range: [0, 125μs]).

The `report=` value indicates the time which has elapsed between sending the
Caps Lock key press via USB and receiving the HID set-report response. The value
includes 125μs of USB transaction time, which you should subtract if you’re
interested in processing time.

Installation
------------

1. Download the most recent .bin file from the archive/ directory.
1. Connect the FRDM-K66F’s OpenSDAv2.1 Micro USB port to your computer.
1. Mount the mass storage device, copy the .bin file onto it, unmount.
1. Wait until the Power LED stopped rapidly flashing, indicating the .bin file has been written to the board’s flash memory.

Running a measurement
---------------------

1. Disconnect all cables from the FRDM-K66F (including a serial adapter, if any).
1. Connect the FRDM-K66F’s K66 Micro USB port to the computer you would like to measure.
1. On the computer, you should now see a new keyboard and a CDC ACM serial port.
1. Read measurements from the serial port, e.g. using `cat /dev/ttyACM0` on Linux.
1. Press the FRDM-K66F’s SW3 to start a new measurement.
