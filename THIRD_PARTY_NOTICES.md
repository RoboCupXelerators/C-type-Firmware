# Third-Party Notices

This repository contains vendor code alongside Xelerators application code.
The root Apache-2.0 license does not replace the licenses in those vendor
files.

## STMicroelectronics

The STM32F4 HAL and USB device middleware retain their original ST headers and
license references. The HAL files identify the BSD 3-Clause license. The USB
files refer to the ST Ultimate Liberty license SLA0044. Consult the original
headers and the current ST terms before redistributing a binary or a modified
vendor component.

## Arm CMSIS

CMSIS files retain Arm copyright and Apache-2.0 SPDX headers where supplied.

## FreeRTOS

The FreeRTOS kernel files retain the MIT permission notice from the upstream
source. The current bare-metal CMake targets do not link the FreeRTOS kernel,
but the vendor tree is retained for source compatibility with the official
project layout.

## RoboMaster and Damiao references

Board examples, motor protocol references, and test evidence are documented in
`reference/source_manifest.txt`. Vendor manuals and archived binary captures
are intentionally excluded from the initial commit; their local copies remain
available on the development host for audit work.
