# README

The repository contains the driver code along with simple initialization code
with two message objects set-up and receive/transmit logic in place.

Since the code targets the ARM Cortex-M4F based CPU inside the TIVA-C, the code
was tested to compile using `arm-none-eabi-gcc` version 10.3.1 with the
`-march=armv7-m` option.

The Makefile contains a target called `arm`, it requires `arm-none-eabi-gcc` to
be installed. To build it, run `make arm`. 

the `--specs=nosys.specs` option is used with `arm-none-eabi-gcc` to enable
semi-hosting.
