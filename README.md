# max1720x
An embedded-hal driver for the MAX1720x fuel gauge ICs

This driver is loosely based on
[Tock's MAX17205 driver](https://github.com/tock/tock/blob/master/capsules/src/max17205.rs)
but rewritten to use embedded-hal's I2C driver instead of the one built in to
Tock's kernel.  It does not take ownership of the I2C bus so works in
conjunction with other I2C drivers.

Tested on a Raspberry Pi with a MAX17205 but should in theory work on any
embedded-hal I2C device and with any of the MAX1720x family of ICs.