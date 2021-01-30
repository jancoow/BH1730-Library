# BH1730 Library

This is an easy to use library for communicating with a BH1730 light sensor.  It automatically converts the values to lux. 
Gain can be changed from X1 to X2, X64 and X128. The gain is taken into account when calculating the lux values. 

This sensor use I2C to communicate, 2 pins are required to interface. 
There is no need for connecting the interupt pin. 

