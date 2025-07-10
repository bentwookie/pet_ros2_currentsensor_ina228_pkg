# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
#
# $ sudo i2cdetect -y 1
#	     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
#	00:                         -- -- -- -- -- -- -- -- 
#	10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#	20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#	30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#	40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#	50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#	60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#	70: -- -- -- -- -- -- -- --                         
# $ 
# $ sudo pip3 install adafruit-circuitpython-ina228
# $ sudo pip3 install adafruit-blinka
#
# $ sudo python3 INA228.py 
# $ python3 INA228.py 
#
"""Sample code and test for adafruit_ina228"""

import time
import board
from adafruit_ina228 import ADCResolution, BusVoltageRange, INA228


i2c_bus = board.I2C()

ina228 = INA228(i2c_bus)

print("ina228 test")

# display some of the advanced field (just to test)
print("Config register:")
print("  bus_voltage_range:    0x%1X" % ina228.bus_voltage_range)
print("  gain:                 0x%1X" % ina228.gain)
print("  bus_adc_resolution:   0x%1X" % ina228.bus_adc_resolution)
print("  shunt_adc_resolution: 0x%1X" % ina228.shunt_adc_resolution)
print("  mode:                 0x%1X" % ina228.mode)
print("")

# optional : change configuration to use 32 samples averaging for both bus voltage and shunt voltage
ina228.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
ina228.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S

# optional : change voltage range to 16V
ina228.bus_voltage_range = BusVoltageRange.RANGE_16V

# measure and display loop
while True:
    bus_voltage = ina228.bus_voltage  # voltage on V- (load side)
    shunt_voltage = ina228.shunt_voltage  # voltage between V+ and V- across the shunt
    current = ina228.current  # current in mA
    power = ina228.power  # power in watts

    # INA228 measure bus voltage on the load side. So PSU voltage = bus_voltage + shunt_voltage
    print("Voltage (VIN+) : {:6.3f}   V".format(bus_voltage + shunt_voltage))
    print("Voltage (VIN-) : {:6.3f}   V".format(bus_voltage))
    print("Shunt Voltage  : {:8.5f} V".format(shunt_voltage))
    print("Shunt Current  : {:7.4f}  A".format(current / 1000))
    print("Power Calc.    : {:8.5f} W".format(bus_voltage * (current / 1000)))
    print("Power Register : {:6.3f}   W".format(power))
    print("")

    # Check internal calculations haven't overflowed (doesn't detect ADC overflows)
    if ina228.overflow:
        print("Internal Math Overflow Detected!")
        print("")

    time.sleep(2)