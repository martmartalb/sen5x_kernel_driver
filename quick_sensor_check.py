import sys
import board
import busio
import time

i2c = busio.I2C(board.SCL, board.SDA)

print("I2C bus initialized:", [hex(i) for i in i2c.scan()])

sen5x = 0x69

if not sen5x in i2c.scan():
    print("Sensor not found at address", hex(sen5x))
    sys.exit(1)

print("Sensor found at address", hex(sen5x))



i2c.writeto(sen5x, bytearray([0xD2, 0x10]))  # Clear status register
time.sleep(0.2)  # Wait for the sensor to wake up

i2c.writeto(sen5x, bytearray([0x00, 0x21]))  # Wake up the sensor
time.sleep(0.2)  # Wait for the sensor to wake up
while(1):
    i2c.writeto(sen5x, bytearray([0x03, 0xC4])) 
    time.sleep(0.2)
    result = bytearray(24)
    i2c.readfrom_into(sen5x, result)
    print("Measurement result:", [hex(i) for i in result])
    time.sleep(2)  # Wait for the sensor to process the data
i2c.writeto(sen5x, bytearray([0xd3, 0x04])) # Reset the sensor