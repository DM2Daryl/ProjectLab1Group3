#Speed Sensor test Code
from machine import Pin
import utime
SpeedPin = Pin(6, Pin.OUT)


while True: #Speed Sensor outputs a 1 when nothing is found, and a 0 when something is in front of the sensor. 
    print(SpeedPin.value())
    utime.sleep(0.5)
    
    
#once a zero is output, start a timer. Once another zero output is completed, end the timer. the distance between the first rim and the second rim is the meters and time is seconds, this way you can get m/s
#can hypothetically get speed (high confidence) , distance traveled? (maybe use  