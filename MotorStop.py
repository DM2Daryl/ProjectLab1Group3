'''
CODE TO RUN PULLEY MOTOR
from machine import Pin, PWM
from Mdriver import MotorDriver
import utime 

Pulley = MotorDriver()
while True:
    Pulley.PulleyMotor(1,20)
    utime.sleep(1)
    
    #pos 1 lifts , pos 2 drops
    
'''

'''

CODE TO STOP ALL MOTOR FUNCTION

from Mdriver import MotorDriver
from machine import Pin
import utime

led = Pin(25)

led.value(0)

motors = MotorDriver()
Pulley = MotorDriver()

#motors.Forward(60)
#utime.sleep(1)
Pulley.PulleyMotor(0 ,20)

motors.Stop(0)


''' 

