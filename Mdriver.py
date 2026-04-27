from machine import Pin, PWM
from time import sleep

# SETUP
class MotorDriver:

    def __init__(self):
        self.en1 = Pin(11, Pin.OUT)
        self.ch1 = Pin(12, Pin.OUT)
        self.ch2 = Pin(13, Pin.OUT)

        self.en2 = Pin(20, Pin.OUT)
        self.ch3 = Pin(19, Pin.OUT)
        self.ch4 = Pin(18, Pin.OUT)

        self.button = Pin(10, Pin.IN, Pin.PULL_DOWN)

        self.LMspeed = PWM(self.en1)
        self.RMspeed = PWM(self.en2)

    def LeftMotor(self, direction, speed):
        self.LMspeed.freq(1000)
        self.LMspeed.duty_u16(int((speed/100)*65535))

        if direction == 0:
            self.ch1.value(0)
            self.ch2.value(0)
        elif direction == 1:
            self.ch1.value(0)
            self.ch2.value(1)
        elif direction == 2:
            self.ch1.value(1)
            self.ch2.value(0)
        elif direction == 3:
            self.ch1.value(1)
            self.ch2.value(1)

    def RightMotor(self, direction, speed):
        self.RMspeed.freq(1000)
        self.RMspeed.duty_u16(int((speed/100)*65535))

        if direction == 0:
            self.ch3.value(0)
            self.ch4.value(0)
        elif direction == 1:
            self.ch3.value(0)
            self.ch4.value(1)
        elif direction == 2:
            self.ch3.value(1)
            self.ch4.value(0)
        elif direction == 3:
            self.ch3.value(1)
            self.ch4.value(1)

    def Forward(self, speed):
        self.LeftMotor(2, speed)
        self.RightMotor(2, speed)

    def Reverse(self, speed):
        self.LeftMotor(1, speed)
        self.RightMotor(1, speed)

    def Coast(self, speed):
        self.LeftMotor(0, speed)
        self.RightMotor(0, speed)

    def Stop(self, speed):
        self.LeftMotor(3, speed)
        self.RightMotor(3, speed)

    def TurnLeft(self, speed):
        self.LeftMotor(1, speed)
        self.RightMotor(2, speed)

    def TurnRight(self, speed):
        self.LeftMotor(2, speed)
        self.RightMotor(1, speed)

