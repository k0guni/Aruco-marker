import time
import busio
import RPi.GPIO as GPIO

from board import SCL_1, SDA_1
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL_1, SDA_1)

pca = PCA9685(i2c, address=0x40)
pca.frequency = 100

pwm_channel = 0
dir_channel = 1
brk_channel = 2

class Sensor():
    def __init__(self,trig_pin,echo_pin):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD) 
        GPIO.setup(self.trig_pin,GPIO.OUT)
        GPIO.setup(self.echo_pin,GPIO.IN)

    def getDistance(self):
        GPIO.output(self.trig_pin, GPIO.LOW)
        # TRIG = HIGH
        GPIO.output(self.trig_pin, True)
        # 0.01ms TRIG = LOW
        time.sleep(0.00001)        
        GPIO.output(self.trig_pin, False)

        signaloff=0
        signalon=0
        timeout = 1
        start_time = time.time()

        while GPIO.input(self.echo_pin) == 0:
            signaloff = time.time()
            if signaloff - start_time > timeout:
                raise RuntimeError("Timeout waiting for signal off")
        while GPIO.input(self.echo_pin) == 1:
            signalon = time.time()
        if signalon - start_time > timeout:
            raise RuntimeError("Timeout waiting for signal on")
        return (signalon - signaloff) * 17000

    def __del__(self):
        GPIO.cleanup()

def motor(speed, i):

    if speed > 100:
        speed = 100
    elif speed < -100:
        speed = -100
    if speed == 0:
        pca.channels[dir_channel+4*i].duty_cycle = int(0xffff)
        pca.channels[brk_channel+4*i].duty_cycle = int(0xffff)
    elif speed > 0:
        pca.channels[dir_channel+4*i].duty_cycle = int(0xffff)
        pca.channels[brk_channel+4*i].duty_cycle = int(0x0000)
        pca.channels[pwm_channel+4*i].duty_cycle = int((speed/100) * (0xffff/100))
    else:
        pca.channels[dir_channel+4*i].duty_cycle = int(0x0000)
        pca.channels[brk_channel+4*i].duty_cycle = int(0x0000)
        pca.channels[pwm_channel+4*i].duty_cycle = int((speed/100) * (0xffff/100))

def forward(speed):
    motor(speed,0)
    motor(speed,1)
    motor(speed,2)
    motor(speed,3)
def backward(speed):
    motor(-speed,0)
    motor(-speed,1)
    motor(-speed,2)
    motor(-speed,3)
def right(speed):
    motor(speed,0)
    motor(-speed,1)
    motor(-speed,2)
    motor(speed,3)
def left(speed):
    motor(-speed,0)
    motor(speed,1)
    motor(speed,2)
    motor(-speed,3)
def cw(speed):
    motor(speed,0)
    motor(-speed,1)
    motor(speed,2)
    motor(-speed,3)
def ccw(speed):
    motor(-speed,0)
    motor(speed,1)
    motor(-speed,2)
    motor(speed,3)
def stop(void):
    motor(0,0)
    motor(0,1)
    motor(0,2)
    motor(0,3)

def main():
        sensors = [
            Sensor(19,21),
            Sensor(22,37),
            Sensor(33,35),
            Sensor(16,15),
            Sensor(31,32)         
        ]

        while True:
            forward(30)
            distances = [sensor.getdistance() for sensor in sensors]
            time.sleep(0.1)
            if distances[0] < 100 :
                if distances[0] - distances[2] < 10:
                    left(30)
                    if distances[0] >= 100:
                        stop()
                else :
                    cw(30)
                    if distances[0] >= 100:
                        stop()                        
            if distances[1] < 100 :
                if distances[1] - distances[3] < 10:
                    left(30)
                    if distances[1] >= 100:
                        stop()
                else :
                    cw(30)
                    if distances[1] >= 100:
                        stop()   
            if distances[4] < 100 :
                stop()
                if distances[4] >= 100:
                    forward(30)
        for sensor in sensors:
            del sensor
    if __name__ == "__main__":
        main()
        pca.deinit()