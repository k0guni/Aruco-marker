#2023-05-23
#Tech University of Korea
#Mechatronics Engineering 2018130002 Ko Geon Hui

import time
import RPi.GPIO as GPIO


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

        while GPIO.input(self.echo_pin) == 0:
            signaloff = time.time()

        while GPIO.input(self.echo_pin) == 1:
            signalon = time.time()

        return (signalon - signaloff) * 17000

    def __del__(self):
        GPIO.cleanup()

    def main():
        sensors = [
            Sensor(19,21),
            Sensor(22,37),
            Sensor(33,35),
            Sensor(16,15),
            Sensor(31,32)         
        ]

        while True:
            distances = [sensor.getdistance() for sensor in sensors]
            print(" | ".join("{:.0f}cm".format(distance) for distance in distances))
            time.sleep(0.1)
        for sensor in sensors:
            del sensor
    if __name__ == "__main__":
        main()
