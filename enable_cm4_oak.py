import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(1, GPIO.OUT)
GPIO.output(1, GPIO.LOW)
