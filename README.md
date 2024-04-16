1
import RPi.GPIO as GPIO
import time

LED_PIN = 3
GPIO.setmode(GPIO.BOARD)
GPIO.setup(LED_PIN, GPIO.OUT)

a=5
while a>0:
  print("HIGH")
  GPIO.output(LED_PIN,GPIO.HIGH)
  time.sleep(0.2)
  print("LOW")
  GPIO.output(LED_PIN,GPIO.LOW)
  time.sleep(0.2)
  a-=1
  
GPIO.cleanup()

