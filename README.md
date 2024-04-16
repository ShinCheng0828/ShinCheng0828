2
import RPi.GPIO as GPIO
import time

LED_PIN = [2,3,5,6]
GPIO.setmode(GPIO.BCM)
for i in LED_PIN:
 GPIO.setup(i, GPIO.OUT)
 GPIO.output(i,GPIO.HIGH)
 
a=1
while a<6:
  print("LOOP %d"%a)
  for i in LED_PIN:
   print ('LED %d is ON'%i,end = ', ')
   GPIO.output(i,GPIO.LOW)
   time.sleep(0.2)
   print ("LED %d is OFF"%i)
   GPIO.output(i,GPIO.HIGH)
  a+=1
GPIO.cleanup()


