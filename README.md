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

3
import RPi.GPIO as GPIO
import time

LED_PIN = [2,3,5,6]
GPIO.setmode(GPIO.BCM)
for i in LED_PIN:
 GPIO.setup(i, GPIO.OUT)
 GPIO.output(i,GPIO.HIGH)
N = eval(input('LOOP NUMBER = '))
a = N
while a > 0:
  print("LOOP %d"%a)
  for i in LED_PIN:
   print ('LED %d is ON'%i,end = ', ')
   GPIO.output(i,GPIO.LOW)
   time.sleep(0.2)
   print ("LED %d is OFF"%i)
   GPIO.output(i,GPIO.HIGH)
  a-= 1
GPIO.cleanup()

4
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
import time

def buzzer():
    GPIO.setmode(GPIO.BCM)
    GPIO_PIN=12
    GPIO.setup(GPIO_PIN,GPIO.OUT)
    
    GPFrequency=50
    pwm = GPIO.PWM(GPIO_PIN,GPFrequency)
    pwm.start(50)
    pwm.ChangeFrequency(2222)
    time.sleep(10)
    pwm.stop()
    GPIO.cleanup()


buzzer()

5.
from scipy.spatial import distance
from imutils import face_utils
import imutils
import dlib
import cv2
import RPi.GPIO as GPIO

#import winsound
import time
import RPi.GPIO as GPIO

def buzzer():
    GPIO.setmode(GPIO.BCM)
    GPIO_PIN=12
    GPIO.setup(GPIO_PIN,GPIO.OUT)
    
    GPFrequency=50
    pwm = GPIO.PWM(GPIO_PIN,GPFrequency)
    pwm.start(50)
    pwm.ChangeFrequency(2222)
    time.sleep(10)
    pwm.stop()
    GPIO.cleanup()

    
def eye_aspect_ratio(eye):
	A = distance.euclidean(eye[1], eye[5])
	B = distance.euclidean(eye[2], eye[4])
	C = distance.euclidean(eye[0], eye[3])
	ear = (A + B) / (2.0 * C)
	return ear

GPIO.setmode(GPIO.BCM)
thresh = 0.25
frame_check = 20
detect = dlib.get_frontal_face_detector()
predict = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")# Dat file is the crux of the code

(lStart, lEnd) = face_utils.FACIAL_LANDMARKS_68_IDXS["left_eye"]
(rStart, rEnd) = face_utils.FACIAL_LANDMARKS_68_IDXS["right_eye"]
cap=cv2.VideoCapture(0)
flag=0
while True:
	ret, frame=cap.read()
	frame = imutils.resize(frame, width=450)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)#COLOR SPACE
	subjects = detect(gray, 0)

	for subject in subjects:
		shape = predict(gray, subject)
		shape = face_utils.shape_to_np(shape)#converting to NumPy Array
		leftEye = shape[lStart:lEnd]
		rightEye = shape[rStart:rEnd]
		leftEAR = eye_aspect_ratio(leftEye)
		rightEAR = eye_aspect_ratio(rightEye)
		ear = (leftEAR + rightEAR) / 2.0
		leftEyeHull = cv2.convexHull(leftEye)
		rightEyeHull = cv2.convexHull(rightEye)
		cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
		cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)
		if ear < thresh:
			flag += 1
			print (flag)
			if flag >= frame_check:
				cv2.putText(frame, "************* Drowsy ALERT! ***************", (10, 30),
					cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
				cv2.putText(frame, "************* Drowsy ALERT! ***************", (10,325),
					cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
				# winsound.Beep(1000,100)
				buzzer()
				print ("Drowsy")
		else:
			flag = 0
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
		break
cv2.destroyAllWindows()
cap.stop()

6.
#!/usr/bin/env python


import i2c_driver
import time


mylcd = i2c_driver.LCD()


while True:
    mylcd.lcd_display_string(time.strftime('%I:%M:%S %p'), 1)
    mylcd.lcd_display_string(time.strftime('%a %b %d, 20%y'), 2)
