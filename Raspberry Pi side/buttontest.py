import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)

while True:
	input_state4 = GPIO.input(18)
	input_state3 = GPIO.input(27)
	input_state2 = GPIO.input(22)

	if input_state4 == False: print('Button4 Pressed GPIO18')
	if input_state3 == False: print('Button3 Pressed GPIO27')
	if input_state2 == False: 
		print('Button2 Pressed GPIO22')
		quit()
	time.sleep(0.2)