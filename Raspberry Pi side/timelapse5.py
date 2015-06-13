import glob
import re
import os
import serial
from time import sleep
from time import time
import datetime
import RPi.GPIO as GPIO

def CaCuProcGetLogNum(): # this scans the folder for the highest numbered log and adds 1
	cacufilelist = glob.glob('logs/log*.csv')
	caculistsize = len(cacufilelist)
	for x in range (0, caculistsize):
		cacufilelist[x] = int(re.sub("[^0-9]","", cacufilelist[x]))
	cacufilenum = max(cacufilelist)+1
	return cacufilenum

caculognum = CaCuProcGetLogNum()
cacuframenum = 0
caculoop = 0
cacuitscan = 0
cacuitphoto = 0
cacustart = time()
cacudelay = time()
print "Starting log number "+str("%04d" % caculognum)
cacuser = serial.Serial('/dev/ttyUSB0', 9600)
sleep(3) # give the arduino time to boot
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)
caculogname = "logs/log"+str("%04d" % caculognum)+".csv"
os.system("echo 252 > /sys/class/gpio/export")
os.system("echo 'out' > /sys/class/gpio/gpio252/direction")
os.system("echo '0' > /sys/class/gpio/gpio252/value")	# turn backlight off

while (caculoop < 1):	# main program loop
	if (time() > cacudelay):
		cacudelay = cacudelay + 10
		cacufile = open(caculogname, 'a')	# open log file	
		cacuser.write('2')	# request log mode scan from arduino
		sleep(1)	# give arduino time to respond
		cacuscan = cacuser.readline()	# read scan data
		cacufile.write(str(time() - cacustart)+","+str(cacuframenum)+","+cacuscan)	# write data to file	
		cacufile.close()	# close file
		if (cacuitphoto < 1):	# if it's time to take a photo
			os.system("raspistill -n -w 1024 -h 768 -t 100 -ex auto -mm matrix -o logs/log"+str("%04d" % caculognum)+"frame"+str("%04d" % cacuframenum)+".jpg")
			cacuframenum = cacuframenum + 1
			cacuitphoto = 2		# number of iterations -1 until next photo
		else:
			cacuitphoto = cacuitphoto - 1	

	sleep(0.5)	
	if (GPIO.input(18) == 0):
		os.system("echo '0' > /sys/class/gpio/gpio252/value")	# turn backlight off
	if (GPIO.input(27) == 0):
		os.system("echo '1' > /sys/class/gpio/gpio252/value")	# turn backlight on
	if (GPIO.input(22) == 0):
		caculoop = 3
		print "stopping, button pushed"
	if (cacuframenum > 9997):
		caculoop = 3
		print "stopping, frame limit exceeded"
	
os.system("echo '1' > /sys/class/gpio/gpio252/value")	# turn backlight on
quit()