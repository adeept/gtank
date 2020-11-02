#!/usr/bin/env python3
# File name   : servo.py
# Description : Control Functions
# Author	  : William
# Date		: 2020/03/17
import time
import RPi.GPIO as GPIO
import threading
from mpu6050 import mpu6050
import Adafruit_PCA9685
import os
import json
import ultra
import Kalman_filter
import move

move.setup()

kalman_filter_X =  Kalman_filter.Kalman_filter(0.01,0.1)

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

MPU_connection = 1
try:
    sensor = mpu6050(0x68)
    print('mpu6050 connected, PT MODE ON')
except:
    MPU_connection = 0
    print('mpu6050 disconnected, ARM MODE ON')

curpath = os.path.realpath(__file__)
thisPath = "/" + os.path.dirname(curpath)

def num_import_int(initial):        #Call this function to import data from '.txt' file
    global r
    with open(thisPath+"/RPIservo.py") as f:
        for line in f.readlines():
            if(line.find(initial) == 0):
                r=line
    begin=len(list(initial))
    snum=r[begin:]
    n=int(snum)
    return n

pwm0_direction = 1
pwm0_init = num_import_int('init_pwm0 = ')
pwm0_max  = 520
pwm0_min  = 100
pwm0_pos  = pwm0_init

pwm1_direction = 1
pwm1_init = num_import_int('init_pwm1 = ')
pwm1_max  = 520
pwm1_min  = 100
pwm1_pos  = pwm1_init

pwm2_direction = 1
pwm2_init = num_import_int('init_pwm2 = ')
pwm2_max  = 520
pwm2_min  = 100
pwm2_pos  = pwm2_init


line_pin_right = 20
line_pin_middle = 16
line_pin_left = 19


def pwmGenOut(angleInput):
	return int(round(23/9*angleInput))


def setup():
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(line_pin_right,GPIO.IN)
	GPIO.setup(line_pin_middle,GPIO.IN)
	GPIO.setup(line_pin_left,GPIO.IN)


class Functions(threading.Thread):
	def __init__(self, *args, **kwargs):
		self.functionMode = 'none'
		self.steadyGoal = 0

		self.scanNum = 3
		self.scanList = [0,0,0]
		self.scanPos = 0
		self.scanDir = -1
		self.rangeKeep = 0.7
		self.scanRange = 100
		self.scanServo = 0
		self.turnServo = 0
		self.turnWiggle = 200

		super(Functions, self).__init__(*args, **kwargs)
		self.__flag = threading.Event()
		self.__flag.clear()

	def radarScan(self):
		global pwm0_pos
		scan_speed = 3
		result = []

		if pwm0_direction:
			pwm0_pos = pwm0_max
			pwm.set_pwm(0, 0, pwm0_pos)
			time.sleep(0.8)

			while pwm0_pos>pwm0_min:
				pwm0_pos-=scan_speed
				pwm.set_pwm(0, 0, pwm0_pos)
				dist = ultra.checkdist()
				if dist > 20:
					continue
				theta = 180 - (pwm0_pos-100)/2.55 # +30 deviation
				result.append([dist, theta])
		else:
			pwm0_pos = pwm0_min
			pwm.set_pwm(0, 0, pwm0_pos)
			time.sleep(0.8)

			while pwm0_pos<pwm0_max:
				pwm0_pos+=scan_speed
				pwm.set_pwm(0, 0, pwm0_pos)
				dist = ultra.checkdist()
				if dist > 20:
					continue
				theta = (pwm0_pos-100)/2.55
				result.append([dist, theta])
		pwm.set_pwm(0, 0, pwm0_init)
		return result


	def pause(self):
		self.functionMode = 'none'
		move.move(80, 'no', 'no', 0.5)
		self.__flag.clear()


	def resume(self):
		self.__flag.set()


	def automatic(self):
		self.scanPos = 0
		self.functionMode = 'Automatic'
		self.resume()


	def trackLine(self):
		self.functionMode = 'trackLine'
		self.resume()


	def keepDistance(self):
		self.functionMode = 'keepDistance'
		self.resume()


	def steady(self,goalPos):
		self.functionMode = 'Steady'
		xGet = sensor.get_accel_data()
		xGet = xGet['x']
		global pwm1_pos
		pwm1_pos = goalPos
		self.steadyGoal = xGet
		self.resume()


	def automaticProcessing(self):
		print('automaticProcessing')
		pwm.set_pwm(2, 0, pwm2_init)

		if self.scanDir == 1:
			if self.scanPos == 1:
				self.scanList[2] = ultra.checkdist()
				self.scanPos = 1
			else:
				pwm.set_pwm(self.scanServo, 0, pwm1_init-self.scanRange)
				time.sleep(1)
				self.scanList[2] = ultra.checkdist()
				self.scanPos = 1

			pwm.set_pwm(self.scanServo, 0, pwm1_init)
			time.sleep(1)
			self.scanList[1] = ultra.checkdist()
			self.scanPos = 2

			if self.scanPos == 3:
				self.scanList[0] = ultra.checkdist()
				self.scanPos = 3
			else:
				pwm.set_pwm(self.scanServo, 0, pwm1_init+self.scanRange)
				time.sleep(1)
				self.scanList[0] = ultra.checkdist()
				self.scanPos = 3

			self.scanDir = -1

		elif self.scanDir == -1:
			if self.scanPos == 3:
				self.scanList[0] = ultra.checkdist()
				self.scanPos = 3
			else:
				pwm.set_pwm(self.scanServo, 0, pwm1_init+self.scanRange)
				time.sleep(1)
				self.scanList[0] = ultra.checkdist()
				self.scanPos = 3

			pwm.set_pwm(self.scanServo, 0, pwm1_init)
			time.sleep(1)
			self.scanList[1] = ultra.checkdist()
			self.scanPos = 2

			if self.scanPos == 1:
				self.scanList[2] = ultra.checkdist()
				self.scanPos = 1
			else:
				pwm.set_pwm(self.scanServo, 0, pwm1_init-self.scanRange)
				time.sleep(1)
				self.scanList[2] = ultra.checkdist()
				self.scanPos = 1

			self.scanDir = 1

		if min(self.scanList) < self.rangeKeep:
			if max(self.scanList) < self.rangeKeep or min(self.scanList) < self.rangeKeep/3:
				move.move(80, 'backward', 'no', 0.5)
			elif self.scanList.index(min(self.scanList)) == 0:
				move.move(100, 'no', 'right', 0.5)
			elif self.scanList.index(min(self.scanList)) == 1:
				move.move(80, 'backward', 'no', 0.5)
			elif self.scanList.index(min(self.scanList)) == 2:
				move.move(100, 'no', 'left', 0.5)
		else:
			move.move(80, 'forward', 'no', 0.5)
		time.sleep(0.3)
		move.move(80, 'no', 'no', 0.5)


	def trackLineProcessing(self):
		status_right = GPIO.input(line_pin_right)
		status_middle = GPIO.input(line_pin_middle)
		status_left = GPIO.input(line_pin_left)
		#print('R%d   M%d   L%d'%(status_right,status_middle,status_left))
		if status_middle == 0:
			move.move(100, 'forward', 'no', 0.5)
		elif status_left == 0:
			move.move(100, 'no', 'left', 0.5)
		elif status_right == 0:
			move.move(100, 'no', 'right', 0.5)
		else:
			move.move(100, 'backward', 'no', 0.5)
		print(status_left,status_middle,status_right)
		time.sleep(0.01)


	def keepDisProcessing(self):
		print('keepDistanceProcessing')
		distanceGet = ultra.checkdist()
		if distanceGet > (self.rangeKeep/2+0.1):
			move.move(100, 'forward', 'no', 0.5)
		elif distanceGet < (self.rangeKeep/2-0.1):
			move.move(100, 'forward', 'no', 0.5)
		else:
			move.motorStop()


	def steadyProcessing(self):
		if MPU_connection:
			global pwm1_pos
			# print('steadyProcessing')
			xGet = sensor.get_accel_data()
			xGet = xGet['y']
			xOut = kalman_filter_X.kalman(xGet)

			if xOut > self.steadyGoal+0.2:
				xOut = abs(xOut - self.steadyGoal)
				pwm1_pos = int(pwm1_pos-xOut*5)
				pwm.set_pwm(1, 0, pwm1_pos)
			elif xOut < self.steadyGoal-0.2:
				xOut = abs(xOut - self.steadyGoal)
				pwm1_pos = int(pwm1_pos+xOut*5)
				pwm.set_pwm(1, 0, pwm1_pos)
			print(xOut)

			time.sleep(0.03)
		else:
			print('mpu6050 disconnected')
			self.pause()


	def functionGoing(self):
		if self.functionMode == 'none':
			self.pause()
		elif self.functionMode == 'Automatic':
			self.automaticProcessing()
		elif self.functionMode == 'Steady':
			self.steadyProcessing()
		elif self.functionMode == 'trackLine':
			self.trackLineProcessing()
		elif self.functionMode == 'keepDistance':
			self.keepDisProcessing()

	def run(self):
		while 1:
			self.__flag.wait()
			self.functionGoing()
			pass


if __name__ == '__main__':
	pass
	# fuc=Functions()
	# fuc.radarScan()
	# fuc.start()
	# fuc.automatic()
	# # fuc.steady(300)
	# time.sleep(30)
	# fuc.pause()
	# time.sleep(1)
	# move.move(80, 'no', 'no', 0.5)