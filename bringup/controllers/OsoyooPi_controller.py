#  ___   ___  ___  _   _  ___   ___   
# / _ \ /___)/ _ \| | | |/ _ \ / _ \
#| |_| |___ | |_| | |_| | |_| | |_| |
# \___/(___/ \___/ \__  |\___/ \___/
#                  (____/ 
# Osoyoo Raspberry Pi Web Camera Control Robot Car
# tutorial url: https://osoyoo.com/?p=32066
#
# BASIC CONTROL MODULE


from __future__ import division
import time
import Adafruit_PCA9685		# imports the PCA9685 module
from flask import Flask, render_template, request
import RPi.GPIO as GPIO

pi_ip_address = '192.168.2.231 '		# replace with your Raspberry Pi IP address
					
# motors initialization

app = Flask(__name__)
GPIO.setmode(GPIO.BCM)			# GPIO number in BCM mode
GPIO.setwarnings(False)

IN1 = 23				# define actuators GPIOs
IN2 = 24				# define actuators GPIOs
IN3 = 27				# define actuators GPIOs
IN4 = 22				# define actuators GPIOs
ENA = 0 				# Right motor speed PCA9685 port 0
ENB = 1  				# Left motor speed PCA9685 port 1

GPIO.setup(IN1, GPIO.OUT)		# Define motor control pin as output	
GPIO.setup(IN2, GPIO.OUT)		# Define motor control pin as output
GPIO.setup(IN3, GPIO.OUT) 		# Define motor control pin as output
GPIO.setup(IN4, GPIO.OUT) 		# Define motor control pin as output


# servos initialization

pwm = Adafruit_PCA9685.PCA9685()	# alternatively specify a different address and/or bus: 
					#  -> pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)
					# initialises the PCA9685 using the default address (0x40)

pwm.set_pwm_freq(60)			# set pwm frequency to 60hz, good for servos

servo_sleep_time = 0.05		# set sleep time (in seconds) to call between any servo input modification

# steering map

# controls

class steer:
	pin = 15			# steering servo connects to PWM 15
	
	# DUTY CYCLES (0-4095) - increment in values cause STEERING rotation to RIGHT
	CENTER = 416			# steering facing front
	MAX_R = 514			# pwm for steering facing right
	MAX_L = 345			# pwm for facing left
	
	MIN_ROT_radius = 0.4		# insert minimum steer radius [m] corresponding to both MAX_ROT
	DEADZONE = 0.1			# maximum steering radius for witch the turn is computed

	# STEERING MAP
	radius_list = -2.2, -1.74, -1.105, -0.865, -0.675, -0.61, -0.545, -0.48, -0.44, -0.423, -0.4, 0.4, 0.43, 0.485, 0.525, 0.625, 0.735, 0.960, 1.185, 1.830, 2.380	# [rad]
	pwm_list = 430, 434, 443, 452, 461, 470, 479, 488, 497, 506, 514, 340, 345, 352, 359, 366, 373, 380, 387, 394, 401		# radius corresponding pwms (same index than radius_list)
	
	
	def turn_radius(radius, TEST_print_pwm = False): 	# turns the car to the given radius
		if radius == 0:
			raise ValueError("radius given is 0, no difference will be set")
		if -steer.MIN_ROT_radius < radius < 0:
			print(f"ERROR: given radius ({radius} m) has an absolute value lower than the minimum set ({steer.MIN_ROT_radius} m), minimum value will be set")
			pwm.set_pwm(steer.pin, 0, steer.MAX_R)
		elif 0 < radius < steer.MIN_ROT_radius:
			print(f"ERROR: given radius ({radius} m) has an absolute value lower than the minimum set ({steer.MIN_ROT_radius} m), minimum value will be set")
			pwm.set_pwm(steer.pin, 0, steer.MAX_L)
		elif radius < steer.radius_list[0] or radius > steer.radius_list[len(steer.radius_list)-1]:
			pwm.set_pwm(steer.pin, 0, steer.CENTER)
		else:
			for i in range(0, len(steer.radius_list)-1):
				if steer.radius_list[i] <= radius <= steer.radius_list[i+1]:
					pwm_value = int(round(steer.pwm_list[i]+((radius-steer.radius_list[i])/(steer.radius_list[i+1]-steer.radius_list[i]))*(steer.pwm_list[i+1]-steer.pwm_list[i])))
					pwm.set_pwm(steer.pin, 0, pwm_value)
					if TEST_print_pwm == True:
						print(f"Steering pwm set to {pwm_value}")
					return
			raise ValueError("unknown error, pwm not assigned")
	
	def center():				# realigns the steering to center
		pwm.set_pwm(steer.pin, 0, steer.CENTER)

	def turn_right(percent_value=100, TEST_print_pwm=False):	# steers to the right of a percent value of the max angle set
		if percent_value==100:
			angle = steer.MAX_R
			pwm.set_pwm(steer.pin, 0, steer.MAX_R)
		elif 0<=percent_value<100:
			angle = int(round(steer.CENTER+(percent_value/100)*steer.MAX_R))
			pwm.set_pwm (steer.pin, 0, angle)
		else:
			all_off()
			raise ValueError("parameter 'percent_value' must be a number between 0 and 100")
		if TEST_print_pwm==True:	
			print(f"steering pwm set to {angle}")
		#time.sleep(servo_sleep_time)
	
	def turn_left(percent_value=100, TEST_print_pwm=False):	# steers to the left of a percent value of the max angle set
		if percent_value==100:
			angle = steer.MAX_R
			pwm.set_pwm(steer.pin, 0, steer.MAX_L)
		elif 0<=percent_value<100:
			angle = int(round(steer.CENTER-(percent_value/100)*steer.MAX_L))
			pwm.set_pwm (steer.pin, 0, angle)
		else:
			all_off()
			raise ValueError("parameter 'percent_value' must be a number between 0 and 100")
		if TEST_print_pwm==True:	
			print(f"steering pwm set to {angle}")
		#time.sleep(servo_sleep_time)

	def TEST_set_pwm(duty_cycle):		# sets the steering pwm to the given value
		pwm.set_pwm(steer.pin, 0, duty_cycle)
		

class cam:
	pin = 14			# camera servo connects to PWM 14
        
	# DUTY CYCLES (0-4095) - increment in values cause CAMERA rotation to LEFT
	CENTER = 315			# camera facing front
	MAX_ROT = 140			# maximum rotation
	MAX_R = CENTER-MAX_ROT		# camera facing fully right
	MAX_L = CENTER+MAX_ROT		# camera facing fully left
	RIGHT_90 = 115			# camera facing right 90°
	LEFT_90 = 570			# camera facing left 90°
	
	def center():				# realigns the camera to center
		pwm.set_pwm(cam.pin, 0, cam.CENTER)
	
	def turn_right(percent_value=100):	# steers to the right of a percent value of the max angle set
		if percent_value==100:
			pwm.set_pwm(cam.pin, 0, cam.MAX_R)
		elif 0<=percent_value<100:
			angle = int(round(cam.CENTER-(percent_value/100)*cam.MAX_ROT))
			pwm.set_pwm (cam.pin, 0, angle)
		else:
			all_off()
			raise ValueError("parameter 'percent_value' must be a number between 0 and 100")
			
	def turn_left(percent_value=100):	# steers to the left of a percent value of the angle set
		if percent_value==100:
			pwm.set_pwm(cam.pin, 0, cam.MAX_L)
		elif 0<=percent_value<100:
			angle = int(round(cam.CENTER+(percent_value/100)*cam.MAX_ROT))
			pwm.set_pwm (cam.pin, 0, angle)
		else:
			all_off()
			raise ValueError("parameter 'percent_value' must be a number between 0 and 100")
	
	def turn_right_90():			# cam turns 90° to the left
		pwm.set_pwm(cam.pin, 0, cam.RIGHT_90)
		time.sleep(servo_sleep_time)
			
	def turn_left_90():			# cam turns 90° to the right
		pwm.set_pwm(cam.pin, 0, cam.LEFT_90)
		time.sleep(servo_sleep_time)
	
class motors:
	R_F = IN1	# RIGHT motor FORWARD voltage pin
	R_B = IN2	# RIGHT motor BACKWARD voltage pin
	L_F = IN3	# LEFT motor FORWARD voltage pin
	L_B = IN4	# LEFT motor BACKWARD voltage pin
	R_pwm = ENA	# RIGHT motor pwm pin
	L_pwm = ENB	# LEFT motor pwm pin
	# look at start of module to see and change pin numbers relative to INs and ENs
	
	MAX_SPEED_DC = 4000		# maximum duty cicle for the maximum speed (0-4095)
	MAX_SPEED_fwd = 0.33		# insert the maximum speed [m/s]
	MAX_SPEED_bwd = 0.28		# (same as 'fwd' but backwards)
	MAX_SPEED_NO_RES_fwd = 0.455	# set the value so that the curve matches experimental data
	MAX_SPEED_NO_RES_bwd = 0.448	# (same as 'fwd' but backwards)
	MAX_STILL_DC_fwd = 1100	# maximum duty cycle for witch the car stays still, to test with TEST_set_pwm function
	MAX_STILL_DC_bwd = 1500	# (same as 'fwd' but backwards)
	
	def set_forward():			# sets both motors direction to forward	
		GPIO.output(motors.R_F, GPIO.HIGH)
		GPIO.output(motors.R_B, GPIO.LOW)
		GPIO.output(motors.L_F, GPIO.HIGH)
		GPIO.output(motors.L_B, GPIO.LOW)
		
	def set_backwards():			# sets both motors direction to backward
		GPIO.output(motors.R_F, GPIO.LOW)
		GPIO.output(motors.R_B, GPIO.HIGH)
		GPIO.output(motors.L_F, GPIO.LOW)
		GPIO.output(motors.L_B, GPIO.HIGH)
		
	def stopcar():				# stops the car
		GPIO.output(motors.R_F, GPIO.LOW)
		GPIO.output(motors.R_B, GPIO.LOW)
		GPIO.output(motors.L_F, GPIO.LOW)
		GPIO.output(motors.L_B, GPIO.LOW)
		pwm.set_pwm(motors.R_pwm, 0, 0)
		pwm.set_pwm(motors.L_pwm, 0, 0)
	
	def go(percent_value=100, TEST_print_pwm=False):	# sets the motor pwm to a percent value of the maximum set
		if percent_value==100:
			pwm_value = motors.MAX_SPEED_DC
			pwm.set_pwm(motors.R_pwm, 0, pwm_value)
			pwm.set_pwm(motors.L_pwm, 0, pwm_value)
		elif 0<=percent_value<100:
			pwm_value = int(round((percent_value/100)*motors.MAX_SPEED_DC))
			pwm.set_pwm(motors.R_pwm, 0, pwm_value)
			pwm.set_pwm(motors.L_pwm, 0, pwm_value)
		else:
			all_off()
			raise ValueError("parameter 'percent_value' must be a number between 0 and 100")
		if TEST_print_pwm==True:	
				print(f"Motors pwm set to {pwm_value}")
			
	def speed(speed, TEST_print_pwm=False):	# sets the motor pwm to match speed to the given absolute value [m/s]
		if 0<=speed<=motors.MAX_SPEED_fwd:
			pwm_value = int(round(motors.MAX_STILL_DC_fwd/(1-speed/motors.MAX_SPEED_NO_RES_fwd)))
			pwm.set_pwm(motors.R_pwm, 0, pwm_value)
			pwm.set_pwm(motors.L_pwm, 0, pwm_value)
		elif -motors.MAX_SPEED_bwd<=speed<=0:
			pwm_value = int(round(motors.MAX_STILL_DC_bwd/(1+speed/motors.MAX_SPEED_NO_RES_bwd)))
			pwm.set_pwm(motors.R_pwm, 0, pwm_value)
			pwm.set_pwm(motors.L_pwm, 0, pwm_value)
		else:
			raise ValueError("parameter 'speed' must have absolute value <= 'MAX_SPEED'")
		if TEST_print_pwm==True:	
				print(f"Motors pwm set to {pwm_value}")
	
	'''def diff_speed(turning_radious, percent_value=100):	# (WRONG, motors are in torque control, not speed!) sets the 2 motors speeds to match turning
		# radious [m], its absolute value must be higher than the min reachable by the car with right behaviour
		TRACK = 0.156	# defines vehicle track (between middles of the tires)
		if -MIN_ROT_radious<R<0:
			if percent_value==100:
				rightspeed = int(round(motors.MAX_SPEED_DC*(1-(TRACK/2)/abs(turning_radious))))
				leftspeed = int(round(motors.MAX_SPEED_DC*(1+(TRACK/2)/abs(turning_radious))))
				pwm.set_pwm(motors.R_pwm, 0, rightspeed)
				pwm.set_pwm(motors.L_pwm, 0, leftspeed)
			elif 0<=percent_value<100:
				center_reduced_speed = int(round((percent_value/100)*motors.MAX_SPEED_DC))
				rightspeed = int(round(center_reduced_speed*(1-(TRACK/2)/abs(turning_radious))))
				leftspeed = int(round(center_reduced_speed*(1+(TRACK/2)/abs(turning_radious))))
				pwm.set_pwm(motors.R_pwm, 0, rightspeed)
				pwm.set_pwm(motors.L_pwm, 0, leftspeed)
			else:
				all_off()
				raise ValueError("third parameter 'percent_value' must be a number between 0 and 100")
		elif 	MIN_ROT_radious>R>0:
			if percent_value==100:
				rightspeed = int(round(motors.MAX_SPEED_DC*(1+(TRACK/2)/turning_radious)))
				leftspeed = int(round(motors.MAX_SPEED_DC*(1-(TRACK/2)/turning_radious)))
				pwm.set_pwm(motors.R_pwm, 0, rightspeed)
				pwm.set_pwm(motors.L_pwm, 0, leftspeed)
			elif 0<=percent_value<100:
				center_reduced_speed = int(round(percent_value/100)*motors.MAX_SPEED_DC)
				rightspeed = int(round(center_reduced_speed*(1+(TRACK/2)/turning_radious)))
				leftspeed = int(round(center_reduced_speed*(1-(TRACK/2)/turning_radious)))
				pwm.set_pwm(motors.R_pwm, 0, rightspeed)
				pwm.set_pwm(motors.L_pwm, 0, leftspeed)
			else:
				all_off()
				raise ValueError("third parameter 'percent_value' must be a number between 0 and 100")
		elif r==0:
			speed(percent_value)
		else:
			raise ValueError("first parameter 'turning_radious' must have an absolute value higher than 'MIN_ROT_radious' between 0 and 100")'''		

	def TEST_set_pwm(dc):
		pwm.set_pwm(motors.R_pwm, 0, dc)
		pwm.set_pwm(motors.L_pwm, 0, dc)


# main function

def drive (linear_vel, angular_vel, previous_linear_vel=0, TEST_print_pwm=False):		# controls the car
	# linear_vel [m/s] (must be < motors.MAX_SPEED_velocity)
	# angular_vel [rad/s] (>0 anti-clockwise)
	# previous_linear_vel [m/s] is the one of the previous step
	
	MIN_DRIVE_VELOCITY = 0.03 	# minimum velocity to activate motors
	
	if linear_vel > 0.0 and previous_linear_vel <= 0.0:
		motors.set_forward()
		if TEST_print_pwm == True:
			print("Motors direction set to FORWARD")
	elif linear_vel < 0.0 and previous_linear_vel >= 0.0:
		motors.set_backwards()
		if TEST_print_pwm == True:
                        print("Motors direction set to BACKWARDS")
	try:
		if -MIN_DRIVE_VELOCITY <= linear_vel <= MIN_DRIVE_VELOCITY:
			motors.stopcar()
		else: 	
			motors.speed(linear_vel, TEST_print_pwm)
			#motors.go()
			if angular_vel == 0: 
				steer.center()
			else:
				turning_radius = linear_vel/angular_vel
				steer.turn_radius(turning_radius, TEST_print_pwm)
	except ValueError as e:
		print(e)

# switch-off and exit codes

def center_all(): 	# centers all the servos
	cam.center()
	steer.center()

def all_off():		# switches off all the servos and motors 
	pwm.set_all_pwm(0, 0)
	motors.stopcar()
	
def end_switch_off():	# to be added at the end of any script, centers and switches off servos
	center_all()
	time.sleep(0.2)
	all_off()
