"""mavic2ros controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import InertialUnit
from controller import Gyro
from controller import Keyboard
from controller import Motor
import math
import rospy
import os

from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import String
from sensor_msgs.msg import Image

# create the Robot instance.
robot = Robot()

pitch_disturbance = 0
roll_disturbance = 0
yaw_disturbance = 0
def crying_orangutan(data):
    # pitch - data.data is either 1,0,-1
    global pitch_disturbance
    pitch_disturbance = 0.261799*data.data
    pass
    
def crying_polar_bears(data):
    global roll_disturbance
    roll_disturbance = 0.261799*data.data
    pass
    
def crying_animals_in_general(data):
    global yaw_disturbance
    yaw_disturbance = 5*data.data
    pass
    
def crying_mother_earth_in_general(data):
    global altitude
    global target_altitude
    if data.data==1:
        target_altitude = altitude+0.1
    elif data.data==-1:
        target_altitude = altitude+0.1
        
    pass
    
    
# get the time step of the current world.
timeStep = int(robot.getBasicTimeStep())
print("Time Step is: "+str(timeStep))
# Get and enable devices.
rospy.init_node('python_submarine_controller', anonymous=True) # node is called 'python_webots_controller'
rospy.loginfo("Loading Webots Controller")
pub = rospy.Publisher('imu_values_topic', Vector3, queue_size=10)
depth_pub = rospy.Publisher('depth_topic', Float32, queue_size=10)
log_pub = rospy.Publisher('python_submarine_logger', String, queue_size=10)
camera_pub = rospy.Publisher('python_submarine_camera_images', Image, queue_size=10)
rearcamera_pub = rospy.Publisher('python_submarine_rear_camera_images', Image, queue_size=10)
bleh_pub = rospy.Publisher("python_submarine_heading_speed",Float32,queue_size=10)
speed_pub = rospy.Publisher('python_submarine_speeds', Vector3, queue_size=10)


rospy.Subscriber("pitch_control_input", Int16, crying_orangutan)
rospy.Subscriber("roll_control_input", Int16, crying_polar_bears)
rospy.Subscriber("heading_control_input", Int16, crying_animals_in_general)
rospy.Subscriber("altitude_control_input", Int16, crying_mother_earth_in_general)

IMUsensor = robot.getInertialUnit('inertial unit')  # front central proximity sensor
IMUsensor.enable(timeStep)

GPSsensor = robot.getGPS('gps')
GPSsensor.enable(timeStep)

GYROsensor = robot.getGyro("gyro")
GYROsensor.enable(timeStep)

KeyB = robot.getKeyboard()
KeyB.enable(timeStep)

front_left_motor = robot.getMotor("front left thruster")
front_right_motor = robot.getMotor("front right thruster")
rear_left_motor = robot.getMotor("rear left thruster")
rear_right_motor = robot.getMotor("rear right thruster")
front_left_motor.setPosition(float('inf'))
front_right_motor.setPosition(float('inf'))
rear_left_motor.setPosition(float('inf'))
rear_right_motor.setPosition(float('inf'))
front_left_motor.setVelocity(0.0)
front_right_motor.setVelocity(0.0)
rear_left_motor.setVelocity(0.0)
rear_right_motor.setVelocity(0.0)

camera = robot.getCamera("camera")
camera.enable(timeStep)

rearcamera = robot.getCamera("rearcamera")
rearcamera.enable(timeStep)

FL_wheel = robot.getMotor("fl wheel")
FR_wheel = robot.getMotor("fr wheel")
RL_wheel = robot.getMotor("rl wheel")
RR_wheel = robot.getMotor("rr wheel")
FL_wheel.setPosition(float('inf'))
FR_wheel.setPosition(float('inf'))
RL_wheel.setPosition(float('inf'))
RR_wheel.setPosition(float('inf'))
FL_wheel.setVelocity(0.0)
FR_wheel.setVelocity(0.0)
RL_wheel.setVelocity(0.0)
RR_wheel.setVelocity(0.0)

fly_wheel = robot.getMotor("flywheel")
fly_wheel.setPosition(float('inf'))
fly_wheel.setVelocity(0.0)

k_roll_p = 200.0           # P constant of the roll PID.
k_pitch_p = 200.0         # P constant of the pitch PID.
k_roll_d = 50.0
k_pitch_d = 50.0
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
target_altitude = 5.0
k_vertical_thrust = 67.1 # with this thrust, the drone lifts.
k_vertical_offset = 0.1   # Vertical offset where the robot actually targets to stabilize itself.
k_vertical_p = 10.0        # P constant of the vertical PID.
k_vertical_d = 13000

def CLAMP(value, low, high):
    if value < low:
        return low
    elif value > high:
        return high
    return value

robot.step(timeStep)
xpos, altitude , zpos = GPSsensor.getValues()
xpos_old=xpos
altitude_old=altitude
zpos_old=zpos
roll_vel_old=0
lock_on = False
depth_msg = Float32()
pi = math.pi
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timeStep) != -1:
    # Read the sensors:
    roll, pitch, heading = IMUsensor.getRollPitchYaw() 
    xpos, altitude , zpos = GPSsensor.getValues()
    roll_vel, bleh, pitch_vel =GYROsensor.getValues()
    #print(str(roll_vel)+"\t"+str(pitch_vel))
    littleTimeStep = timeStep/1000.0
    xSpeed=(xpos-xpos_old)/littleTimeStep
    ySpeed=(altitude-altitude_old)/timeStep
    zSpeed=(zpos-zpos_old)/littleTimeStep
    #print(str(xSpeed)+"\t"+str(ySpeed)+"\t"+str(zSpeed))
    xpos_old=xpos
    altitude_old=altitude
    zpos_old=zpos
    #  val = ds.getValue()
    left=0
    right=0
    
    ## Now we send some things to ros BELOW
    camera_image_msg = Image()
    camera_image_msg.width = 320
    camera_image_msg.height = 240
    camera_image_msg.encoding = "bgra8"
    camera_image_msg.is_bigendian = 1
    camera_image_msg.step = 1280
    camera_image_msg.data = camera.getImage()
    camera_pub.publish(camera_image_msg)
    
    
    ## Now we send some things to ros BELOW
    rearcamera_image_msg = Image()
    rearcamera_image_msg.width = 320
    rearcamera_image_msg.height = 240
    rearcamera_image_msg.encoding = "bgra8"
    rearcamera_image_msg.is_bigendian = 1
    rearcamera_image_msg.step = 1280
    rearcamera_image_msg.data = rearcamera.getImage()
   
            
    #rearcamera_image_msg.data = flat_list
            
    rearcamera_pub.publish(rearcamera_image_msg)
    
    depth_msg.data = altitude
    depth_pub.publish(depth_msg)
    
    radcoeff = 180.0/pi
    # Process sensor data here.
    #rospy.loginfo("Sending Simulated IMU Data. Roll: "+str(round(roll*radcoeff))+" Pitch: "+str(round(pitch*radcoeff))+" Heading: "+str(round(heading*radcoeff)))
    pub.publish(Vector3(roll*radcoeff*-1,pitch*radcoeff*-1,heading*radcoeff*-1))
    speed_pub.publish(Vector3(math.cos(heading)*xSpeed*-1+math.sin(heading)*zSpeed*-1,ySpeed,math.sin(heading)*xSpeed+math.cos(heading)*zSpeed))
    log_pub.publish(str(round(roll*radcoeff))+","+str(round(pitch*radcoeff))+","+str(round(heading*radcoeff))+","+str(altitude)+","+str(roll_vel)+","+str(bleh)+","+str(pitch_vel)+","+str(xSpeed)+","+str(ySpeed)+","+str(zSpeed))
    
    bleh_pub.publish(bleh)
    
    front_left_motor_input = 0
    front_right_motor_input = 0
    rear_left_motor_input = 0
    rear_right_motor_input = 0
    spin_boy = 0
    key=KeyB.getKey()
    while (key>0):
        if (key==KeyB.UP):
            #pitch_disturbance = 2.0
            left = 10
            right= 10
        if (key==KeyB.DOWN):
            #pitch_disturbance = -2.0
            left = -10
            right= -10
        if (key==KeyB.LEFT):
            #roll_disturbance = 1.0
            left = 10
            right= -10
        if (key==KeyB.RIGHT):
            #roll_disturbance = -1.0
            left = -10
            right= 10
        if (key==KeyB.HOME):
            front_left_motor_input = 100
            front_right_motor_input = 100
            rear_left_motor_input = 100
            rear_right_motor_input = 100
        if (key==KeyB.END):
            spin_boy = 1000
        if (key==ord('W')):
            pitch_disturbance = 0.261799
        if (key==ord('A')):
            roll_disturbance = 0.261799
        if (key==ord('S')):
            pitch_disturbance = -0.261799
        if (key==ord('D')):
            roll_disturbance = -0.261799
        if (key==ord('Q')):
            yaw_disturbance = 5
        if (key==ord('E')):
            yaw_disturbance = -5
        if (key==ord('Z')):
            target_altitude = altitude+0.1
        if (key==ord('X')):
            target_altitude = altitude-0.1
        if (key==ord('L')):
            lock_on = True
        if (key==ord('R')):
            roll_disturbance = -3
        key=KeyB.getKey()
    yaw_disturbance
    # Process sensor data here.
    #print(str(roll)+"\t"+str(pitch)+"\t"+str(heading))
    #print(str(roll)+"\t"+str(roll_vel))
    if abs(roll_vel_old-roll_vel)>2:
        k_roll_d=0
    else:
        k_roll_d=100.0
    roll_input = k_roll_p * (roll_disturbance-roll) - k_roll_d*roll_vel
    roll_vel_old=roll_vel
    pitch_input = (k_pitch_p *(pitch_disturbance-pitch) - k_pitch_d*pitch_vel)
    yaw_input = yaw_disturbance;
    #print("pitch_input: "+str(pitch_input)+"\t velocity: "+str(pitch_vel))
    
    vertical_input = k_vertical_p *CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0)-k_vertical_d*ySpeed;
    if roll>math.pi/2 or roll<-math.pi/2:
        vertical_input=-vertical_input
        k_vertical_thrust=-67.1
    else:
        k_vertical_thrust=67.1
    #vertical_input = 0# k_vertical_p * pow(clamped_difference_altitude, 3.0);
    #0.2635 #0.266  #0.2635 #0.266  #roll distance
    #0.3582 #0.3582 #0.3346 #0.3346 #pitch distance
    #print(str(k_vertical_thrust)+"\t"+str(vertical_input)+"\t"+str(roll_input)+"\t"+str(pitch_input))
    if (lock_on==False):
        front_left_motor_input = k_vertical_thrust + vertical_input  + roll_input + pitch_input + yaw_input
        front_right_motor_input=(k_vertical_thrust + vertical_input) - roll_input + pitch_input - yaw_input
        rear_left_motor_input  =(k_vertical_thrust + vertical_input) + roll_input - pitch_input - yaw_input
        rear_right_motor_input =(k_vertical_thrust + vertical_input) - roll_input - pitch_input + yaw_input
    else:
        front_left_motor_input =-100
        front_right_motor_input=-100
        rear_left_motor_input  =-100
        rear_right_motor_input =-100
        lock_on= False
    clampval = 200
    #print(str(front_left_motor_input)+"\t"+str(front_right_motor_input)+"\t"+str(rear_left_motor_input)+"\t"+str(rear_right_motor_input))
    front_left_motor.setVelocity(CLAMP(-front_left_motor_input,-clampval,clampval))#positive is up  #0.44467908653
    front_right_motor.setVelocity(CLAMP(front_right_motor_input,-clampval,clampval))#negative is up #0.44616503673
    rear_left_motor.setVelocity(CLAMP(rear_left_motor_input,-clampval,clampval))#negative is up     #0.42589835641
    rear_right_motor.setVelocity(CLAMP(-rear_right_motor_input,-clampval,clampval))#positive is up  #0.42744959936
    fly_wheel.setVelocity(spin_boy)
    FL_wheel.setVelocity(left)
    FR_wheel.setVelocity(right)
    RL_wheel.setVelocity(left)
    RR_wheel.setVelocity(right)
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    
    
    pass

# Enter here exit cleanup code.
