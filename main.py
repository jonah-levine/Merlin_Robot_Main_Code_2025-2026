# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Thomas Maynard                                               #
# 	Created:      9/11/2025, 2:50:28 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
import math
import time

# I need to do this everytime I add a new device
brain = Brain()
controller = Controller()
motor1 = Motor(Ports.PORT1)
motor2 = Motor(Ports.PORT2)
motor3 = Motor(Ports.PORT11)
motor4 = Motor(Ports.PORT12)

IntakemotorR = Motor(Ports.PORT6)
IntakemotorL = Motor(Ports.PORT7)
Conveyormotor_A = Motor(Ports.PORT14)
Conveyormotor_B = Motor(Ports.PORT15)
Conveyormotor_C = Motor(Ports.PORT16)
Color_sorting = Motor(Ports.PORT17)

pneumatic_1_OUT = DigitalOut(brain.three_wire_port.a)

rotationL = Rotation(Ports.PORT3)
rotationL.set_position(0, DEGREES) 
rotationL.set_reversed(True)
rotationR = Rotation(Ports.PORT13)
rotationR.set_position(0, DEGREES)

left_side = MotorGroup(motor1, motor2)
right_side = MotorGroup(motor3, motor4)

Color_sensorA = Optical(Ports.PORT18)
Color_sensorB = Optical(Ports.PORT19)
Color_sensorA.set_light(100)
Color_sensorB.set_light(100)

# This is initializing the global variables for position tracking
X_Position = 0
Y_Position = 0
Robot_Angle = 0

# This timer on the sorting motor to let the blocks pass
timer_duration = 5
Start_time = 0
time.time()

# This function allows the driver to control the robot
def Driver_Control():
    global Change_Color_Needed
    Base_speed_Control = 1
    Speed_Control = Base_speed_Control

    global timer_duration
    global Start_time

    while True:

        # This is for controlling the speed of the robot and the number should be between 0 and 1, for 0 to 100%

        # These are the driving controls
        motor1.spin(REVERSE, controller.axis3.position() * Speed_Control, PERCENT)
        motor2.spin(REVERSE, controller.axis3.position() * Speed_Control, PERCENT)
        motor3.spin(FORWARD, controller.axis2.position() * Speed_Control, PERCENT)
        motor4.spin(FORWARD, controller.axis2.position() * Speed_Control, PERCENT)

        # This is for controling the Conveyor belt and the Intake motors
        if controller.buttonL2.pressing():
            Conveyormotor_A.spin(FORWARD, 100, PERCENT)
            Conveyormotor_B.spin(REVERSE, 100, PERCENT)
            Conveyormotor_C.spin(REVERSE, 100, PERCENT)

            Start_time = time.time()

            # This is for sorting the blocks based on color for when the blocks or moving up the converyor
            if 0 < Color_sensorA.hue() < 14 or 0 < Color_sensorB.hue() < 14:
                Color_sorting.spin(REVERSE, 100, PERCENT)

                timer_duration = time.time() - Start_time + 5

            elif 210 < Color_sensorA.hue() < 250 or 210 < Color_sensorB.hue() < 250:
                Color_sorting.spin(FORWARD, 100, PERCENT)

                timer_duration = time.time() - Start_time + 5
            
            elif time.time() - Start_time >= timer_duration:
                Color_sorting.spin(FORWARD, 100, PERCENT)

        elif controller.buttonR2.pressing():
            Conveyormotor_A.spin(REVERSE, 100, PERCENT)
            Conveyormotor_B.spin(FORWARD, 100, PERCENT)
            Conveyormotor_C.spin(FORWARD, 100, PERCENT)

            Start_time = time.time()

            # This is for sorting the blocks based on color for when the blocks or moving down the converyor
            if 0 < Color_sensorA.hue() < 14 or 0 < Color_sensorB.hue() < 14:
                Color_sorting.spin(FORWARD, 100, PERCENT)

                timer_duration = time.time() - Start_time + 5

            elif 210 < Color_sensorA.hue() < 250 or 210 < Color_sensorB.hue() < 250:
                Color_sorting.spin(REVERSE, 100, PERCENT)

                timer_duration = time.time() - Start_time + 5

            elif time.time() - Start_time  >= timer_duration:
                Color_sorting.spin(REVERSE, 100, PERCENT)

        else:
            Conveyormotor_A.stop()
            Conveyormotor_B.stop()
            Conveyormotor_C.stop()
            Color_sorting.stop()

        # This may need to change depending on how the intake is made and if it end up being different
        #if controller.buttonR1.pressing():
            #IntakemotorL.spin(REVERSE, 100, PERCENT)
            #IntakemotorR.spin(FORWARD, 100, PERCENT)

        #elif controller.buttonL1.pressing():
            #IntakemotorL.spin(FORWARD, 100, PERCENT)
            #IntakemotorR.spin(REVERSE, 100, PERCENT) 
        #else:
            #IntakemotorR.stop()
            #IntakemotorL.stop()

        if controller.buttonY.pressing():
            pneumatic_1_OUT.set(False)
        elif  controller.buttonX.pressing():
            pneumatic_1_OUT.set(True)

def Position_Tracking():
    global X_Position
    global Y_Position 
    global Robot_Angle
    # This is the Wheel dimamiter in inches
    Wheel_Dimamiter = 8.25

    # This should be found by measuring the distance from middle of each the tracking wheels
    # This is the distance for the Tracking Center to the Left Tracking wheel
    S_l = 17
    # This is the distance for the Tracking Center to the Left Tracking wheel 
    S_r = 17

    Robot_Angle = 0
    WheelR_Position = 0
    WheelL_Position = 0
    DetlaR = 0
    DetlaL = 0

    while True:
        # This is finding the arcs
        DetlaR = WheelR_Position - (rotationR.position(TURNS)) * (Wheel_Dimamiter*math.pi) 
        DetlaL =  WheelL_Position- (rotationL.position(TURNS)) * (Wheel_Dimamiter*math.pi)  

        # See how much the robot has moved 
        WheelR_Position = (rotationR.position(TURNS)) * (Wheel_Dimamiter*math.pi)
        WheelL_Position = (rotationL.position(TURNS)) * (Wheel_Dimamiter*math.pi)

        # This is finding the Arc's angle to see if it is turning or going straight
        Arc_Angle = (DetlaL - DetlaR) / (S_l + S_r)
        if not Arc_Angle == 0:

            # This is finding the Arc's angle and circle for the position tracking
            R = (DetlaL - DetlaR) / (Arc_Angle*2)
            Arc_Circle = math.sqrt(2*(R**2) * (1-math.cos(Arc_Angle)))

               # This is finding the robot's new position and angle
            if DetlaL + DetlaR < 0:
                X_Position = X_Position - math.cos(Robot_Angle + (Arc_Angle/2)) * Arc_Circle
                Y_Position = Y_Position - math.sin(Robot_Angle + (Arc_Angle/2)) * Arc_Circle
            else:
                X_Position = X_Position + math.cos(Robot_Angle + (Arc_Angle/2)) * Arc_Circle
                Y_Position = Y_Position + math.sin(Robot_Angle + (Arc_Angle/2)) * Arc_Circle

            Robot_Angle = Robot_Angle +  Arc_Angle
        else:
            # This is if the robot is going straight
            X_Position = X_Position + (math.cos(Robot_Angle)*DetlaL)
            Y_Position = Y_Position + (math.sin(Robot_Angle)*DetlaL)


#def PID_Control():
    target   = 1

    K_p = 1
    K_i = 1
    K_d = 1

    while True:
        error_x =  error_x
        error_
        error =  target_Y- Y_Position

        proportioal_error = error_x * K_p








    time.sleep(0.1)

def Drive_to_Coordinates(Traget_Position):  
    global X_Position
    global Y_Position 
    global Robot_Angle
    # Breaking down the target position list in to each step
    for step in Traget_Position:

        # Get the target x, y, and angle from the current tuple
        target_x = step [0]
        target_y = step [1]
        Conveyor_motor_controls = step [2]
        Intake_motor_controls = step [3]
        Wait_time = step [4]

        
        #Calculate distance and angle errors
        distance_error = math.hypot(target_x - X_Position, target_y - Y_Position)
        angle_to_target = math.atan2(target_y - Y_Position, target_x - X_Position)
        angle_error = Robot_Angle - angle_to_target
    
        while not (distance_error < 1 and abs(angle_error) < 1):

            #Calculate distance and angle errors
            distance_error = math.hypot(target_x - X_Position, target_y - Y_Position)
            angle_to_target = math.atan2(target_y - Y_Position, target_x - X_Position)
            angle_error = Robot_Angle - angle_to_target

            # Normalize angle error
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi

            # Drive and turn based on errors
            #turn_power = angle_error * 10.0
            #drive_power = distance_error * 0.1
    
            #motor1.spin(FORWARD, drive_power - turn_power, PERCENT)
            #motor2.spin(FORWARD, drive_power - turn_power, PERCENT)
            #motor3.spin(FORWARD, drive_power + turn_power, PERCENT)
            #motor4.spin(FORWARD, drive_power + turn_power, PERCENT) 

        # Turns the conveyor and intake motor on for the specified wait time
        while Wait_time > 0:
            if Conveyor_motor_controls == 1:
                Conveyormotor_A.spin(FORWARD, 100, PERCENT)
                Conveyormotor_B.spin(FORWARD, 100, PERCENT)
            elif Conveyor_motor_controls == -1:
                Conveyormotor_A.spin(REVERSE, 100, PERCENT)
                Conveyormotor_B.spin(FORWARD, 100, PERCENT)
            if Intake_motor_controls == 1:
                IntakemotorL.spin(FORWARD, 100, PERCENT)
                IntakemotorR.spin(FORWARD, 100, PERCENT)
            elif Intake_motor_controls == -1:
                IntakemotorL.spin(REVERSE, 100, PERCENT)
                IntakemotorR.spin(REVERSE, 100, PERCENT)
            time.sleep(1)
            Wait_time -= 1

        # Stop all motors when time is up
        Conveyormotor_A.stop()
        Conveyormotor_B.stop()
        IntakemotorL.stop()
        IntakemotorR.stop()

# This function updates the controller screen with the robot's position and angle
def colltroller_sceen_refresh():  
    while True:
        controller.screen.clear_screen()

        # These are the labels of each of the values
        controller.screen.set_cursor(1,1) 
        controller.screen.print("X Position")
        controller.screen.set_cursor(2,1)
        controller.screen.print("Y Position")
        controller.screen.set_cursor(3,1)
        controller.screen.print("Robot Angle")

        # These are the values for the controller screen
        controller.screen.set_cursor(1,12)
        controller.screen.print(X_Position)
        controller.screen.set_cursor(2,12)
        controller.screen.print(Y_Position)
        controller.screen.set_cursor(3,13)
        controller.screen.print(Robot_Angle)
        
        brain.screen.set_cursor(2,1)
        brain.screen.print(Color_sensorA.hue())
        brain.screen.set_cursor(3,1)
        brain.screen.print(Color_sensorB.hue())

        print(Color_sensorA.hue())
        print(Color_sensorA.hue())

        time.sleep(0.01)

# Define a function "driver".
def driver():
    brain.screen.clear_screen()
    # Start driver controls in a background thread (Thread auto-starts in VEX API)
    Thread(Driver_Control)
    Thread(Position_Tracking)
    Thread(colltroller_sceen_refresh)

    # Print to the brain screen to see if all of the systems are go
    brain.screen.print("Driver Period has Started")

# This is an example autonomous function
def Example_autonomous():
    # This is driving to 20 inches forward and facing 0 degrees
    # Again, when make a anton the X and Y coordinates need to be in inches and X is forward and back and Y is left and right
    # Again, When make a anton The angle needs to be in degrees
    Traget_Position = [(20, 0, 0 ,0, 0) ,(0, 0, 0 ,0, 0)]
    Drive_to_Coordinates(Traget_Position)

# Define a function "autonomous".  
def autonomous():
    brain.screen.clear_screen() 
    Thread(Position_Tracking) 
    Thread(colltroller_sceen_refresh)

    # Print to the brain screen to see if all of the systems are go
    brain.screen.print("Autonomous Period has Started")

    # When make a anton the X and Y coordinates need to be in inches and X is forward and back and Y is left and right
    # When make a anton The angle needs to be in degrees
    # Example: Drive to coordinates
    #Uncomment the autonomous you want to run and remember to download the code again to the robot

    Example_autonomous()

# Construct a Competition Control object "competition"
# with the Competition class.
competition = Competition(driver, autonomous)