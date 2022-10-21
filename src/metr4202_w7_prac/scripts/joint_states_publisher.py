#!/usr/bin/env python3
"""
This script publishes a set of random joint states to the dynamixel controller.
Use this to get an idea of how to code your inverse kinematics!
"""

# Funny code
from cmath import pi
from ctypes.wintypes import PSIZE
from multiprocessing.dummy import Array
import random
from sre_parse import State
import numpy as np

# Always need this
import rospy

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from fiducial_msgs.msg import FiducialTransformArray


# imports for gripper
import pigpio


import math


"""

def zRotMatrix(theta):
    R = np.array([[np.cos(theta), -np.sin(theta), 0],
                  [np.sin(theta), np.cos(theta), 0],
                  [0,0,1]])
    return

#makes a transformation matrix
def RpToTrans(R, p):
    return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]

# makes a transformation from the camera to the centre of the cube    
def transCamCen(theta, p):
    I = np.eye(3)

    # Tag side length
    L = 28 #mm
    pcor_cen = np.array([L/2, L/2, 0])
    # transformation from corner of tag to centre
    Tcor_cen = RpToTrans(I, pcor_cen)

    R_cam_cor = zRotMatrix(theta)
    Tcam_cor = RpToTrans(R_cam_cor, p)
    # Transformation from camera to corner of arucotag
    Tcam_cen = np.dot(Tcam_cor, Tcor_cen)
    return Tcam_cen
   
   
"""    
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

##finds the inverse kinematics angles for the robot to get to x,y,z 
#zero position at joint 1 with x and y pos as written on robot
def invk2(x,y,z):


    #robot arm lengths
    L1 = 95 
    L2 = 117.5 
    L3 = 95 
    L4 = 80


    phi = -np.array((0,1,2,3,4))*np.pi/8 #angle the gripper makes from horizontal
    solutions = list()

    theta1 = np.arctan2(y,x)+np.pi/2


    for angle in phi: #iterates over the gripper angles
        pwx = np.sqrt(x**2+y**2) - L4*np.cos(angle) #distance of of joint 4 on x-y plane from zero position
        pwy = z - L4*np.sin(angle) - L1 #height position of joint 4
 

        if (np.sqrt(pwx**2 + pwy**2) <= L2+L3): #condition for solution to exist

            c2 = (pwx**2 + pwy**2 - L2**2 - L3**2)/(2*L2*L3) #cosine rule



            v3_1 = -np.arccos(c2)  #elbow-up position
            v3_2 = +np.arccos(c2)  #elbow-down position

            alpha = np.arctan2(pwy,pwx) #angle from zero position to joint 4
            beta = np.arccos((pwx**2+pwy**2+L2**2-L3**2)/(2 * L2 *np.sqrt(pwx**2 + pwy**2 )))
  
            v2_1 = alpha + beta #elbow up
            v2_2 = alpha - beta #elbow down

            v4_1 = angle - v2_1 - v3_1 #elbow up
            v4_2 = angle - v2_2 - v3_2 #elbow down

            #our robot zero is vertical not horizontal so take the compliment angle 
            theta2_1 = np.pi/2 - v2_1
            theta2_2 = np.pi/2 - v2_2

            # i think the joint_3 and joint_4 angles are the same as in the book
            # might need to change depending on which way the robot joints actually move icr
            theta3_1 = -v3_1
            theta3_2 = -v3_2
            theta4_1 = -v4_1
            theta4_2 = -v4_2

            #check that solutions 1 and 2 are within joint angle contraints
            if (np.abs(theta2_1)< 1.7 and np.abs(theta3_1)<2.3 and np.abs(theta4_1)<1.6):
                solutions.append(np.array((theta1, theta2_1, theta3_1, theta4_1)))

            # have no added the elbow down solutions as they can cause collision with ground
            '''if (np.abs(theta2_2)<1.7 and np.abs(theta3_2)<1.7 and np.abs(theta4_2)<1.7):
                solutions.append(np.array((theta1, theta2_2, theta3_2, theta4_2)))
            '''

    # if not(bool(solutions)): # if no solution is found return FALSE
        #rospy.loginfo("Position value (" + x + ", "+ y +", " z + ") is invalid. Limits are 1000-2000.")
        # return(False)
            
    #finds the best solution - defined for now as least total angle of robot
    #I think a better definition would be smallest (joints_now - solution) so robots moves the least
    best_solution = solutions[0]
    for solution in solutions:

        if (solution[0] > np.pi): # if the theta1 is too big we can rotate pi rad and flip the angles
            solution[0] = solution[0] - np.pi
            solution[1] = -solution[1]
            solution[2] = -solution[2]
            solution[3] = -solution[3]

        if (solution[0] < -np.pi): #same as above for large negative theta1
            solution[0] = solution[0] + np.pi
            solution[1] = -solution[1]
            solution[2] = -solution[2]
            solution[3] = -solution[3]

        if (np.sum(np.abs(solution)) < np.sum(np.abs(best_solution))):
            best_solution = solution

    # Create message of type JointState
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4'],
        position=[best_solution[0], best_solution[1], best_solution[2], best_solution[3]],
        #velocity=[0.4, 0.4, 0.4, 0.4],
        #effort=[20, 20, 20, 20]
    )
    
    return(msg)

class Cube:
    def __init__(self, id):
        self.id = id
        self.history = []
        self.last_detected_seq = -1

    def update_pos(self, x, y, z):
        if len(self.history) == 5:
            temp = self.history.pop(0)

        self.history.append([x, y, z])

# This one doesn't actually do it though...
def invk_cb(pose: Pose):
    global pub
    global desired_joint_angles
    #rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
    
    desired_jstate = invk2(pose.position.x, pose.position.y, pose.position.z)
    desired_joint_angles = desired_jstate.position
    
<<<<<<< HEAD
    """    
    def euler_from_quaternion(x, y, z, w):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return roll_x, pitch_y, yaw_z # in radians

    ##finds the inverse kinematics angles for the robot to get to x,y,z 
    #zero position at joint 1 with x and y pos as written on robot
    def invk2(x,y,z):


        #robot arm lengths
        L1 = 95 
        L2 = 117.5 
        L3 = 95 
        L4 = 80


        phi = -np.array((0,1,2,3,4))*np.pi/8 #angle the gripper makes from horizontal
        solutions = list()

        theta1 = np.arctan2(y,x)+np.pi/2


        for angle in phi: #iterates over the gripper angles
            pwx = np.sqrt(x**2+y**2) - L4*np.cos(angle) #distance of of joint 4 on x-y plane from zero position
            pwy = z - L4*np.sin(angle) - L1 #height position of joint 4
    

            if (np.sqrt(pwx**2 + pwy**2) <= L2+L3): #condition for solution to exist

                c2 = (pwx**2 + pwy**2 - L2**2 - L3**2)/(2*L2*L3) #cosine rule



                v3_1 = -np.arccos(c2)  #elbow-up position
                v3_2 = +np.arccos(c2)  #elbow-down position

                alpha = np.arctan2(pwy,pwx) #angle from zero position to joint 4
                beta = np.arccos((pwx**2+pwy**2+L2**2-L3**2)/(2 * L2 *np.sqrt(pwx**2 + pwy**2 )))
    
                v2_1 = alpha + beta #elbow up
                v2_2 = alpha - beta #elbow down

                v4_1 = angle - v2_1 - v3_1 #elbow up
                v4_2 = angle - v2_2 - v3_2 #elbow down

                #our robot zero is vertical not horizontal so take the compliment angle 
                theta2_1 = np.pi/2 - v2_1
                theta2_2 = np.pi/2 - v2_2

                # i think the joint_3 and joint_4 angles are the same as in the book
                # might need to change depending on which way the robot joints actually move icr
                theta3_1 = -v3_1
                theta3_2 = -v3_2
                theta4_1 = -v4_1
                theta4_2 = -v4_2

                #check that solutions 1 and 2 are within joint angle contraints
                if (np.abs(theta2_1)< 1.7 and np.abs(theta3_1)<2.3 and np.abs(theta4_1)<1.6):
                    solutions.append(np.array((theta1, theta2_1, theta3_1, theta4_1)))

                # have no added the elbow down solutions as they can cause collision with ground
                '''if (np.abs(theta2_2)<1.7 and np.abs(theta3_2)<1.7 and np.abs(theta4_2)<1.7):
                    solutions.append(np.array((theta1, theta2_2, theta3_2, theta4_2)))
                '''

        # if not(bool(solutions)): # if no solution is found return FALSE
            #rospy.loginfo("Position value (" + x + ", "+ y +", " z + ") is invalid. Limits are 1000-2000.")
            # return(False)
                
        #finds the best solution - defined for now as least total angle of robot
        #I think a better definition would be smallest (joints_now - solution) so robots moves the least
        best_solution = solutions[0]
        for solution in solutions:

            if (solution[0] > np.pi): # if the theta1 is too big we can rotate pi rad and flip the angles
                solution[0] = solution[0] - np.pi
                solution[1] = -solution[1]
                solution[2] = -solution[2]
                solution[3] = -solution[3]

            if (solution[0] < -np.pi): #same as above for large negative theta1
                solution[0] = solution[0] + np.pi
                solution[1] = -solution[1]
                solution[2] = -solution[2]
                solution[3] = -solution[3]

            if (np.sum(np.abs(solution)) < np.sum(np.abs(best_solution))):
                best_solution = solution

        # Create message of type JointState
        msg = JointState(
            # Set header with current time
            header=Header(stamp=rospy.Time.now()),
            # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
            name=['joint_1', 'joint_2', 'joint_3', 'joint_4'],
            position=[best_solution[0], best_solution[1], best_solution[2], best_solution[3]],
            #velocity=[0.4, 0.4, 0.4, 0.4],
            #effort=[20, 20, 20, 20]
        )
        
        return(msg)

    # This one doesn't actually do it though...
    def invk_cb(self, pose: Pose):
        #rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
        
        desired_jstate = self.invk2(pose.position.x, pose.position.y, pose.position.z)
        self.desired_joint_angles = desired_jstate.position
        
        print(f"from invk{self.desired_joint_angles}")
        self.pub.publish(desired_jstate)

    # grippper callback checks if grip value is not outside of limits and sets it
    def gripper_cb(self, gripValue: Float32):
        # check limits so servo doesnt break
        if (gripValue.data < 1000 or gripValue.data > 2000):
            print(f"Gripper value ({gripValue.data}) is invalid. Limits are 1000-2000.")
        else:
            self.rpi.set_servo_pulsewidth(18,gripValue.data) 
            print(f"Gripper state value changed to: + {gripValue.data}")

    def acuro_cb(self, data: FiducialTransformArray):
        # NOTE: must add ignorance to arm tag
        ARM_ID = 0

        # if there is a cube detected, transforms array > 0
        if len(data.transforms) > 0:
            # get main cube information
            tagId = data.transforms[0].fiducial_id
            tagPos = data.transforms[0].transform.translation
            tagOrient = data.transforms[0].transform.rotation
            
            x, y, z = self.euler_from_quaternion(tagOrient.x, tagOrient.y, tagOrient.z, tagOrient.w)
            #print(f"EULER x:{x}, y:{y}, z:{z}")
            #print(f"Data{data.transforms[0]}")
            
            """
            p = np.array([tagPos.x, tagPos.y, tagPos.z])
            print(f"this is p: {p}")
            #transformation from camera to centre of cube
            Tcam_cen = transCamCen(z, p)
            print(f"Transformation from cam to centre T{Tcam_cen}")
            """
            
            if tagId != ARM_ID:

                # check if cube in cubelist already
                if tagId not in self.cubes.keys():
                    newCube = Cube(tagId)
                    self.cubes[tagId] = newCube

                # transformation of camera from arm base
                Tsc = np.array([
                    [1, 0, 0, 0],
                    [0, 1, 0, -150],
                    [0, 0, -1, 420],
                    [0, 0, 0, 1]
                ])

                # convert cube position data 
                V = np.dot(Tsc, np.array([tagPos.x*1000, tagPos.y*1000, tagPos.z*1000, 1]))

                # update cube position relative to arm base
                self.cubes.get(tagId).update_pos(V[0], V[1], V[2])
            
        pass
=======
    print(f"from invk{desired_joint_angles}")
    pub.publish(desired_jstate)

# grippper callback checks if grip value is not outside of limits and sets it
def gripper_cb(gripValue: Float32):
    global rpi
    # check limits so servo doesnt break
    if (gripValue.data < 1000 or gripValue.data > 2000):
        print(f"Gripper value ({gripValue.data}) is invalid. Limits are 1000-2000.")
    else:
        rpi.set_servo_pulsewidth(18,gripValue.data) 
        print(f"Gripper state value changed to: + {gripValue.data}")

def acuro_cb(data: FiducialTransformArray):
    global cubes
    # NOTE: must add ignorance to arm tag
    ARM_ID = 0

    # if there is a cube detected, transforms array > 0
    if len(data.transforms) > 0:
        # get main cube information
        tagId = data.transforms[0].fiducial_id
        tagPos = data.transforms[0].transform.translation
        tagOrient = data.transforms[0].transform.rotation
        
        x, y, z = euler_from_quaternion(tagOrient.x, tagOrient.y, tagOrient.z, tagOrient.w)
        #print(f"EULER x:{x}, y:{y}, z:{z}")
        #print(f"Data{data.transforms[0]}")
        
        """
        p = np.array([tagPos.x, tagPos.y, tagPos.z])
        print(f"this is p: {p}")
        #transformation from camera to centre of cube
        Tcam_cen = transCamCen(z, p)
        print(f"Transformation from cam to centre T{Tcam_cen}")
        """
        
        if tagId != ARM_ID:

            # check if cube in cubelist already
            if tagId not in cubes.keys():
                newCube = Cube(tagId)
                cubes[tagId] = newCube

            # transformation of camera from arm base
            Tsc = np.array([
                [1, 0, 0, 0],
                [0, 1, 0, -150],
                [0, 0, -1, 420],
                [0, 0, 0, 1]
            ])

            # convert cube position data 
            V = np.dot(Tsc, np.array([tagPos.x*1000, tagPos.y*1000, tagPos.z*1000, 1]))

            # update cube position relative to arm base
            cubes.get(tagId).update_pos(V[0], V[1], V[2])
        
    pass
>>>>>>> parent of e28c6df... made class to avoid global variables

def joint_state_cb(data:JointState):
    global current_joint_angles
    #print(data)
    if len(data.position) == 4:
        #print(data)
<<<<<<< HEAD
        if len(data.position) == 4:
            #print(data)
            # data is returned as [j4, j3, j2, j1] but desired posed saved as [j1, j2, j3, j4]
            self.current_joint_angles = list(data.position)
            self.current_joint_angles.reverse()

    def move_to_home(self):        
        msg = Pose()
        msg.position.x = 100
        msg.position.y = 100
        msg.position.z = 100
        self.desired_pose_pub.publish(msg)
        self.check_arm_in_place()
        
    def check_arm_in_place(self):
        arm_in_place = False
        
        while(not arm_in_place):
            #print(f"desired: {desired_joint_angles} current: {current_joint_angles}")
            diff_j1 = np.abs(self.desired_joint_angles[0] - self.current_joint_angles[0])
            diff_j3 = np.abs(self.desired_joint_angles[2] - self.current_joint_angles[2])
            diff_j2 = np.abs(self.desired_joint_angles[1] - self.current_joint_angles[1])
            diff_j4 = np.abs(self.desired_joint_angles[3] - self.current_joint_angles[3])
        
            #print(f"j1:{diff_j1} j2:{diff_j2} j3:{diff_j3} j4:{diff_j4}")
            if diff_j1 < 0.5 and diff_j2 < 0.5 and diff_j3 < 0.5 and diff_j4 < 0.5:
                arm_in_place = True

    def pickup_cube(self, cube: Cube):

        cube_last_pos = cube.history[-1]
=======
        # data is returned as [j4, j3, j2, j1] but desired posed saved as [j1, j2, j3, j4]
        current_joint_angles = list(data.position)
        current_joint_angles.reverse()

def init():
    # Initialise node with any node name
    rospy.init_node('metr4202_w7_prac')
    global pub
    global desired_pose_pub
    global gripper_pub
    global rpi

    rpi = pigpio.pi()
    rpi.set_mode(18, pigpio.OUTPUT)

    # Create publisher
    pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState,             # Message type
        queue_size=10           # Topic size (optional)
    )
    
    # publisher for desired pose
    desired_pose_pub = rospy.Publisher(
        'desired_pose',
        Pose,
        queue_size=1
    )

    # publisher for gripper
    gripper_pub = rospy.Publisher(
        'gripper_value',
        Float32,
        queue_size=10
    )

    # subscriber for desired pose
    sub = rospy.Subscriber(
        'desired_pose',     # Topic name
        Pose,               # Message type
        invk_cb             # Callback function (required)
    )

    # subscriber for gripper
    grip_sub = rospy.Subscriber(
        'gripper_value',    # topic name
        Float32,            # msg type
        gripper_cb          # callback function
    )
    # NOTE: gripper values 
    #1000 is the closed position
    #1500 is the grip box position
    #2000 is the open position


    # subscriber for fiducial transforms
    tag_tracker = rospy.Subscriber(
        'fiducial_transforms',
        FiducialTransformArray,
        acuro_cb
    )

    # subscriber for joint_states
    joint_state_sub = rospy.Subscriber(
        'joint_states',
        JointState,
        joint_state_cb
    )

    global cubes        # dictionary of cubes detected in the system
    global states       # dictionary of states
    global state        # current state of arm
    cubes = {}

    states = {
        "PREDICTION" : 0,
        "HOMESTATE" : 1,
        "PICKUP" : 2,
        "COLOUR_CHECK" : 3,
        "DROP_OFF" : 4
    }

    # begin project in homestate
    state = states["HOMESTATE"]

def move_to_home():
    global desired_pose_pub
    global desired_joint_angles
    global current_joint_angles
    
    msg = Pose()
    msg.position.x = 100
    msg.position.y = 100
    msg.position.z = 100
    desired_pose_pub.publish(msg)
    arm_in_place = False
    
    while(not arm_in_place):
        #print(f"desired: {desired_joint_angles} current: {current_joint_angles}")
        diff_j1 = np.abs(desired_joint_angles[0] - current_joint_angles[0])
        diff_j2 = np.abs(desired_joint_angles[1] - current_joint_angles[1])
        diff_j3 = np.abs(desired_joint_angles[2] - current_joint_angles[2])
        diff_j4 = np.abs(desired_joint_angles[3] - current_joint_angles[3])
    
        #print(f"j1:{diff_j1} j2:{diff_j2} j3:{diff_j3} j4:{diff_j4}")
        if diff_j1 < 0.5 and diff_j2 < 0.5 and diff_j3 < 0.5 and diff_j4 < 0.5:
            arm_in_place = True
    
    print("got home")
    
def pickup_cube(cube: Cube):
    global desired_pose_pub
    global gripper_pub
    global desired_joint_angles
    global current_joint_angles
>>>>>>> parent of e28c6df... made class to avoid global variables

    cube_last_pos = cube.history[-1]

<<<<<<< HEAD
        self.desired_pose_pub.publish(msg)

        self.check_arm_in_place()
=======
    # send desired position to desired pose topic
    msg = Pose()
    msg.position.x = cube_last_pos[0]
    msg.position.y = cube_last_pos[1]
    msg.position.z = cube_last_pos[2]

    desired_pose_pub.publish(msg)
>>>>>>> parent of e28c6df... made class to avoid global variables

    arm_in_place = False

    while(not arm_in_place):
        global desired_joint_angles
        print(f"desired: {desired_joint_angles} current: {current_joint_angles}")
        diff_j1 = np.abs(desired_joint_angles[0] - current_joint_angles[0])
        diff_j2 = np.abs(desired_joint_angles[1] - current_joint_angles[1])
        diff_j3 = np.abs(desired_joint_angles[2] - current_joint_angles[2])
        diff_j4 = np.abs(desired_joint_angles[3] - current_joint_angles[3])
    
        #print(f"j1:{diff_j1} j2:{diff_j2} j3:{diff_j3} j4:{diff_j4}")
        if diff_j1 < 0.5 and diff_j2 < 0.5 and diff_j3 < 0.5 and diff_j4 < 0.5:
            arm_in_place = True

    # grab box (1500 value)
    gripper_pub.publish(Float32(1250))
    print("closed gripper")

def main():
    # initialise nodes and gripper
    init()

    # global variables
    global current_joint_angles     # array of current angles in radians
    global desired_joint_angles     # array of desired angles in radians
    global cubes                    # dictionary of cubes detected in the system
    global state
    global states

    # add initial delay so dynamixel can load
    rospy.sleep(3)
    
    
    testSpeed = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        print(state)
        if state == states.get("PREDICTION"):
            # implementation 1:
            # detect when cubes have stopped and detect from there
            stopped = False
            
            # check if there has been a cube added to the system
            if len(cubes) > 0:
                id, cube = list(cubes.items())[0]
                if len(cube.history) == 5:
                    current = cube.history[0]
                    oldest = cube.history[4]

                    dist = np.sqrt((current[0]-oldest[0])**2 + (current[1]-oldest[1])**2 + (current[2]-oldest[2])**2)
                    
                    if dist < 5:
                        stopped = True

            if stopped:
                # change to PICKUP state
                state = states["PICKUP"]

        elif state == states.get("HOMESTATE"):
            # TODO:
            #   - 
            move_to_home()
            state = states["PREDICTION"]

        elif state == states.get("PICKUP"):
            # TODO:
            #   - check which block to pickup
            #   - pickup block
            #   - move out of the way
            #   - move

            # CURRENT IMPLEMENTATION: pickup the first box
            id, cube = list(cubes.items())[0]
            pickup_cube(cube)
            state = states["COLOUR_CHECK"]

        elif state == states.get("COLOUR_CHECK"):
            # TODO:
            #   - check if the block has been picked up
            #   - check the colour of block
            #   - drop block for designated colour
            #   - remove from dictionary of blocks

            move_to_home()
            state = states["DROP_OFF"]

        elif state == states.get("DROP_OFF"):
            # TODO: drop at detected colour
            # temporaryly move to home position and drop it there
            move_to_home()
            gripper_pub.publish(Float32(2000))
            state = states["HOMESTATE"]
        # You spin me right round baby, right round...
        # Just stops Python from exiting and executes callbacks
        testSpeed.sleep()
          
    rospy.spin()

if __name__ == '__main__':
    main()

