#!/usr/bin/env python3

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
from std_msgs.msg import ColorRGBA


# imports for gripper
import pigpio
import math
import time


class Cube:
    """
    Object that holds useful cube information:
    id - cube od
    history - where the cube has been
    ime_his - when the cube has been seen
    """
    def __init__(self, id):
        self.id = id
        self.history = []
        self.time_his = []
        self.z_his = []
        self.last_detected_seq = -1

    def get_position(self):
        return(self.history[0])

    def update_pos(self, x, y, z, z_rot):
        if len(self.history) == 5:
            self.history.pop(0)
            self.z_his.pop(0)
            self.time_his.pop(0)
            
        self.history.append([x, y, z])
        self.z_his.append(z_rot)
        self.time_his.append(time.time())
        # print(f"Cube ID: {self.id}, Time: {time.time()}, New Pos: {x}, {y}, {z}")

######################################
#         HELPER FUNCTIONS           #
######################################

def z_rot_matrix(theta):
    """
    Calculates a rotation matrix in the z-axis given a theta angle.
    
    Parameters:
        theta: angle rotate about z axis
    Returns:
        Transformation matrix of rotation
    """
    R = np.array([[np.cos(theta), -np.sin(theta), 0],
                  [np.sin(theta), np.cos(theta), 0],
                  [0,0,1]])
    return R

#makes a transformation matrix
def rp_to_trans(R, p):
    """
    Calculates the transformation matrix of the rotation and translation given
    a rotation matrix and translation vector.
    
    Parameters:
        R : a rotation matrix
        p: a translation vector 3x1
    
    Returns:
        The trasnformation matrix of the rotation and translation R and p
    """

    return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]

def cam_cen_trans(z_rot, p):
    """
    Calculates the transformation from the camera to the center of the aruco tag.
    
    Parameters:
        zrot: the rotation of the aruco tag about the z axis
        p: the translation of the aruco tag from the camera
    
    Returns: 
        The transformation matrix from the camera to the centre of 
        an aruco tag

    """
    I = np.eye(3)

    # Tag side length
    L = 0 #mm
    pcor_cen = np.array([L/2, L/2, 0])
    # transformation from corner of tag to centre
    Tcor_cen = rp_to_trans(I, pcor_cen)

    #theta should be given from aruco
    Rcam_cor = z_rot_matrix(z_rot)

    Tcam_cor = rp_to_trans(Rcam_cor, p)
    # Transformation from camera to corner of arucotag
    Tcam_cen = np.dot(Tcam_cor, Tcor_cen)

    return Tcam_cen

# transformation from s frame to centre of cube
def s_cen_trans(Tcam_cen,Ts_cam):
    """
    Calculates the transformation between the center of a cube 
    from the s-frame (base of the robot arm)
    
    Parameters:
        Tcam_cen: transformation matrix from camera to centre of block
        Ts_cam: transformation matrix from the s frame at the base of 
                the robot to the camera
    Returns:
        Ts_cen: The framformation matrix from the s frame on the robot to the
                centre of the tag
    """
    Ts_cen = np.dot(Ts_cam, Tcam_cen)
    return Ts_cen

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    
    Output:
        roll_x: rotation around x in radians (counterclockwise)
        pitch_y: rotation around y in radians (counterclockwise)
        yaw_z: rotation around z in radians (counterclockwise)
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

def invk(x,y,z):
    """
    Calculates the inverse kinematics for a given (x, y, z) coordinate position.
    Coordinates are relative to the base of robot arm.
    
    Parameters:
        x: desired x-position in millimeters
        y: desired y-position in millimeters
        z: desired z-position in millimeters
    Returns:
        msg: returns the ROS std_msg JointState with the desired 
             joint angle parameters  
    """
   
    #robot arm lengths
    L1 = 95 
    L2 = 117.5 
    L3 = 95 
    L4 = 70

    phi = -np.array((0,1,2,3,4))*np.pi/8 #angle the gripper makes from horizontal
    solutions = list()

    theta1 = np.arctan2(y,x)+np.pi/2
    

    for angle in phi: #iterates over the gripper angles
        pwx = np.sqrt(x**2+y**2) - L4*np.cos(angle) #distance of of joint 4 on x-y plane from zero position
        pwy = z- L4*np.sin(angle) - L1 + 18*np.cos(angle) #height position of joint 4
 

        if (np.sqrt(pwx**2 + pwy**2) <= L2+L3): #condition for solution to exist

            c2 = (pwx**2 + pwy**2 - L2**2 - L3**2)/(2*L2*L3) #cosine rule

            try:
                v3_1 = -np.arccos(c2)  #elbow-up position
                v3_2 = +np.arccos(c2)  #elbow-down position
            except:
                print(f"error in invk {c2}")
                
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
            theta4_1 = v4_1
            theta4_2 = v4_2

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
        #print(f"phi is = {np.abs(np.pi/2-solution[1]-solution[2]-solution[3])}")

        if (solution[0] > 2.6): # if the theta1 is too big we can rotate pi rad and flip the angles
            solution[0] = solution[0] - np.pi
            solution[1] = -solution[1]
            solution[2] = -solution[2]
            solution[3] = -solution[3]

        if (solution[0] < -2.6): #same as above for large negative theta1
            solution[0] = solution[0] + np.pi
            solution[1] = -solution[1]
            solution[2] = -solution[2]
            solution[3] = -solution[3]

        if (np.abs(np.pi/2-solution[1]-solution[2]-solution[3]) \
        < (np.abs(np.pi/2-best_solution[1]-best_solution[2]-best_solution[3]))):
            best_solution = solution
            print(f'Inverse Kinematics Joint Angles: {solution}')
            #print('Phi', np.abs(np.pi/2-solution[1]-solution[2]-solution[3]))

    # Create message of type JointState
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4'],
        position=[best_solution[0], best_solution[1], best_solution[2], best_solution[3]],
        velocity=[2, 2, 2, 2],
        #effort=[20, 20, 20, 20]
    )
    
    return msg

def init_global_vars():
    """
    Initialises all global variables used for robot arm.
    """

    # setup gripper / gpio
    global rpi
    rpi = pigpio.pi()
    rpi.set_mode(18, pigpio.OUTPUT)

    # dictionary of cubes detected in the system
    global cubes        
    cubes = {}

    # variables which manage state machine of code
    global states       # dictionary of states
    global state        # current state of arm

    states = {
        "PREDICTION" : 0,   # for looking for blocks
        "PICKUP" : 1,       # to move to pickup and grip blocks
        "COLOUR_CHECK" : 2, # to move to a positoin to read color then read color
        "DROP_OFF" : 3      # move to position according to color of block and drop
    }
    state = states["PREDICTION"]

    # colour related
    global current_colour           # current colour in colour check state
    global drop_off_points          # dictionary of drop off points    
    
    drop_off_points = { #positions of the relevant drop off areas according to color
        "red"     : [150, 40, 150],    
        "green"   : [50, 150, 75],   
        "blue"    : [-50, 150, 75],    
        "yellow"  : [-150, 40, 150],
    }
        
    current_colour = {
        "r": 0,
        "g": 0,
        "b": 0
    }

    #transformation matrix from camera to robot - found manually 
    global T_base_cam
    T_base_cam = np.array([
                    [1,  0,  0, 0],
                    [0, -1,  0, -175],
                    [0,  0, -1, 455],
                    [0,  0,  0,    1],
                ])

def init_sub_pub():
    """
    Initlialises all ROS publishers and subscribers used for the project.
    """
    # Initialise node with any node name
    rospy.init_node('metr4202_w7_prac')
    global pub
    global desired_pose_pub
    global gripper_pub

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

    # subscriber for desired pose=
    sub = rospy.Subscriber(
        'desired_pose',     # Topic name
        Pose,               # Message type
        invk_cb             # Callback function (required)
    )

    # subscriber for gripper
    # NOTE: gripper values 
    #   750 is the closed position
    #   1150 is the grip box position
    #   2000 is the open position
    grip_sub = rospy.Subscriber(
        'gripper_value',    # topic name
        Float32,            # msg type
        gripper_cb          # callback function
    )

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
    
    # subscriber for colour
    colour_sub = rospy.Subscriber(
        'test_color',
        ColorRGBA,
        colour_check_cb
    )

def check_arm_in_place():
    """
    Waits until the desired_joint_angles and current_joint_angles are 
    in within a set error threshold (err_threshold).
    """
    global desired_joint_angles
    global current_joint_angles
    global state
    global states
    global cubes

    initial_time = time.time()
    arm_in_place = False
    err_threshold = 0.1
    
    while(not arm_in_place):
        current_time = time.time()
        
        if current_time - initial_time > 3:
            cubes.clear()
            state = states["PREDICTION"]
            print("arm_in_place reset to prediction")
            break
            
        # add delay incase new information needs to be proccessed
        rospy.sleep(0.5)
        
        #print(f"desired: {desired_joint_angles} current: {current_joint_angles}")
        
        # difference between joint angles
        diff_j1 = np.abs(desired_joint_angles[0] - current_joint_angles[0])
        diff_j2 = np.abs(desired_joint_angles[1] - current_joint_angles[1])
        diff_j3 = np.abs(desired_joint_angles[2] - current_joint_angles[2])
        diff_j4 = np.abs(desired_joint_angles[3] - current_joint_angles[3])
        
        
        #print("Arm not in place")
        #print(f"Angle Differences: \n j1:{diff_j1} j2:{diff_j2} j3:{diff_j3} j4:{diff_j4}")
        if (diff_j1 < err_threshold and diff_j2 < err_threshold and 
                diff_j3 < err_threshold and diff_j4 < err_threshold):
            arm_in_place = True
    print("Arm set in place")  
    
def move_to_pos(x, y, z):
    """
    Publishes a desired (x, y, z) position to the ROS topic 'desired_pose'.
    
    Parameters:
        x: x-position relative to arm
        y: y-position relative to arm
        z: z-position relative to arm
    """
    global desired_pose_pub

    msg = Pose()
    msg.position.x = x
    msg.position.y = y
    msg.position.z = z
    desired_pose_pub.publish(msg)

    check_arm_in_place()        

def set_joint_angles(theta_1, theta_2, theta_3, theta_4):
    """
    Moves robot arm to specific set of joint angles.
    
    Parameters:
        theta_1: joint 1 angle
        theta_2: joint 2 angle
        theta_3: joint 3 angle
        theta_4: joint 4 angle 
    """
    # msg_2: colour check position with rig
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4'],
        position=[theta_1, theta_2, theta_3, theta_4]
    )
    
    desired_joint_angles = msg.position
    pub.publish(msg)
    check_arm_in_place()
    
def move_to_colour_check_pos():
    """
    Moves robot arm to a position used to check colour of cube.
    """
    global pub
    global desired_joint_angles

    set_joint_angles(0, 0.7, 0.69, 1.39)
    
def pickup_cube(cube: Cube):
    """
    Moves to the cube position and closes gripper on arrival
    """
    global desired_pose_pub
    global gripper_pub
    global desired_joint_angles
    global current_joint_angles

    cube_last_pos = cube.history[-1]
    
    # calibration values
    x_cal = np.sign(cube_last_pos[0])*5
    y_cal = np.sign(cube_last_pos[1])*10 + 10/(cube_last_pos[1]/50)
    
    # send desired intermediate pose above cube for traj. control
    msg = Pose()
    msg.position.x = cube_last_pos[0] + x_cal
    msg.position.y = cube_last_pos[1] + y_cal
    msg.position.z = cube_last_pos[2] + 75

    desired_pose_pub.publish(msg)
    check_arm_in_place()
    rospy.sleep(0.5)

    # send desired position to desired pose topic
    msg = Pose()
    msg.position.x = cube_last_pos[0] + x_cal
    msg.position.y = cube_last_pos[1] + y_cal
    msg.position.z = cube_last_pos[2] + 30

    desired_pose_pub.publish(msg)
    check_arm_in_place()
    rospy.sleep(1)
    # grab box (1500 value)
    gripper_pub.publish(Float32(1150))
    rospy.sleep(1)
    print("closed gripper")
    
    # send desired intermediate pose above cube for traj. control
    msg = Pose()
    msg.position.x = cube_last_pos[0] + x_cal
    msg.position.y = cube_last_pos[1] + y_cal
    msg.position.z = cube_last_pos[2] + 150
    
    desired_pose_pub.publish(msg)
    check_arm_in_place()
    rospy.sleep(1)

def colour_check():
    """
    Checks the current colour being detected against a set of colours to detect.
    
    Returns:
        An index corresponding to a specific coulour.
    """
    global current_colour

    # check colour
    red = [150, 50, 50]
    green = [50, 150, 50]
    blue = [50, 50, 50]
    yellow = [150, 100, 50]
    test_colours = [red, green, blue, yellow]
    
    current_colour_vals = [current_colour["r"], current_colour["g"], current_colour["b"]]
    colour_index = -1
    
    min_colour_diff = 999 # colour diff max
    
    for i, colour_check in enumerate(test_colours):
        colour_diff = np.sqrt((colour_check[0]-current_colour_vals[0])**2 + (colour_check[1]-current_colour_vals[1])**2 + (colour_check[2]-current_colour_vals[2])**2)
        
        if colour_diff < min_colour_diff:
            colour_index = i
            min_colour_diff = colour_diff
                
    return colour_index

################################################################
#                 CALLBACK FUNCTIONS                           #
################################################################


def invk_cb(pose: Pose):
    """
    ROS subscriber callback which runs a Pose ROS std_msg through inverse
    kinematics and publishes the desired joint angles.
    
    Parameters:
        pose: ROS std_msg holding (x, y, z) positions 
    """
    global pub
    global desired_joint_angles
    global desired_pos
    global state
    global states
    global cubes
    
    try:
        desired_jstate = invk(pose.position.x, pose.position.y, pose.position.z)
        desired_pos = [pose.position.x, pose.position.y, pose.position.z]
        desired_joint_angles = desired_jstate.position
        pub.publish(desired_jstate)
    except:
        state = states["PREDICTION"]
        cubes.clear()
        print(f'Failed at desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')            
    # print(f"from invk{desired_joint_angles}")

def gripper_cb(gripValue: Float32):
    """
    ROS subscriber callback which takes a given value to set PWM to the preset 
    GPIO pin 18.
    
    Parameters:
        gripValue: Float32 value to set PWM
    """
    global rpi
    # check limits so servo doesnt break
    if (gripValue.data < 1000 or gripValue.data > 2000):
        print(f"Gripper value ({gripValue.data}) is invalid. Limits are 1000-2000.")
    else:
        rpi.set_servo_pulsewidth(18,gripValue.data) 
        # print(f"Gripper state value changed to: + {gripValue.data}")

def acuro_cb(data: FiducialTransformArray): 
    """
    ROS subscriber callback which takes a FiducialTransformArray holding the transformations
    and information about detected aruco tags.

    Parameters:
        data: FiducialTrnasformArray from ROS std_msgs holding detected aruco tag information
    """
    global cubes
    global T_base_cam

    # if there is a cube detected, transforms array > 0
    if len(data.transforms) > 0:
        for data_transform in data.transforms:
            # get main cube information
            
            tag_id = data_transform.fiducial_id
            tag_pos = data_transform.transform.translation
            tag_rot = data_transform.transform.rotation
            
            x, y, z_rot = euler_from_quaternion(tag_rot.x, tag_rot.y, tag_rot.z, tag_rot.w)
            
            # convert pos data to array
            p = np.array([tag_pos.x*1000, tag_pos.y*1000, tag_pos.z*1000])
            # print(f"Cube corner position: {p}")
            
            # transformation from cam to cube center
            T_cam_cubeCenter = cam_cen_trans(z_rot, p)
            pos_cam_cubeCenter = [T_cam_cubeCenter[0][3], T_cam_cubeCenter[1][3], T_cam_cubeCenter[2][3]]
            #print(f"Displacement from cam to centre p{pos_cam_cubeCenter}")
            
            # check if cube in cubelist already
            if tag_id not in cubes.keys():
                newCube = Cube(tag_id)
                cubes[tag_id] = newCube

            # convert cube position data 
            #T_s_cube_center = np.dot(T_base_cam, np.array([pos_cam_cubeCenter[0], pos_cam_cubeCenter[1], pos_cam_cubeCenter[2]+5, 1]))
            
            T_s_cube_center = np.dot(T_base_cam, T_cam_cubeCenter)
            np.set_printoptions(suppress=True)   
            # print(f"T_s_cube_center: \n{T_s_cube_center}")
            p = T_s_cube_center[0:3, 3]
            # print(f"Cube center position: {p}")
            
            # update cube position relative to arm base
            cubes.get(tag_id).update_pos(T_s_cube_center[0][3], T_s_cube_center[1][3], T_s_cube_center[2][3], z_rot)
                       
def joint_state_cb(data:JointState):
    """
    ROS subscriber callback which takes a JoinState std_msg which holds the current
    information on dynamixel motors. Updates global variable holding the information
    of current joint angles.

    Paramaters:
        data: the data of the current joint states
    """
    global current_joint_angles
    if len(data.position) == 4:
        # data is returned as [j4, j3, j2, j1] but desired posed saved as [j1, j2, j3, j4]
        current_joint_angles = list(data.position)
        current_joint_angles.reverse()

def colour_check_cb(data:ColorRGBA):
    """
    ROS subscriber callback which takes a ColorRGBA std_msg holding RGBA levels.
    Updates the current colour being detected by camera.
    
    Paramaters:
        data: the data of the currnet color read
    """
    global current_colour
    global states
    global state

    if state == states["COLOUR_CHECK"]:
        current_colour["r"]=data.r
        current_colour["g"]=data.g
        current_colour["b"]=data.b
        # print(f"Updated colour data to: {current_colour}")

########################################
#              MAIN                    #
########################################

def main():
    # initialise nodes and gripper
    init_sub_pub()
    init_global_vars()

    # global variables used in main
    global current_joint_angles     # array of current angles in radians
    global desired_joint_angles     # array of desired angles in radians
    global cubes                    # dictionary of cubes detected in the system
    global state                    # current state of state machine
    global states                   # ditcionary of states
    global current_colour           # current colour in colour check state
    global drop_off_points          # dictionary of drop off points
    global desired_pos              # desried x,y,z position of arm
    # add initial delay so everything can load
    rospy.sleep(2)
    
    testSpeed = rospy.Rate(4)

    colour_name = ["red", "green", "blue", "yellow"]
    state_names = ["prediction", "pickup", "colour check", "drop off"]

    reset_pos = [1, -100, 200]
    init_pos = [150, 150, 200]
    colour_check_pos = [-1, -180, 240]

    while not rospy.is_shutdown():
        print(f"---------- Current State: {state} {state_names[state]}----------")
        if state == states.get("PREDICTION"):
            # implementation 1:
            # detect when cubes have stopped and detect from there

            stopped = False

            # move to a position to view cubes
            move_to_pos(reset_pos[0], reset_pos[1], reset_pos[2])
            gripper_pub.publish(Float32(2000))

            # check if there has been a cube added to the system
            if len(cubes) > 0:
                id, cube = list(cubes.items())[0]
                if len(cube.history) == 5:
                    current = cube.history[0]
                    oldest = cube.history[4]

                    dist = np.sqrt((current[0]-oldest[0])**2 + (current[1]-oldest[1])**2 + (current[2]-oldest[2])**2)
                    print(f"distance prediction value: {dist}")
                    if dist < 5:
                        stopped = True


            if stopped:
                # change to PICKUP state
                state = states["PICKUP"]
            

        elif state == states.get("PICKUP"):
            # CURRENT IMPLEMENTATION: 
            #   pickup closest box to robot arm
            best_distance = 10000
            
            for cube_id, cube in list(cubes.items()):

                distance = np.linalg.norm(cube.get_position())
                cube_rot = abs(np.rad2deg(cube.z_his[-1]))%90
                arm_rot = abs(np.rad2deg(np.arctan2(desired_pos[0], desired_pos[1])))%90
                rot_comp = abs(cube_rot - arm_rot)
                
                print(f"Cube {cube_id}: distance- {distance} cube_rot- {cube_rot} arm_rot- {arm_rot} rot_comp- {rot_comp}") 
                if distance < best_distance:
                    closest_cube = cube
                    best_distance = distance

            pickup_cube(closest_cube)            
            
            # move arm holding block out of the way
            move_to_pos(100,1,200)
            rospy.sleep(2)
            state = states["COLOUR_CHECK"]
        elif state == states.get("COLOUR_CHECK"):
            block_removed = False
            
            # check if cube still on the board
            for cube_id in cubes.keys():
                current_time = time.time()
                time_diff = current_time - cubes[cube_id].time_his[0]
                print(f"time last detected {cube_id}: {time_diff}") 
                if time_diff > 3:
                     block_removed = True
                     break
            
            if block_removed:
                # move to colour check position
                move_to_colour_check_pos()
                # update values for 2 seconds
                rospy.sleep(3)
                
                drop_off_colour_index = colour_check()
                print('drop off color position: ', colour_name[drop_off_colour_index])
                state = states["DROP_OFF"]
            else:
                cubes.clear()
                state = states["PREDICTION"]
                
        elif state == states.get("DROP_OFF"):
            drop_off_point = drop_off_points[colour_name[drop_off_colour_index]]
            move_to_pos(drop_off_point[0], drop_off_point[1], drop_off_point[2])
            
            rospy.sleep(1)
            
            # open gripper
            gripper_pub.publish(Float32(2000))
            
            rospy.sleep(0.5)
            cubes.clear()
            state = states["PREDICTION"]
        # You spin me right round baby, right round...
        # Just stops Python from exiting and executes callbacks
        testSpeed.sleep()
        
    rospy.spin()


if __name__ == '__main__':
    main()
