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
        self.last_detected_seq = -1

    def get_position(self):
        return(self.history[0])

    def update_pos(self, x, y, z):
        if len(self.history) == 5:
            self.history.pop(0)
            self.time_his.pop(0)
            
        self.history.append([x, y, z])
        self.time_his.append(time.time())
        # print(f"Cube ID: {self.id}, Time: {time.time()}, New Pos: {x}, {y}, {z}")

######################################
# HELPER FUNCTIONS
######################################

def zRotMatrix(theta):
    """
    params: theta- angle rotate about z axis
    returns: transformation matrix of rotation
    """
    R = np.array([[np.cos(theta), -np.sin(theta), 0],
                  [np.sin(theta), np.cos(theta), 0],
                  [0,0,1]])
    return R

#makes a transformation matrix
def RpToTrans(R, p):
    """
    params - R : a rotation matrix
    params - p: a translation vector 3x1
    returns - the trasnformation matrix of the rotation and translation R and p
    """

    return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]

def TransCamCen(z_rot, p):
    """
    param zrot - the rotation of the aruco tag about the z axis
    param p - the translation of the aruco tag from the camera
    returns - the transformation matrix from the camera to the centre of 
    an aruco tag

    """
    I = np.eye(3)

    # Tag side length
    L = 0 #mm
    pcor_cen = np.array([L/2, L/2, 0])
    # transformation from corner of tag to centre
    Tcor_cen = RpToTrans(I, pcor_cen)

    #theta should be given from aruco
    Rcam_cor = zRotMatrix(z_rot)

    Tcam_cor = RpToTrans(Rcam_cor, p)
    # Transformation from camera to corner of arucotag
    Tcam_cen = np.dot(Tcam_cor, Tcor_cen)

    return Tcam_cen

# transformation from s frame to centre of cube
def TransSCen(Tcam_cen,Ts_cam):
    """
    param Tcam_cen - transformation matrix from camera to centre of block
    param Ts_cam - transformation matrix from the s frame at the base of 
    the robot to the camera
    returns - the framformation matrix from the s frame on the robot to the
    centre of the tag
    """
    Ts_cen = np.dot(Ts_cam, Tcam_cen)
    return Ts_cen

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

def invk2(x,y,z):
    """
    finds the inverse kinematics angles for the robot to get to x,y,z 
    zero position at joint 1 with x and y pos as written on robot
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
                print(f"error in invk2 {c2}")
                
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
        velocity=[0.4, 0.4, 0.4, 0.4],
        #effort=[20, 20, 20, 20]
    )
    
    return msg

def init_global_vars():

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
    global initialised  # variable check to see if robot has initialised

    states = {
        "HOMESTATE" : 0, #for initalising the robot
        "PREDICTION" : 1, # for looking for blocks
        "PICKUP" : 2, # to move to pickup and grip blocks
        "COLOUR_CHECK" : 3, #to move to a positoin to read color then read color
        "DROP_OFF" : 4 # move to position according to color of block and drop
    }
    state = states["PREDICTION"]
    initialised = False

    # colour related variables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
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
    Initialises subscribers and publishers
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
    #750 is the closed position
    #1150 is the grip box position
    #2000 is the open position
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
    Checks if the arm has arrived at desired position
    Only returns when arm is in position
    """
    global desired_joint_angles
    global current_joint_angles
    global state
    global states
    global cubes

    initial_time = time.time()
    arm_in_place = False
    
    while(not arm_in_place):
        current_time = time.time()
        
        if current_time - initial_time > 3:
            cubes.clear()
            state = states["PREDICTION"]
            print("arm_in_place reset to prediction")
            break
            
        # add delay incase new information needs to be proccessed
        rospy.sleep(0.1)
        #print(f"desired: {desired_joint_angles} current: {current_joint_angles}")
        diff_j1 = np.abs(desired_joint_angles[0] - current_joint_angles[0])
        diff_j2 = np.abs(desired_joint_angles[1] - current_joint_angles[1])
        diff_j3 = np.abs(desired_joint_angles[2] - current_joint_angles[2])
        diff_j4 = np.abs(desired_joint_angles[3] - current_joint_angles[3])
        
        
        #print("Arm not in place")
        #print(f"Angle Differences: \n j1:{diff_j1} j2:{diff_j2} j3:{diff_j3} j4:{diff_j4}")
        if diff_j1 < 0.5 and diff_j2 < 0.5 and diff_j3 < 0.5 and diff_j4 < 0.5:
            arm_in_place = True
    print("Arm set in place")  
    
def move_to_pos(x, y, z):
    """
    Moves to a position
    """
    global desired_pose_pub

    msg = Pose()
    msg.position.x = x
    msg.position.y = y
    msg.position.z = z
    desired_pose_pub.publish(msg)

    check_arm_in_place()        

def set_joint_angles(theta_1, theta_2, theta_3, theta_4):
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
    global pub
    global desired_joint_angles

    """
    # msg_1: colour check position without rig
    msg_1 = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4'],
        position=[0, 0.7, 0.69, 1.39]
    )
    
    # msg_2: colour check position with rig
    msg_2 = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4'],
        position=[0, 0.62, 0.77, 1.5]
    )
    """
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
    
    # send desired intermediate pose above cube for traj. control
    msg = Pose()
    msg.position.x = cube_last_pos[0] + np.sign(cube_last_pos[0])*5 
    msg.position.y = cube_last_pos[1] + np.sign(cube_last_pos[1])*10 + 10/(cube_last_pos[1]/50)
    msg.position.z = cube_last_pos[2] + 75

    desired_pose_pub.publish(msg)
    check_arm_in_place()
    rospy.sleep(0.5)

    # send desired position to desired pose topic
    msg = Pose()
    msg.position.x = cube_last_pos[0] + np.sign(cube_last_pos[0]) *5
    msg.position.y = cube_last_pos[1] + np.sign(cube_last_pos[1])*10 + 10/(cube_last_pos[1]/50)
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
    msg.position.x = cube_last_pos[0] + np.sign(cube_last_pos[0])*5 
    msg.position.y = cube_last_pos[1] + np.sign(cube_last_pos[1])*10 + 10/(cube_last_pos[1]/50)
    msg.position.z = cube_last_pos[2] + 150
    
    desired_pose_pub.publish(msg)
    check_arm_in_place()
    rospy.sleep(1)

def colour_check():
    global current_colour

    # check colour
    red = [150, 50, 50]
    green = [50, 150, 50]
    blue = [50, 50, 50]
    yellow = [150, 100, 50]
    test_colours = [red, green, blue, yellow]
    
    current_colour_vals = [current_colour["r"], current_colour["g"], current_colour["b"]]
    colour_index = -1
    # colour diff max
    min_colour_diff = 999
    for i, colour_check in enumerate(test_colours):
        colour_diff = np.sqrt((colour_check[0]-current_colour_vals[0])**2 + (colour_check[1]-current_colour_vals[1])**2 + (colour_check[2]-current_colour_vals[2])**2)
        if colour_diff < min_colour_diff:
            colour_index = i
            min_colour_diff = colour_diff
                
    return colour_index

################################################################
# CALLBACK FUNCTIONS
################################################################

# This one doesn't actually do it though...
def invk_cb(pose: Pose):
    global pub
    global desired_joint_angles
    global state
    global states
    global cubes
    
    try:
        desired_jstate = invk2(pose.position.x, pose.position.y, pose.position.z)
        desired_joint_angles = desired_jstate.position
        pub.publish(desired_jstate)
    except:
        state = states["PREDICTION"]
        cubes.clear()
        print(f'Failed at desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
            
    #print(f"from invk{desired_joint_angles}")

# grippper callback checks if grip value is not outside of limits and sets it
def gripper_cb(gripValue: Float32):
    global rpi
    # check limits so servo doesnt break
    if (gripValue.data < 1000 or gripValue.data > 2000):
        print(f"Gripper value ({gripValue.data}) is invalid. Limits are 1000-2000.")
    else:
        rpi.set_servo_pulsewidth(18,gripValue.data) 
        # print(f"Gripper state value changed to: + {gripValue.data}")

def acuro_cb(data: FiducialTransformArray): 
    global cubes
    global T_base_cam
    # global initialised
    # ARM_ID = 19

    # if there is a cube detected, transforms array > 0
    if len(data.transforms) > 0:
        for data_transform in data.transforms:
            # get main cube information
            
            tag_id = data_transform.fiducial_id
            tag_pos = data_transform.transform.translation
            tag_rot = data_transform.transform.rotation
            
            x, y, z = euler_from_quaternion(tag_rot.x, tag_rot.y, tag_rot.z, tag_rot.w)
            
            # convert pos data to array
            p = np.array([tag_pos.x*1000, tag_pos.y*1000, tag_pos.z*1000])
            # print(f"this is p: {p}")
            
            # transformation from cam to cube center
            T_cam_cubeCenter = TransCamCen(z, p)
            pos_cam_cubeCenter = [T_cam_cubeCenter[0][3], T_cam_cubeCenter[1][3], T_cam_cubeCenter[2][3]]
            #print(f"Displacement from cam to centre p{pos_cam_cubeCenter}")
            
            # check if cube in cubelist already
            if tag_id not in cubes.keys():
                newCube = Cube(tag_id)
                cubes[tag_id] = newCube

            # convert cube position data 
            #T_s_cube_center = np.dot(T_base_cam, np.array([pos_cam_cubeCenter[0], pos_cam_cubeCenter[1], pos_cam_cubeCenter[2]+5, 1]))
            
            T_s_cube_center = np.dot(T_base_cam, T_cam_cubeCenter)
            p = T_s_cube_center[0:3, 3]
            
            # update cube position relative to arm base
            cubes.get(tag_id).update_pos(T_s_cube_center[0][3], T_s_cube_center[1][3], T_s_cube_center[2][3])

            """ #
            if tag_id != ARM_ID and initialised:
                # check if cube in cubelist already
                if tag_id not in cubes.keys():
                    newCube = Cube(tag_id)
                    cubes[tag_id] = newCube

                # convert cube position data 
                #T_s_cube_center = np.dot(T_base_cam, np.array([pos_cam_cubeCenter[0], pos_cam_cubeCenter[1], pos_cam_cubeCenter[2]+5, 1]))
                
                T_s_cube_center = np.dot(T_base_cam, T_cam_cubeCenter)
                
                p = T_s_cube_center[0:3, 3]
                
                #V = np.dot(T_base_cam, np.array([tagPos.x*1000, tagPos.y*1000, tagPos.z*1000, 1]))
                        
                #print("V", V)
                # update cube position relative to arm base
                cubes.get(tag_id).update_pos(T_s_cube_center[0][3], T_s_cube_center[1][3], T_s_cube_center[2][3])

            elif tag_id == ARM_ID and not initialised:
                T_base_aruco = np.array([
                    [ 1, 0, 0, -45],
                    [ 0, 1, 0, -42],
                    [ 0, 0, 1,  0],
                    [ 0, 0, 0,  1],
                ])
                
              
                print("T base aruco is initialized")

                T_cam_aruco = np.array([
                    [1,  0,  0, p[0]],
                    [0, -1,  0, p[1]],
                    [0,  0, -1, p[2]],
                    [0,  0,  0,    1],
                ])
                
                T_aruco_cam = np.linalg.inv(T_cam_aruco)
                
                T_base_cam = T_base_aruco @ T_aruco_cam
                T_base_cam[0,3] = T_base_cam[0,3] + 30
                T_base_cam[1,3] = T_base_cam[1,3]
                initialised = True
                """
                        
    pass

def joint_state_cb(data:JointState):
    """
    params data - the data of the currnet joint states
    updates the global current-joint_angles varible
    """
    global current_joint_angles
    if len(data.position) == 4:
        # data is returned as [j4, j3, j2, j1] but desired posed saved as [j1, j2, j3, j4]
        current_joint_angles = list(data.position)
        current_joint_angles.reverse()

def colour_check_cb(data:ColorRGBA):
    """
    params data - the data of the currnet color read
    updates the global current_colour varible
    """
    global current_colour
    global states
    global state

    if state == states["COLOUR_CHECK"]:
        current_colour["r"]=data.r
        current_colour["g"]=data.g
        current_colour["b"]=data.b
        # print(f"Updated colour data to: {current_colour}")



########
# MAIN
########
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
    global initialised              # variable check to see if robot has initialised
    global current_colour           # current colour in colour check state
    global drop_off_points          # dictionary of drop off points
    
    # add initial delay so everything can load
    rospy.sleep(2)
    
    testSpeed = rospy.Rate(4)

    colour_name = ["red", "green", "blue", "yellow"]
    state_names = ["homestate", "prediction", "pickup", "colour check", "drop off"]

    reset_pos = [1, -100, 200]
    init_pos = [150, 150, 200]
    colour_check_pos = [-1, -180, 240]
    initialised = True

    while not rospy.is_shutdown():
        print(f"---------- Current State: {state} {state_names[state]}----------")
        if state == states.get("HOMESTATE"):
            move_to_pos(init_pos[0], init_pos[1], init_pos[2])
            # open gripper
            gripper_pub.publish(Float32(2000))
            print('out')
            
            # clear detected cubes
            cubes.clear()
        elif state == states.get("PREDICTION"):
            # implementation 1:
            # detect when cubes have stopped and detect from there

            stopped = False

            if initialised:
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
            # CURRENT IMPLEMENTATION: pickup the first box
            best_distance = 10000
            
            for cube_id, cube in list(cubes.items()):

                distance = np.linalg.norm(cube.get_position())
                print(f"Cube {cube_id} distance: {distance}") 
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
            #if drop_off_colour_index == 1:
            #set_joint_angles(2.82, 
            #print("goes into move to pos green")
            #else:
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
