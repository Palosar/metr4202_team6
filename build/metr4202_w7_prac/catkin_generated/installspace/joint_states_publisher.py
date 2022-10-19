#!/usr/bin/env python3
"""
This script publishes a set of random joint states to the dynamixel controller.
Use this to get an idea of how to code your inverse kinematics!
"""

# Funny code
from cmath import pi
from ctypes.wintypes import PSIZE
import random
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

# command of ximearos
# echo 0 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb

def rad_to_deg(rad):
    return rad*180/pi
    
# paulo new inverse kinematics
def p_invk(x, y, z):
    #robot arm lengths
    L1 = 90 
    L2 = 116 
    L3 = 95 
    L4 = 98

    # consistent variables between modes
    px = np.sqrt(x**2+y**2)
    theta1 = np.arctan2(x, y)

    # select a gripper mode: close or far
    if (px < L2 + L3):
        print("Gripper mode close")
        # gripper is close, set gripper to vertical
        py = z + L4 - L1

        # cosine rule on theta3
        c3 = (px**2 + py**2 - L2**2 - L3**2) / (2*L2*L3)

        theta3 = np.arctan2(c3, np.sqrt(1-c3**2))
        theta2 = np.pi/2 - (np.arctan2(px, py) + np.arctan2(L2+L3*np.cos(theta3), L3*np.sin(theta3))) 
        theta4 = np.pi - theta3 - theta2
    else:
        print("Gripper mode far")
        # gripper is far, set theta2 to constant 0
        py = z - L1

        # cosine rule on theta4
        c4 = (px**2 + py**2 - (L2+L3)**2 - L4**2) / (2*(L2+L3)*(L4))

        theta4 = np.arctan2(c4, np.sqrt(1-c4**2))
        theta3 = 0
        theta2 = np.pi/2 - (np.arctan2(px, py) + np.arctan2(L2+L3+L4*np.cos(theta4), L4*np.sin(theta4)))

    # check theta1 limits
    if (theta1 > 1.7):
        theta1 = theta1 - np.pi
        theta2 = -theta2
        theta3 = -theta3
        theta4 = -theta4
    if (theta1 < -1.7):
        theta1 = theta1 + np.pi
        theta2 = -theta2
        theta3 = -theta3
        theta4 = -theta4

    print(f"Original Values: {rad_to_deg(theta1)}, {rad_to_deg(theta2)}, {rad_to_deg(theta3)}, {rad_to_deg(theta4)}")
    # check theta4 limits
    if (theta4 > 1.7):
        theta4 = 1.7
        print("Theta4 hit limit 1.7rad")
    if (theta4 < -1.7):
        theta4 = -1.7 
        print("Theta4 hit limit -1.7rad")

    # return
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
    )
    
    # put joint messages into msg
    msg.position = [
        theta1,
        theta2,
        theta3,
        -theta4
    ]

    print(f"Final Values {msg.position}")
    print(f"px:{px} py:{py}")
    return(msg)


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
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
    )
    
    print(best_solution)
    
    # put joint messages into msg
    msg.position = [
        best_solution[0],
        best_solution[1],
        best_solution[2],
        best_solution[3]
    ]
     
    return(msg)


# This one doesn't actually do it though...
def inverse_kinematics(pose: Pose) -> JointState:
    global pub
    # TODO: Have fun :)
    rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
    pub.publish(invk2(pose.position.x, pose.position.y, pose.position.z))

# grippper callback checks if grip value is not outside of limits and sets it
def gripper_cb(gripValue: Float32):
    rpi = pigpio.pi()
    rpi.set_mode(18, pigpio.OUTPUT)

    # check limits so servo doesnt break
    if (gripValue.data < 1000 or gripValue.data > 2000):
        rospy.loginfo("Gripper value (" + gripValue.data + ") is invalid. Limits are 1000-2000.")
    else:
        rpi.set_servo_pulsewidth(18,gripValue.data) 
        rospy.loginfo("Gripper state value changed to: " + gripValue.data)



def acuro_cb(data: FiducialTransformArray):
    # print(data.transforms)
    global stateMove
    global pub2
    if len(data.transforms) > 0:
        x = data.transforms[0].transform.translation.x
        y = data.transforms[0].transform.translation.y
        z = data.transforms[0].transform.translation.z

        # if stateMove:
        Tsd = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, -150],
            [0, 0, -1, 420],
            [0, 0, 0, 1]
        ])

        V = np.dot(Tsd, np.array([x*1000, y*1000, z*1000, 1]))
        print(V)
        
        msg = Pose()
        msg.position.x = V[0]
        msg.position.y = V[1]
        msg.position.z = V[2]

        # 

        pub2.publish(msg)

            # stateMove = False
    pass

def main():
    
    # Initialise node with any node name
    rospy.init_node('metr4202_w7_prac')
    global pub
    global pub2
    global stateMove

    stateMove = True

    # Create publisher
    pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState,             # Message type
        queue_size=10           # Topic size (optional)
    )
    
    # publisher for desired pose
    pub2 = rospy.Publisher(
        'desired_pose',
        Pose,
        queue_size=10
    )

    # subscriber for desired pose
    sub = rospy.Subscriber(
        'desired_pose',         # Topic name
        Pose,                   # Message type
        inverse_kinematics      # Callback function (required)
    )

    # subscriber for gripper
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




    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()