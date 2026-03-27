
import sys
import os
import time
import threading

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2
from kortex_api.Exceptions.KServerException import KServerException

def initBase(router):
    return BaseClient(router), BaseCyclicClient(router)


def example_forward_kinematics(base, angles):
    # Current arm's joint angles (in home position)
    try:
        print("Getting Angles for every joint...")
        input_joint_angles = base.GetMeasuredJointAngles()
    except KServerException as ex:
        print("Unable to get joint angles")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))
        return False

    print("Joint ID : Joint Angle actual, input")
    for i in range(len(input_joint_angles.joint_angles)):
        print(input_joint_angles.joint_angles[i].joint_identifier, " : ", 
              input_joint_angles.joint_angles[i].value, ', ', angles[i])
        input_joint_angles.joint_angles[i].value = angles[i]
    print()
    
    # Computing Foward Kinematics (Angle -> cartesian convert) from arm's current joint angles
    try:
        print("Computing Foward Kinematics using joint angles...")
        pose = base.ComputeForwardKinematics(input_joint_angles)
    except KServerException as ex:
        print("Unable to compute forward kinematics")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))
        return False

    print("Pose calculated : ")
    print("Coordinate (x, y, z)  : ({}, {}, {})".format(pose.x, pose.y, pose.z))
    print("Theta (theta_x, theta_y, theta_z)  : ({}, {}, {})".format(pose.theta_x, pose.theta_y, pose.theta_z))
    print()
    return [pose.x, pose.y, pose.z, pose.theta_x, pose.theta_y, pose.theta_z]

def example_inverse_kinematics(base, x, y, z, ox, oy, oz):
    # get robot's pose (by using forward kinematics)
    try:
        input_joint_angles = base.GetMeasuredJointAngles()
        pose = base.ComputeForwardKinematics(input_joint_angles)
    except KServerException as ex:
        print("Unable to get current robot pose")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))
        return False

    # Object containing cartesian coordinates and Angle Guess
    input_IkData = Base_pb2.IKData()
    
    # Fill the IKData Object with the cartesian coordinates that need to be converted
    input_IkData.cartesian_pose.x = x #pose.x
    input_IkData.cartesian_pose.y = y #pose.y
    input_IkData.cartesian_pose.z = z #pose.z
    input_IkData.cartesian_pose.theta_x = ox #pose.theta_x
    input_IkData.cartesian_pose.theta_y = oy #pose.theta_y
    input_IkData.cartesian_pose.theta_z = oz #pose.theta_z

    # Fill the IKData Object with the guessed joint angles
    for joint_angle in input_joint_angles.joint_angles :
        jAngle = input_IkData.guess.joint_angles.add()
        # '- 1' to generate an actual "guess" for current joint angles
        jAngle.value = joint_angle.value - 1
    
    try:
        print("Computing Inverse Kinematics using joint angles and pose...")
        computed_joint_angles = base.ComputeInverseKinematics(input_IkData)
    except KServerException as ex:
        print("Unable to compute inverse kinematics")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))
        return False

    print("Joint ID : Joint Angle")
    joint_identifier = 0
    T = []
    for joint_angle in computed_joint_angles.joint_angles :
        print(joint_identifier, " : ", joint_angle.value)
        joint_identifier += 1
        T.append(joint_angle.value)

    return T


class GripperCommandExample:
    def __init__(self, router, proportional_gain = 2.0):

        self.proportional_gain = proportional_gain
        self.router = router

        # Create base client using TCP router
        self.base = BaseClient(self.router)

    def close(self, end=1.0):

        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with position increments
        print("Performing gripper test in position...")
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        position = 0.00
        finger.finger_identifier = 1
        while position < end:
            finger.value = position
            print("Going to position {:0.2f}...".format(finger.value))
            self.base.SendGripperCommand(gripper_command)
            position += 0.1
            time.sleep(1)
    def open(self):
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        # Set speed to open gripper
        print ("Opening gripper using speed command...")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = 0.1
        self.base.SendGripperCommand(gripper_command)
        gripper_request = Base_pb2.GripperRequest()

        # Wait for reported position to be opened
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        while True:
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
            if len (gripper_measure.finger):
                print("Current position is : {0}".format(gripper_measure.finger[0].value))
                if gripper_measure.finger[0].value < 0.01:
                    break
            else: # Else, no finger present in answer, end loop
                break
    def speedMove(self, speed=-0.1):
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        # Set speed to close gripper
        print ("Closing gripper using speed command...")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = speed
        self.base.SendGripperCommand(gripper_command)
        gripper_request = Base_pb2.GripperRequest()

        # Wait for reported speed to be 0
        gripper_request.mode = Base_pb2.GRIPPER_SPEED
        while True:
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
            if len (gripper_measure.finger):
                print("Current speed is : {0}".format(gripper_measure.finger[0].value))
                if gripper_measure.finger[0].value == 0.0:
                    break
            else: # Else, no finger present in answer, end loop
                break

# def main():
#     # Import the utilities helper module
#     import argparse
#     sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
#     import utilities

#     # Parse arguments
#     parser = argparse.ArgumentParser()
#     args = utilities.parseConnectionArguments(parser)

#     # Create connection to the device and get the router
#     with utilities.DeviceConnection.createTcpConnection(args) as router:

#         example = GripperCommandExample(router)
#         example.ExampleSendGripperCommands()

# if __name__ == "__main__":
#     main()

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check
 
def example_move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

def example_angular_action_movement(base):
    
    print("Starting angular action movement ...")
    action = Base_pb2.Action()
    action.name = "Example angular action movement"
    action.application_data = ""

    actuator_count = base.GetActuatorCount()

    # Place arm straight up
    for joint_id in range(actuator_count.count):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = 0

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    
    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Angular movement completed")
    else:
        print("Timeout on action notification wait")
    return finished


def example_cartesian_action_movement(base, base_cyclic, x=None, y=None, z=None, ox=None, oy=None, oz=None):
    feedback = base_cyclic.RefreshFeedback()
    x0 =  feedback.base.tool_pose_x
    y0 =  feedback.base.tool_pose_y
    z0 =  feedback.base.tool_pose_z
    ox0 = feedback.base.tool_pose_theta_x
    oy0 = feedback.base.tool_pose_theta_y
    oz0 = feedback.base.tool_pose_theta_z
    print(x0, y0, z0)
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    #feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    if  x == None:  cartesian_pose.x = feedback.base.tool_pose_x          # (meters)
    else:          cartesian_pose.x =  feedback.base.tool_pose_x -x-x0         # (meters)
    if  y == None:  cartesian_pose.y = feedback.base.tool_pose_y     # (meters)
    else:          cartesian_pose.y = feedback.base.tool_pose_y - y - y0    # (meters)
    if  z == None:  cartesian_pose.z = feedback.base.tool_pose_z    # (meters)
    else:          cartesian_pose.z =  feedback.base.tool_pose_z - z  -z0  # (meters)
    if ox == None: cartesian_pose.theta_x = feedback.base.tool_pose_theta_x # (degrees
    else:          cartesian_pose.theta_x = feedback.base.tool_pose_theta_x -ox-ox0# (degrees)
    if oy == None: cartesian_pose.theta_y = feedback.base.tool_pose_theta_y # (degrees)
    else:          cartesian_pose.theta_y = feedback.base.tool_pose_theta_y -oy-oy0# (degrees)
    if oz == None: cartesian_pose.theta_z = feedback.base.tool_pose_theta_z # (degrees)
    else:          cartesian_pose.theta_z = feedback.base.tool_pose_theta_z -oz-oz0# (degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

def wait_for_action_end(base, action):
    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    base.ExecuteAction(action)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)
    return finished
