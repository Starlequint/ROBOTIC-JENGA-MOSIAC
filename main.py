from math import atan2, sqrt, cos, sin, pi, isclose
from JENGA_detection import detect_planks
from TriangleTessellation import patternRecognition
from getImage import get_single_frame
from robohub_codes import (initBase, example_forward_kinematics, example_inverse_kinematics, 
                           GripperCommandExample, example_cartesian_action_movement)
from move_angular_and_cartesian import mv
import cv2
from numpy import zeros_like
from typing import Final
import argparse
import utilities
import sys
import os
import matplotlib.pyplot as plt

class Position:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z
    def display(self):
        return f"({round(self.x,3)},{round(self.y,3)},{round(self.z,3)})mm"
    def __eq__(self, other):
        if isinstance(other, Position):
            return (isclose(self.x, other.x, rel_tol=1e-9) and 
                    isclose(self.y, other.y, rel_tol=1e-9) and
                    isclose(self.z, other.z, rel_tol=1e-9))
        return False
    def eq2d(self, other):
        if isinstance(other, Position):
            return (isclose(self.x, other.x, rel_tol=1e-9) and 
                    isclose(self.y, other.y, rel_tol=1e-9))
        return False
    def distance(self, other):
        if isinstance(other, Position):
            return sqrt((self.x-other.x)**2+(self.y-other.y)**2+(self.z-other.z)**2)
        else:
            print("Position::distance() misused")
            sys.exit(1)
    def distance2d(self, other):
        if isinstance(other, Position):
            return sqrt((self.x-other.x)**2+(self.y-other.y)**2)
        else:
            print("Position::distance2d() misused")
            sys.exit(1)
    def norm(self):
        return sqrt(self.x**2+self.y**2+self.z**2)

class Orientation:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z
    def display(self):
        return f"({round(self.x,3)},{round(self.y,3)},{round(self.z,3)})rad"
    def __eq__(self, other):
        if isinstance(other, Orientation):
            return self.x == other.x and self.y == other.y and self.z == other.z
        return False
    def distance(self, other):
        if isinstance(other, Orientation):
            return sqrt((self.x-other.x)**2+(self.y-other.y)**2+(self.z-other.z)**2)
        else:
            print("Orientation::distance() misused")
            sys.exit(1)

class Brick:
    LENGTH   : Final[float] = 7.4 # cm
    WIDTH    : Final[float] = 2.4 # cm
    THICKNESS: Final[float] = 1.4 # cm
    def __init__(self, center=0, latitudinalAngle=0, 
                 longitudinalAngle=0, plannarAngle=0):
        """orientation: y is along the brick, z on the big side, x is on the small side"""
        self.center = center
        self.start = Position(center.x-self.LENGTH*cos(plannarAngle), 
                                center.y-self.LENGTH/2*sin(plannarAngle), center.z)
        self.end = Position(center.x+self.LENGTH*cos(plannarAngle), 
                                center.y+self.LENGTH/2*sin(plannarAngle), center.z)
        self.orientation = Orientation(latitudinalAngle, longitudinalAngle, plannarAngle)
    def verticalBrick(self):
        self.length = 0
        print("Unexpected vertical brick")
    def __eq__(self, other):
        if isinstance(other, Brick):
            return (isclose(self.center.x, other.center.x, rel_tol=1e-9) and 
                    isclose(self.center.y, other.center.y, rel_tol=1e-9) and
                    self.orientation.z == other.orientation.z)
        else:
            print("Brick::==() misused")
            sys.exit(1)
            
class Move:
    def __init__(self, positions, orientations):
        self.positions = positions
        self.orientations = orientations
        if len(positions) < 2: 
            print("Error: this move lack of positions")
            sys.exit(1)
        elif len(positions) != len(orientations):
            print("Error: A move must have the same # of orientation and positions")
            sys.exit(1)
        self.start, self.end = positions[0], positions[-1]
        self.O0, self.OEnd = orientations[0], orientations[-1]
    def display(self):
        return ("from "+self.start.display()+' '+self.O0.display()+" to "+
                self.end.display()+' '+self.OEnd.display()+
                f" in {len(self.positions)} steps")

#Constants
IMG_WIDTH, IMG_HEIGHT  = 1280, 720 # pixels
HOME = Position(58, 0, 42) # cm
HOME_o = Orientation(90, 0, 0) # °
CAMERA = Position(30, -15, 40) # cm
CAMERA_o = Orientation(180, 0, 0) # °
IMG_OFFSET = Position(57.7, 15.23, 0) # mm
SCALE = -0.0439
GROUND = Position(None, None, 1) # cm
CATCH_o = Orientation(180, 0, None) # °
threshold = 1 #TODO: define the threshold for a well placed brick
GRIPPER_WIDTH = 10 # cm

def coordConv(x, y, t):
    return y*SCALE+IMG_OFFSET.x, x*SCALE+IMG_OFFSET.y, pi-t

def getImage(args):
    move(args, CAMERA, CAMERA_o, True)
    rtsp_url = "rtsp://admin:admin@192.168.1.10/color"
    frame_array = get_single_frame(rtsp_url)
    frame_depth_normalized = zeros_like(frame_array)
    cv2.normalize(frame_array, frame_depth_normalized, 0, 255, cv2.NORM_MINMAX)
    colourImage = cv2.applyColorMap(frame_depth_normalized,cv2.COLORMAP_JET)
    return frame_array, colourImage
    #code from robothub

def save(image, imagePath):
    #code written the March 15
    cv2.imwrite(imagePath,image)

def brickDetection(args):
    # task 1
    rawImage, colourImage = getImage(args)
    rawImagePath, colourImagePath = 'rawImage.png', 'ColourImage.png'
    save(rawImage, rawImagePath)
    save(colourImage, colourImagePath)

    results = detect_planks(
        raw_image_path   = rawImagePath,
        color_image_path = colourImagePath,
        save_path        = 'detected_planks.png',
    )
    #results : list of (cx, cy, angle) tuples
    bricks = [0]*len(results)
    for i in range(len(results)):
        # convertion to cm
        x, y, t = coordConv(results[i][0], results[i][1], results[i][2])
        bricks[i] = Brick(center=Position(x, y, GROUND.z+Brick.THICKNESS/2), 
                          plannarAngle=t)
    return bricks, results

def recognizePattern(rawData, bricks0):
    triangleGroupedData = patternRecognition(rawData) #none redondant list of triangles
    bricks = []
    for i in range(len(triangleGroupedData)):
        for j in range(len(triangleGroupedData[i])):
            # convertion to cm
            x, y, t= coordConv(triangleGroupedData[i][j][0], triangleGroupedData[i][j][1], triangleGroupedData[i][j][2])
            bricks.append(Brick(center=Position(x, y, GROUND.z+Brick.THICKNESS/2), 
                plannarAngle=t))
    plt.scatter([bricks[0].center.y, bricks[1].center.y, bricks[2].center.y], [bricks[0].center.x, bricks[1].center.x, bricks[2].center.x], color='r')
    plt.show()
    moves = [None]*min(len(bricks0), len(bricks))
    iUsed = [len(bricks0)]*min(len(bricks0), len(bricks))
    for j in range(min(len(bricks0), len(bricks))):
        iClosest = 0
        for i in range(1, len(bricks0)):
            if (not(bricks0[i] in [bricks[0], bricks[1], bricks[2]])):
                if (i not in iUsed and bricks[j].center.distance2d(bricks0[i].center) < bricks[j].center.distance2d(bricks0[iClosest].center)):
                    iClosest = i
            else: 
                print("model brick: ", bricks0[i])
        iUsed[j] = iClosest
        c1, c2 = bricks0[iClosest].center, bricks[j].center
        moves[j] = Move([c1, Position(c1.x, c1.y, c1.z+2*Brick.THICKNESS),
                         Position(c2.x, c2.y, c2.z+2*Brick.THICKNESS),
                         Position(c2.x, c2.y, c2.z+0.5*Brick.THICKNESS)],
                        [Orientation(CATCH_o.x, CATCH_o.y, bricks0[iClosest].orientation.z),
                         Orientation(CATCH_o.x, CATCH_o.y, bricks0[iClosest].orientation.z),
                         Orientation(CATCH_o.x, CATCH_o.y, bricks[j].orientation.z),
                         Orientation(CATCH_o.x, CATCH_o.y, bricks[j].orientation.z)])
    return moves, bricks

def initGripper(router):
    return GripperCommandExample(router)

def catch(gripper):
    """
    position = 0.0 open, 1.0 closed
    """
    gripper.close(1-Brick.WIDTH/GRIPPER_WIDTH)

def release(gripper):
    gripper.open()

def inBoundaries(position):
    """ Boundaries: Table, Joint limits, computer """
    #TODO: complete with the computer limit, and take account of the robot robot orientation
    if (position.z < 0 or position.norm() < 20 or 902.0 < position.norm()):
        return False
    return True

x0, y0, z0, ox0, oy0, oz0 = 0, 0, 0, 0, 0, 0
def move(args, position, orientation=None, firstCall=False):
    global x0, y0, z0, ox0, oy0, oz0
    if not(inBoundaries(position)):
        print("Attempt to exceed the robot limits:", position)
        sys.exit(1)
    #convert position in meter and orientation in degree (already done for orientations)
    print("Before conversion:", position.x, position.y, position.z)
    x, y, z = position.x/100, position.y/100, position.z/100
    print("After conversion:", x, y, z)

    if firstCall: mv(args, x-HOME.x/100, y-HOME.y/100, z-HOME.z/100, 
                     orientation.x-HOME_o.x, 0, orientation.z-HOME_o.z, firstCall)
    else:         mv(args, x-x0        , y-y0        , z-z0        , 
                     orientation.x-ox0     , 0, orientation.z-oz0     , firstCall)
    x0, y0, z0, ox0, oy0, oz0 = x, y, z, orientation.x, 0, orientation.z

def moveBrick(gripper, args, move_):
    move(args, move_.positions[0], move_.orientations[0], False)
    #catch(gripper)
    for j in range(1, len(move_.positions)-1):
        move(args, move_.positions[j], move_.orientations[j], False)
    #release(gripper)
    move(args, Position(move_.end.x, move_[-1].end.y, 
                                     move_.end.z+2*Brick.THICKNESS, False),
        Orientation(move_[-1].OEnd))

def brickPlaced(position, bricks):
    # task 3
    for brick in bricks:
        if brick.center.distance2d(position) < threshold:
            return True
    return False

def findMisplaced(position, bricks):
    # task 3
    iMin, minDistance = 0, bricks[0].distance2d(position)
    for i in range(1, len(bricks)):
        distance = bricks[i].distance2d(position)
        if distance < minDistance:
            iMin = i
            minDistance = distance
    return bricks[iMin]

def pushBrick(args, brick, expectedMove):
    # task 5
    #approach the brick and push it towards the expected location

    APPROACH_Z = 2*Brick.THICKNESS
    PUSH_Z = Brick.THICKNESS

    target = expectedMove.end
    yaw = brick.orientation.z
    push_orientation = Orientation(CATCH_o.x, CATCH_o.y, yaw)
    end_orientation = Orientation(CATCH_o.x, CATCH_o.y, expectedMove.OEnd)

    above_start = Position(brick.center.x, brick.center.y, brick.center.z + APPROACH_Z)
    contact_start = Position(brick.center.x, brick.center.y, brick.center.z + PUSH_Z)
    contact_end = Position(target.x, target.y, brick.center.z + PUSH_Z)
    above_end = Position(target.x, target.y, brick.center.z + APPROACH_Z)

    # Keep gripper open for a push correction
    # send_gripper(0.0)

    move(args, above_start,   push_orientation, False)
    move(args, contact_start, push_orientation, False)
    move(args, contact_end,   end_orientation , False)
    move(args, above_end,     end_orientation , False)

def forwardKinematics(base, angles):
    T = example_forward_kinematics(base, angles)
    # return Position(T[0], T[1], T[2]), Orientation(T[3], T[4], T[5])
    return Position(100*T[0], 100*T[1], 100*T[2]), Orientation(T[3], T[4], T[5])

def InverseKinematics(base, position, orientation):
    # return example_inverse_kinematics(base, position.x, position.y, position.z, 
    #                                   orientation.x, orientation.y, orientation.z)
    return example_inverse_kinematics(base, position.x/100, position.y/100, position.z/100, 
                                      orientation.x, orientation.y, orientation.z)        

def main():
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    # Parse arguments
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        gripper = initGripper(router)
        #base, base_cyclic = None, None
        print("Task 1: brick detection")
        bricks, rawData = brickDetection(args)
        print('u', rawData[0][0], 'v', rawData[0][1]) 
        move(args, bricks[0].center, Orientation(CATCH_o.x, CATCH_o.y, bricks[0].orientation.z), firstCall=False)
        '''
        print(f"Task 2: pattern recognition & prediction: on {len(bricks)} bricks found")
        moves, movedBricks = recognizePattern(rawData, bricks)
        #if (len(moves) != len(movedBricks)): 
        #    print("Incoherent outputs from the pattern recognition")
        #    sys.exit(1)
        x, y = [] ,  []
        for move in moves:
            x.append(move.start.x)
            y.append(move.start.y)
        
        plt.scatter(y, x )
        plt.scatter([bricks[0].center.y, bricks[1].center.y, bricks[2].center.y], [bricks[0].center.x, bricks[1].center.x, bricks[2].center.x], color='r')
        plt.show()
        print(f"Task 5 : robot arm movement: {len(moves)} moves are coming.")
        for i in range(len(moves)):
            print(f"Move {i} starts : "+moves[i].display())
            if i==0: print("Task 4: New brick picking")
            moveBrick(gripper, args, moves[i])
            if i==0: print("Task 3: Feedback mapping")
            currentBricks = brickDetection()
            if not(brickPlaced(moves[i].start, currentBricks)):
                print("Brick misplaced : push attempt coming")
                misplacedBrick = findMisplaced(moves[i].start, currentBricks)
                c1 = misplacedBrick.center
                move = Move([c1, Position(c1.x, c1.y, c1.z+2*Brick.THICKNESS),
                     moves[i].positions[2],
                     moves[i].positions[3]],
                    [Orientation(CATCH_o.x, CATCH_o.y, misplacedBrick.orientation.z),
                     Orientation(CATCH_o.x, CATCH_o.y, misplacedBrick.orientation.z),
                     moves[i].orientations[2],
                     moves[i].orientations[3]])
                #moveBrick(gripper, args, move)
            #pushBrick(args, misplacedBrick, moves[i])
        '''
        print("Tasks completed !")

if __name__ == '__main__':
    main()
