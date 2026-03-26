from math import atan2, sqrt, cos, sin, pi
from JENGA_detection import detect_planks
from TriangleTessellation import patternRecognition
from getImage import get_single_frame
from robohub_codes import (initBase, example_forward_kinematics, example_inverse_kinematics, 
                           GripperCommandExample, moveCarteresian)
import cv2
from numpy import zeros_like
from typing import Final
import argparse
import utilities
import sys
import os

class Position:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z
    def display(self):
        return f"({round(self.x,3)},{round(self.y,3)},{round(self.z,3)})mm"
    def __eq__(self, other):
        if isinstance(other, Position):
            return self.x == other.x and self.y == other.y and self.z == other.z
        return False
    def eq2d(self, other):
        if isinstance(other, Position):
            return self.x == other.x and self.y == other.y
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
    LENGTH   : Final[float] = 74 # cm
    WIDTH    : Final[float] = 24 # cm
    THICKNESS: Final[float] = 14 # cm
    def __init__(self, start=0, center=0, end=0, latitudinalAngle=0, 
                 longitudinalAngle=0, plannarAngle=0):
        """orientation: y is along the brick, z on the big side, x is on the small side"""
        if (start != 0 or end != 0):
            self.start, self.end = start, end
            if (not(start.eq2d(end))):
                thetaZ = atan2(end.y-start.y, end.x-start.x)
                if (self.thetaX != pi/2 and self.thetaX != -pi/2):
                    self.length = sqrt((end.y-start.y)**2+(end.x-start.x)**2
                                    )/cos(self.thetaX)
                else:
                    self.verticalBrick()
            else:
                thetaZ = 0
                self.verticalBrick()
            self.orientation = Orientation(latitudinalAngle, longitudinalAngle, thetaZ)
            self.center = Position((start.x+end.x)/2,(start.y+end.y)/2,(start.z+end.z)/2)
        else:
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
            return (self.center.x == other.center.x and self.center.y == other.center.y 
                    and self.orientation.z == other.orientation.z)
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
IMG_WIDTH  = 1280
IMG_HEIGHT = 720
HOME = Position(30, -15, 40) #cm
IMG_OFFSET = Position(-150-516.3, 300-292.11, 0) #mm
GROUND = Position(None, None, 0)
CATCH_o = (180, 0, None)
threshold = 1 #TODO: define the threshold for a well placed brick

def getImage():
    move(HOME)
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

def brickDetection():
    # task 1
    rawImage, colourImage = getImage()
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
        results[i][0], results[i][1] = ((results[i][1]+IMG_OFFSET.y)*103.269/rawImage.width, 
                                        (results[i][0]+IMG_OFFSET.x)*58.422/rawImage.height)
        bricks[i] = Brick(center=Position(results[i][0], results[i][1], GROUND.z+Brick.THICKNESS/2), 
                          plannarAngle=results[i][2])
    return bricks, results

def recognizePattern(rawData, bricks0):
    triangleGroupedData = patternRecognition(rawData) #none redondant list of triangles
    bricks = []
    for i in range(len(triangleGroupedData)):
        for j in range(len(triangleGroupedData[i])):
            bricks.append(Brick(center=Position(triangleGroupedData[i][j][0], 
                triangleGroupedData[i][j][1], -HOME.z), 
                plannarAngle=triangleGroupedData[i][j][2]))
    moves = [None]*min(len(bricks0), len(bricks))
    iUsed = [len(bricks0)]*min(len(bricks0), len(bricks))
    for j in range(min(len(bricks0), len(bricks))):
        iClosest = 0
        for i in range(1, len(bricks0)):
            if (not(bricks0[i] in [bricks[0], bricks[1], bricks[2]])):
                if (i not in iUsed and bricks[j].distance2d(bricks0[i]) < bricks[j].distance2d(bricks0[iClosest])):
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
    gripper.close(Brick.WIDTH)

def release(gripper):
    gripper.open()

def inBoundaries(position):
    """ Boundaries: Table, Joint limits, computer """
    #TODO: complete with the computer limit
    # if (position.z < 0 or position.norm() < 420.8 or 902.0 < position.norm()):
    #     return False
    return True

def move(position, orientation=None):
    if not(inBoundaries(position)):
        print("Attempt to exceed the robot limits:", position)
        sys.exit(1)
    #convert position in meter and orientation in degree (already done for orientations)
    print("Before conversion:", position.x, position.y, position.z)
    x, y, z = position.x/100, position.y/100, position.z/100
    print("After conversion:", position.x, position.y, position.z)

    if orientation==None:
        #keep current orientation
        moveCarteresian(x, y, z)
    else:
        ox, oy, oz = orientation.x, orientation.y, orientation.z
        # ox, oy, oz = orientation.x*180/pi, orientation.y*180/pi, orientation.z*180/pi
        moveCarteresian(x, y, z, ox, oy, oz)
    # task 5
    # robothub code
    pass

def moveBrick(gripper, moves):
    catch(gripper, moves.start, moves.O0)
    for j in range(1, len(moves.positions)):
        move(moves.positions[j], moves.orientations[j])
    release(gripper)
    move(Position(moves[-1].end.x, moves[-1].end.y, moves[-1].end.z+2*Brick.THICKNESS),
        Orientation(moves[-1].OEnd))

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

def pushBrick(brick, expectedMove):
    # task 5
    pass

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
        # Create required services
        base = initBase(router)
        gripper = initGripper(router)
        print("Task 1: brick detection")
        bricks, rawData = brickDetection()
        print(f"Task 2: pattern recognition & prediction: on {len(bricks)} bricks found")
        moves, movedBricks = recognizePattern(rawData, bricks)
        if (len(moves) != len(movedBricks)): 
            print("Incoherent outputs from the pattern recognition")
            sys.exit(1)
        print(f"Task 5 : robot arm movement: {len(moves)} moves are coming.")
        for i in range(len(moves)):
            print(f"Move {i} starts : "+moves[i].display())
            if i==0: print("Task 4: New brick picking")
            moveBrick(gripper, moves)
            if i==0: print("Task 3: Feedback mapping")
            currentBricks = brickDetection()
            if not(brickPlaced(moves[i].start, currentBricks)):
                print("Brick misplaced : push attempt coming")
                misplacedBrick = findMisplaced(moves[i].start, currentBricks)
                moveBrick(gripper, moves)
                # pushBrick(misplacedBrick, moves[i])
    print("Tasks completed !")

if __name__ == '__main__':
    main()
