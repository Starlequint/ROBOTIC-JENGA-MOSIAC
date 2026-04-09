from math import sqrt, cos, sin, isclose
from JENGA_detection_v5 import detect_planks
from TriangleTessellation import patternRecognition
from getImage import get_single_frame
from robohub_codes import (example_forward_kinematics, example_inverse_kinematics, 
                           GripperCommandExample)
from move_angular_and_cartesian import mv
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

class Plank:
    LENGTH   : Final[float] = 7.4 # cm
    WIDTH    : Final[float] = 2.4 # cm
    THICKNESS: Final[float] = 1.4 # cm
    def __init__(self, center=0, latitudinalAngle=0, 
                 longitudinalAngle=0, plannarAngle=0):
        """orientation: y is along the plank, z on the big side, x is on the small side"""
        self.center = center
        self.start = Position(center.x-self.LENGTH*cos(plannarAngle), 
                                center.y-self.LENGTH/2*sin(plannarAngle), center.z)
        self.end = Position(center.x+self.LENGTH*cos(plannarAngle), 
                                center.y+self.LENGTH/2*sin(plannarAngle), center.z)
        self.orientation = Orientation(latitudinalAngle, longitudinalAngle, plannarAngle)
    def verticalPlank(self):
        self.length = 0
        print("Unexpected vertical plank")
    def __eq__(self, other):
        if isinstance(other, Plank):
            return (isclose(self.center.x, other.center.x, rel_tol=1e-2) and 
                    isclose(self.center.y, other.center.y, rel_tol=1e-2) and
                    isclose(self.orientation.z, other.orientation.z, rel_tol=5))
        else:
            print("Plank::==() misused")
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
IMG_OFFSET = Position(48, 14, 0) # mm
SCALE = -0.041
GROUND = Position(None, None, 2) # cm
CATCH_o = Orientation(180, 0, None) # °
threshold = 1 #TODO: define the threshold for a well placed plank
GRIPPER_WIDTH = 12 # cm
CX_MIN, CX_MAX, CY_MIN, CY_MAX = -100, 1380, 180, 820

def coordConv(x, y, t):
    return y*SCALE+IMG_OFFSET.x, x*SCALE+IMG_OFFSET.y, 90-t

def getImage():
    move(CAMERA, CAMERA_o, True)
    rtsp_url = "rtsp://admin:admin@192.168.1.10/color"
    frame_array = get_single_frame(rtsp_url)
    frame_depth_normalized = zeros_like(frame_array)
    cv2.normalize(frame_array, frame_depth_normalized, 0, 255, cv2.NORM_MINMAX)
    colourImage = cv2.applyColorMap(frame_depth_normalized,cv2.COLORMAP_JET)
    return frame_array, colourImage
    #code from robothub

def save(image, imagePath):
    # Split path and extension (e.g., 'image.png' -> ('image', '.png'))
    base, ext = os.path.splitext(imagePath)
    i = 0
    new_path = imagePath
    while os.path.exists(new_path):
        i += 1
        new_path = f"{base}{i}{ext}"
    success = cv2.imwrite(new_path, image)
    if success: print(f"Image saved as: {new_path}")
    else:       print(f"Failed to save image at: {new_path}")
    return new_path

def plankDetection():
    # task 1
    rawImage, colourImage = getImage()
    rawImagePath, colourImagePath = 'rawImage.png', 'ColourImage.png'
    rawImagePath = save(rawImage, rawImagePath)
    colourImagePath = save(colourImage, colourImagePath)

    results = detect_planks(
        raw_image_path   = rawImagePath,
        color_image_path = colourImagePath,
        save_path        = 'detected_planks.png',
        show=False
    )
    #results : list of (cx, cy, angle) tuples
    planks = [0]*len(results)
    for i in range(len(results)):
        # convertion to cm
        x, y, t = coordConv(results[i][0], results[i][1], results[i][2])
        planks[i] = Plank(center=Position(x, y, GROUND.z+Plank.THICKNESS/2), 
                          plannarAngle=t)
    return planks, results

def recognizePattern(rawData, planks0):
    triangleGroupedData = patternRecognition(rawData) #none redondant list of triangles
    planks = []
    print(f"Patterned {len(triangleGroupedData*3)} planks")
    for i in range(len(triangleGroupedData)):
        for j in range(len(triangleGroupedData[i])):
            cx, cy = triangleGroupedData[i][j][0], triangleGroupedData[i][j][1]
            t = triangleGroupedData[i][j][2]
            	#if (CX_MIN < cx < CX_MAX and CY_MIN < cy < CY_MAX):
            x, y, t= coordConv(cx, cy, t)
            planks.append(Plank(center=Position(x, y, GROUND.z+Plank.THICKNESS/2), plannarAngle=t))
            print(f"  Plank {i+1:>2}: cx={cx:>6.1f}  cy={cy:>6.1f}"+f"  angle={t:>7.2f}°")
    print(f"Patterned and in screen {len(planks)} planks")
    moves = []
    iUsed = []
    print(min(len(planks0), len(planks))-3, "planks to match")
    for j in range(3, min(len(planks0), len(planks))):
        iClosest = 0
        while iClosest in iUsed or not(planks0[iClosest] != planks[0] and 
            planks0[iClosest] != planks[1] and planks0[iClosest] != planks[2]): 
            iClosest +=1
        for i in range(iClosest+1, len(planks0)):
            if (planks0[i] != planks[0] and planks0[i] != planks[1] and planks0[i] != planks[2]):
                if (i not in iUsed and planks[j].center.distance2d(planks0[i].center) 
                    < planks[j].center.distance2d(planks0[iClosest].center)):
                    iClosest = i
            else: 
                print("model plank: ", planks0[i])
        iUsed.append(iClosest)
        c1, c2 = planks0[iClosest].center, planks[j].center
        moves.append(Move([c1, Position(c1.x, c1.y, c1.z+2*Plank.THICKNESS), 
                           Position(c2.x, c2.y, c2.z+2*Plank.THICKNESS), 
                           Position(c2.x, c2.y, c2.z+0.5*Plank.THICKNESS)], 
                [Orientation(CATCH_o.x, CATCH_o.y, planks0[iClosest].orientation.z), 
                 Orientation(CATCH_o.x, CATCH_o.y, planks0[iClosest].orientation.z), 
                 Orientation(CATCH_o.x, CATCH_o.y, planks[j].orientation.z), 
                 Orientation(CATCH_o.x, CATCH_o.y, planks[j].orientation.z)]))
    return moves, planks

def initGripper(router):
    return GripperCommandExample(router)

def catch():
    """
    position = 0.0 open, 1.0 closed
    """
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        gripper = initGripper(router)
        gripper.close(1-Plank.WIDTH/GRIPPER_WIDTH)

def release():
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        gripper = initGripper(router)
        gripper.open()

def inBoundaries(position):
    """ Boundaries: Table, Joint limits, computer """
    #TODO: complete with the computer limit, and take account of the robot robot orientation
    if (position.z < 0 or position.norm() < 20 or 902.0 < position.norm()):
        return False
    #simplification: x and y limits of the table
    if (position.x < 10 or 80 < position.x or position.y < -80 or 20 < position.y):
        return False
    return True

x0, y0, z0, ox0, oy0, oz0 = 0, 0, 0, 0, 0, 0
def move(position, orientation=None, firstCall=False):
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)
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

def movePlank(move_):
    move(move_.positions[0], move_.orientations[0], False)
    catch()
    for j in range(1, len(move_.positions)):
        move(move_.positions[j], move_.orientations[j], False)
    release()
    move(Position(move_.end.x, move_.end.y, move_.end.z+2*Plank.THICKNESS), 
    			move_.orientations[-1], False)

def plankPlaced(position, planks):
    # task 3
    for plank in planks:
        if plank.center.distance2d(position) < threshold:
            return True
    return False

def findMisplaced(position, planks):
    # task 3
    iMin, minDistance = 0, planks[0].distance2d(position)
    for i in range(1, len(planks)):
        distance = planks[i].distance2d(position)
        if distance < minDistance:
            iMin = i
            minDistance = distance
    return planks[iMin]

def pushPlank(plank, expectedMove):
    # task 5
    #approach the plank and push it towards the expected location

    APPROACH_Z = 2*Plank.THICKNESS
    PUSH_Z = Plank.THICKNESS

    target = expectedMove.end
    yaw = plank.orientation.z
    push_orientation = Orientation(CATCH_o.x, CATCH_o.y, yaw)
    end_orientation = Orientation(CATCH_o.x, CATCH_o.y, expectedMove.OEnd)

    above_start = Position(plank.center.x, plank.center.y, plank.center.z + APPROACH_Z)
    contact_start = Position(plank.center.x, plank.center.y, plank.center.z + PUSH_Z)
    contact_end = Position(target.x, target.y, plank.center.z + PUSH_Z)
    above_end = Position(target.x, target.y, plank.center.z + APPROACH_Z)

    # Keep gripper open for a push correction

    move(above_start,   push_orientation, False)
    move(contact_start, push_orientation, False)
    move(contact_end,   end_orientation , False)
    move(above_end,     end_orientation , False)

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
    release()
    print("Task 1: plank detection")
    planks, rawData = plankDetection()
    print('u', rawData[0][0], 'v', rawData[0][1])
    print(f"Task 2: pattern recognition & prediction: on {len(planks)} planks found")
    moves, movedPlanks = recognizePattern(rawData, planks)
    x, y = [] ,  []
    for move in moves:
        x.append(move.start.x)
        y.append(move.start.y)
    
    print(f"Task 5 : robot arm movement: {len(moves)} moves are coming.")
    for i in range(len(moves)):
        print(f"Move {i} starts : "+moves[i].display())
        if i==0: print("Task 4: New plank picking")
        movePlank(moves[i])
        if i==0: print("Task 3: Feedback mapping")
        #currentPlanks = plankDetection()
        '''if not(plankPlaced(moves[i].start, currentPlanks)):
            print("Plank misplaced : push attempt coming")
            misplacedPlank = findMisplaced(moves[i].start, currentPlanks)
            c1 = misplacedPlank.center
            move = Move([c1, Position(c1.x, c1.y, c1.z+2*Plank.THICKNESS),
                    moves[i].positions[2],
                    moves[i].positions[3]],
                [Orientation(CATCH_o.x, CATCH_o.y, misplacedPlank.orientation.z),
                    Orientation(CATCH_o.x, CATCH_o.y, misplacedPlank.orientation.z),
                    moves[i].orientations[2],
                    moves[i].orientations[3]])
            movePlank(move)
         '''   #pushPlank(args, misplacedPlank, moves[i])
        print("Tasks completed !")

if __name__ == '__main__':
    main()
