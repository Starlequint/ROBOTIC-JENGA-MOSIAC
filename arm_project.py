from math import atan2, sqrt, cos, sin, pi
from sys import exit
from JENGA_detection import detect_planks
from TriangleTessellation import patternRecognition
from getImage import get_single_frame
import cv2
from numpy import zeros_like

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
            exit(1)
    def distance2d(self, other):
        if isinstance(other, Position):
            return sqrt((self.x-other.x)**2+(self.y-other.y)**2)
        else:
            print("Position::distance2d() misused")
            exit(1)
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
            exit(1)

class Brick:
    def __init__(self, start=0, center=0, end=0, width=0, latitudinalAngle=0, 
                 longitudinalAngle=0, plannarAngle=0, thickness=0, length=0):
        """orientation: y is along the brick, z on the big side, x is on the small side"""
        if (start != 0 or end != 0):
            self.start, self.end, self.width, self.thickness = start, end, width, thickness
            if (not(start.eq2d(end))):
                thetaZ = atan2(end.y-start.y, end.x-start.x)
                if (self.thetaX != pi/2 and self.thetaX != -pi/2):
                    self.length = sqrt((end.y-start.y)**2+(end.x-start.x)**2
                                    )/cos(self.thetaX)
                else:
                    self.verticalBick()
            else:
                thetaZ = 0
                self.verticalBick()
            self.orientation = Orientation(latitudinalAngle, longitudinalAngle, thetaZ)
            self.center = Position((start.x+end.x)/2,(start.y+end.y)/2,(start.z+end.z)/2)
        else:
            self.center, self.length = center, length
            self.start = Position(center.x-length*cos(plannarAngle), 
                                  center.y-length/2*sin(plannarAngle), center.z)
            self.end = Position(center.x+length*cos(plannarAngle), 
                                  center.y+length/2*sin(plannarAngle), center.z)
            self.orientation = Orientation(latitudinalAngle, longitudinalAngle, plannarAngle)
    def verticalBick(self):
        self.length = 0
        print("Unexpected vertical brick")

            
class Move:
    def __init__(self, positions, orientations):
        self.positions = positions
        self.orientations = orientations
        if len(positions) < 2: 
            print("Error: this move lack of positions")
            exit(1)
        elif len(positions) != len(orientations):
            print("Error: A move must have the same # of orientation and positions")
            exit(1)
        self.start, self.end = positions[0], positions[-1]
        self.thetaZ0, self.thetaZEnd = orientations[0], orientations[-1]
    def display(self):
        return ("from "+self.start.display()+' '+self.thetaZ0.display()+" to "+
                self.end.display()+' '+self.self.thetaZEnd.display()+
                f" in {len(self.positions)} steps")

#Constants
HOME = Position(30, -15, 40)

threshold = 1 #TODO: define the threshold for a well placed brick

def getImage():
    #move(HOME)
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
        bricks[i] = Brick(center=Position(results[i][0], results[i][1], -HOME.z), 
                          plannarAngle=results[i][2]) 
    return bricks, results

def recognizePattern(rawData):
    triangleGroupedData = patternRecognition(rawData)
    # task 2

def catch(position, brick):
    # robothub code
    # task 4
    move(position, Orientation(0, 0, brick.orientation.z))

def inBoundaries(position):
    """ Boundaries: Table, Joint limits, computer """
    #TODO: complete with the computer limit
    if (position.z < 0 or position.norm() < 420.8 or 902.0 < position.norm()):
        return False
    else: return True

def move(position, orientation=None):
    if not(inBoundaries(position)):
        print("Attempt to exceed the robot limits: "+position)
        exit(1)
    if orientation==None:
        #keep current orientation
        pass
    # task 5
    # robothub code
    pass

def release():
    # robothub code
    # task 5
    pass

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

def main():
    print("Task 1: brick detection")
    bricks = brickDetection()
    print(f"Task 2: pattern recognition & prediction: on {len(bricks)} bricks found")
    moves, movedBricks = recognizePattern(bricks)
    if (len(moves) != len(movedBricks)): 
        print("Incoherent outputs from the pattern recognition")
        exit(1)
    print(f"Task 5 : robot arm movement: {len(moves)} moves are coming.")
    for i in range(len(moves)):
        print(f"Move {i} starts : "+moves[i].display())
        if i==0: print("Task 4: New brick picking")
        catch(moves[i].start, moves[i].thetaZ0)
        for j in range(1, len(moves[i].positions)):
            move(moves[i].positions[j], moves[i].orientations[j])
        release()
        if i==0: print("Task 3: Feedback mapping")
        currentBricks = brickDetection()
        if not(brickPlaced(moves[i].start, currentBricks)):
            print("Brick misplaced : push attempt coming")
            misplacedBrick = findMisplaced(moves[i].start, currentBricks)
            pushBrick(misplacedBrick, moves[i])
    print("Tasks completed !")

if __name__ == '__main__':
    main()
