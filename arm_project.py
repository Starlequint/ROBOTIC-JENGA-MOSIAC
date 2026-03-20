from math import atan2, sqrt, cos, pi
from sys import exit

class Position:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z
    def display(self):
        return f"({round(self.x,3)},{round(self.y,3)},{round(self.z,3)})"
    def __eq__(self, other):
        if isinstance(other, Position):
            return self.x == other.x and self.y == other.y and self.z == other.z
        return False
    def eq2d(self, other):
        if isinstance(other, Position):
            return self.x == other.x and self.y == other.y
        return False
    def distance2d(self, other):
        if isinstance(other, Position):
            return sqrt((self.x-other.x)**2+(self.y-other.y)**2)
        else:
            print("Position::distance2d() misused")
            exit(1)
    def distance(self, other):
        if isinstance(other, Position):
            return sqrt((self.x-other.x)**2+(self.y-other.y)**2+(self.z-other.z)**2)
        else:
            print("Position::distance() misused")
            exit(1)

class Brick:
    def __init__(self, start, end, width=0, latitudinalAngle=0, 
                 longitudinalAngle=0, thickness=0):
        self.start, self.end, self.width, self.thickness = start, end, width, thickness
        self.thetaX, self.thetaY = latitudinalAngle, longitudinalAngle
        if (not(start.eq2d(end))):
            self.thetaZ = atan2(end.y-start.y, end.x-start.x)
            if (self.thetaX != pi/2 and self.thetaX != -pi/2):
                self.length = sqrt((end.y-start.y)**2+(end.x-start.x)**2
                                   )/cos(self.thetaX)
            else:
                self.verticalBick()
        else:
            self.verticalBick()
        self.center = Position((start.x+end.x)/2,(start.y+end.y)/2,(start.z+end.z)/2)
    def verticalBick(self):
        self.length = 0
        print("Unexpected vertical brick")

            
class Move:
    def __init__(self, positions):
        self.positions = positions
        if len(positions) < 2: 
            print("Error: this move lack of positions")
            exit(1)
        else: self.start, self.end = positions[0], positions[-1]
    def display(self):
        return ("from "+self.start.display()+" to "+self.end.display()+
                f" in {len(self.positions)} steps")

#Constants
HOME = Position(1,1,1) #TODO: define the position to take the image
threshold = 1 #TODO: define the threshold for a well placed brick

def getImage():
    move(HOME)
    #code from robothub
    

def brickDetection(image):
    # task 1
    image = getImage()

def recognizePattern(bricks):
    # task 2
    pass

def catch(position, brick):
    # task 4
    move(position)

def move(position):
    # task 5
    pass

def release():
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

def pushBrick(currentPos, expectedPos):
    # task 5
    pass

def main():
    print("Task 1: brick deterction")
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
        catch(moves[i].start, movedBricks[i])
        for j in range(1, len(moves[i].positions)):
            move(moves[i].positions[j])
        release()
        if i==0: print("Task 3: Feedback mapping")
        currentBricks = brickDetection()
        if not(brickPlaced(moves[i].start, currentBricks)):
            print("Brick misplaced : push attempt coming")
            misplacedBrick = findMisplaced(moves[i].start, currentBricks)
            pushBrick(misplacedBrick.center, moves[i].position)
    print("Tasks completed !")

if __name__ == '__main__':
    main()