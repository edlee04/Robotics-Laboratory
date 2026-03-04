import sys
import time
import signal
import ros_robot_controller_sdk as rrc
import sonar
import numpy as np

LIFT = 80 # lift height of leg in gait
DESIRED = 400 # deesired distance from wall
CLAMP = 6 # servo adjustment max
WALL_DIRECTION = "left" # relative to front of robot (left or right)
INCREMENT = 65

# define controller gains
Kp = 0.1
Ki = 1
Kd = 0.05

board = rrc.Board()
start = True

def Stop(signum, frame):
    global start
    start = False

def readRuns(duration):

    init_time = time.time()

    while time.time() - init_time < duration:
        tripod(INCREMENT, 0)

def turnLeft():

    # Group 1: (1, 2, 3) + (7, 8, 9) + (13, 14, 15)
    # Group 2: (4, 5, 6) + (10, 11, 12) + (16, 17, 18)

    for i in range(4):

        increment = 170

        board.bus_servo_set_position(0.1, [[1, 500], [4, 500], [7, 500], [10, 500], [13, 500], [16, 500],
                                       [2, 400], [5, 400], [8, 400], [11, 600], [14, 600], [17, 600]])
        time.sleep(0.1)

        # CCW everything
        board.bus_servo_set_position(0.1, [[1, 500 - increment], [7, 500 - increment], [13, 500 - increment],
                                       [4, 500 - increment], [10, 500 - increment], [16, 500 - increment]])
        time.sleep(0.1) 

        # raise Group 1 and home
        board.bus_servo_set_position(0.1, [[2, 400 - LIFT], [8, 400 - LIFT], [14, 600 + LIFT],
                                       [1, 500], [7, 500], [13, 500]])
        time.sleep(0.1)

        # ground Group 1
        board.bus_servo_set_position(0.1, [[2, 400], [8, 400], [14, 600]])
        time.sleep(0.1)
    
        # raise Group 2 and home
        board.bus_servo_set_position(0.1, [[5, 400 - LIFT], [11, 600 + LIFT], [17, 600 + LIFT],
                                       [4, 500], [10, 500], [16, 500]])
        time.sleep(0.1)

        # ground Group 2
        board.bus_servo_set_position(0.1, [[5, 400], [11, 600], [17, 600]])
        time.sleep(0.1)

def turnRight():

    # Group 1: (1, 2, 3) + (7, 8, 9) + (13, 14, 15)
    # Group 2: (4, 5, 6) + (10, 11, 12) + (16, 17, 18)

    for i in range(4):

        increment = 180

        board.bus_servo_set_position(0.1, [[1, 500], [4, 500], [7, 500], [10, 500], [13, 500], [16, 500],
                                       [2, 400], [5, 400], [8, 400], [11, 600], [14, 600], [17, 600]])
        time.sleep(0.1)

        # CW everything
        board.bus_servo_set_position(0.1, [[1, 500 + increment], [7, 500 + increment], [13, 500 + increment],
                                        [4, 500 + increment], [10, 500 + increment], [16, 500 + increment]])
        time.sleep(0.1) 

        # raise Group 1 and home
        board.bus_servo_set_position(0.1, [[2, 400 - LIFT], [8, 400 - LIFT], [14, 600 + LIFT],
                                        [1, 500], [7, 500], [13, 500]])
        time.sleep(0.1)

        # ground Group 1
        board.bus_servo_set_position(0.1, [[2, 400], [8, 400], [14, 600]])
        time.sleep(0.1)
        
        # raise Group 2 and home
        board.bus_servo_set_position(0.1, [[5, 400 - LIFT], [11, 600 + LIFT], [17, 600 + LIFT],
                                        [4, 500], [10, 500], [16, 500]])
        time.sleep(0.1)

        # ground Group 2
        board.bus_servo_set_position(0.1, [[5, 400], [11, 600], [17, 600]])
        time.sleep(0.1)

def turn180(direction):

    if direction == "left":

        turnLeft()
        turnLeft()

    else:

        turnRight()
        turnRight()

def tripod(increment, shift): # parameters: wall direction, step size, shift from controller

    global sonar_position

    # Group 1: (1, 2, 3) + (7, 8, 9) + (13, 14, 15)
    # Group 2: (4, 5, 6) + (10, 11, 12) + (16, 17, 18)

    increment = int(increment)
    shift = int(shift)

    print(f"Increment: {increment}")
    print(f"Shift Amount: {shift}")

    leftleg_steer = increment - shift
    rightleg_steer = increment + shift

    print(f"Left Leg Steering: {leftleg_steer}. Right Leg Steering: {rightleg_steer}")

    # clamp the steer value
    leftleg_steer = np.clip(leftleg_steer, -125, 125)
    rightleg_steer = np.clip(rightleg_steer, -125, 125)

    board.bus_servo_set_position(0.2, [[2, 400 - LIFT], [8, 400 - LIFT], [14, 600 + LIFT], # Group 1 lifts
                                       [1, 500 -  leftleg_steer], [7, 500 - leftleg_steer], [13, 500 + rightleg_steer], # Group 1 moves forward
                                       [4, 500 + leftleg_steer], [10, 500 - rightleg_steer], [16, 500 - rightleg_steer]]) # Group 2 goes back
    time.sleep(0.2)
    
    board.bus_servo_set_position(0.2, [[2, 400], [8, 400], [14, 600]]) # Ground Group 1
    time.sleep(0.2)

    ###

    board.bus_servo_set_position(0.2, [[5, 400 - LIFT], [11, 600 + LIFT], [17, 600 + LIFT], # Group 2 lifts
                                       [1, 500 +  leftleg_steer], [7, 500 + leftleg_steer], [13, 500 - rightleg_steer], # Group 2 moves forward
                                       [4, 500 - leftleg_steer], [10, 500 + rightleg_steer], [16, 500 + rightleg_steer]]) # Group 1 goes back
    time.sleep(0.2)
    
    board.bus_servo_set_position(0.2, [[5, 400], [11, 600], [17, 600]]) # Ground Group 2
    time.sleep(0.2)

    # adjust the sonar servo position to make it approximately parallel to the wall
    '''
    sonar_position = int(np.clip(sonar_position - (shift / 3), 110, 890))
    board.bus_servo_set_position(0.1, [[21, sonar_position]])
    time.sleep(0.1)
    '''

    print(f"Sonar Reading: {s.getDistance()}")

def checkObstacles():

    # Check for Obstacles

    # Check Front
    board.bus_servo_set_position(0.5, [[21, 500]])
    time.sleep(0.5)

    print(f"Distance Read: {s.getDistance()}")
    if s.getDistance() > DESIRED:
        front = "clear"
    else:
        front = "blocked"
    print(f"Front: {front}")

    # if front is clear, automatically walk and exit function
    if front == "clear":
        tripod(INCREMENT, 0)
        return 0

    # Check Left
    board.bus_servo_set_position(0.5, [[21,890]]) # face left
    time.sleep(2)

    if s.getDistance() > DESIRED:
        left = "clear"
    else:
        left = "blocked"

    print(f"Left: {left}")
    print(f"Distance Read: {s.getDistance()}")
   
    ## Right
    board.bus_servo_set_position(0.5, [[21,110]]) # face right
    time.sleep(2)
    if s.getDistance() > DESIRED:
        right = "clear"
    else:
        right = "blocked"

    print(f"Right: {right}")
    print(f"Distance Read: {s.getDistance()}")
    time.sleep(1)
       
    print(f"Status -> Left: {left}, Front: {front}, Right: {right}")

    # reset back to home
    board.bus_servo_set_position(0.2, [[21, 500]])
    time.sleep(0.2) 

    # Decision

    if (left == "blocked" and front == "blocked" and right == "blocked"):
       
        # alarm for fun
        board.set_buzzer(1900, 0.1, 0.9, 1) #(The buzzer sounds at a frequency of 1900Hz for 0.1 seconds followed by a pause of 0.9 seconds, repeating this pattern once)
        time.sleep(2)
        board.set_buzzer(1000, 0.5, 0.5, 0) #(The buzzer sounds at a frequency of 1000Hz for 0.5 seconds followed by a pause of 0.5 seconds, repeating this pattern continually)
        time.sleep(3)
        board.set_buzzer(1000, 0.0, 0.0, 1) #(close)
        turn180("left")
       
    elif (left == "blocked" and front == "blocked" and right == "clear"):
        print("Turn Right") # turn right
        turnRight()

    elif (left == "clear" and front == "blocked" and right == "blocked"):
        print("Turn Left") # turn left
        turnLeft()

    else: # for Left and Right clear
        turnRight()

def feedback_tripod(wall_direction):

    global olde

    # Logic Write Up:

    # If wall is on the right and too close. We WANT to turn LEFT
    # output will be positive
    # Left Legs will steer LESS and Right Legs will steer MORE (turn LEFT)

    # if wall if on the right and too far. We WANT to turn RIGHT
    # output will be negative
    # Left Legs will steer MORE and Right Legs will steer LESS (turn RIGHT)

    # If wall is on the left and too close. We WANT to turn RIGHT
    # output will be negative
    # Left Legs will steer MORE and Right Legs will steer LESS (turn RIGHT)

    # if wall is on the left and too far. We want to LEFT
    # output will be positive
    # Left legs will steer LESS and right legs will steer MORE (turn LEFT)

    timeinit = time.time()

    if wall_direction == "right":
        e = DESIRED - s.getDistance()
    elif wall_direction == "left":
        e = s.getDistance() - DESIRED

    timenew = time.time()

    # reduce noise
    #if abs(e) < 3:
    #    e = 0

    de = e - olde  #/ (timenew - timeinit)

    olde = e

    # Controller Equation to get Shifting Amount
    output = Kp * e # + Kd * de
    output = np.clip(output, -CLAMP, CLAMP)

    return output

#######################################################################
    
signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':

    s = sonar.Sonar()

    # set to default
    board.bus_servo_set_position(0.1, [[1, 500], [4, 500], [7, 500], [10, 500], [13, 500], [16, 500],
                                       [2, 400], [5, 400], [8, 400], [11, 600], [14, 600], [17, 600]])
    time.sleep(0.1)

    olde = 0

    # face sonar towards the wall
    if WALL_DIRECTION == "left":

        board.bus_servo_set_position(0.2, [[21, 890]])
        time.sleep(1)
        sonar_position = 890

    elif WALL_DIRECTION == "right":

        board.bus_servo_set_position(0.2, [[21, 110]])
        time.sleep(1)
        sonar_position = 110

    while True:

        shift = feedback_tripod(WALL_DIRECTION) # shifting amount from controller
        tripod(INCREMENT, shift) # gait with shifting amount