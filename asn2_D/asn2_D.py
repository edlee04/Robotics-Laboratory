import sys
import time
import signal
import ros_robot_controller_sdk as rrc
import sonar
import numpy as np
import math
import map
from collections import deque

LIFT_HEIGHT = 80
LIFT = 80
WALK_STEP = 91
BASE_SHIFT = 19
DESIRED = 330 # desired reading from wall
CLAMP = 2

lift = 100
tibia = 50

Kp = 0.07

# V = 10.5 (10.1 is optimal)
# WALK_STEP = 84
# Left Turn = 163; Right Turn = 175

board = rrc.Board()
start = True

pos_i = 0
pos_j = 0
pos_k = 1

def Stop(signum, frame):
    global start
    start = False

def turnLeft():

    # Group 1: (1, 2, 3) + (7, 8, 9) + (13, 14, 15)
    # Group 2: (4, 5, 6) + (10, 11, 12) + (16, 17, 18)

    for i in range(5):

        increment = 188

        board.bus_servo_set_position(0.1, [[1, 500], [4, 500], [7, 500], [10, 500], [13, 500], [16, 500],
                                       [2, 200], [5, 195], [8, 200], [11, 800], [14, 790], [17, 800]])
        time.sleep(0.1)

        # CCW everything
        board.bus_servo_set_position(0.1, [[1, 500 - increment], [7, 500 - increment], [13, 500 - increment],
                                       [4, 500 - increment], [10, 500 - increment], [16, 500 - increment]])
        time.sleep(0.1) 

        # raise Group 1 and home
        board.bus_servo_set_position(0.1, [[2, 200 - LIFT], [8, 200 - LIFT], [14, 790 + LIFT],
                                       [1, 500], [7, 500], [13, 500]])
        time.sleep(0.1)

        # ground Group 1
        board.bus_servo_set_position(0.1, [[2, 200], [8, 200], [14, 790]])
        time.sleep(0.1)
    
        # raise Group 2 and home
        board.bus_servo_set_position(0.1, [[5, 195 - LIFT], [11, 800 + LIFT], [17, 800 + LIFT],
                                       [4, 500], [10, 500], [16, 500]])
        time.sleep(0.1)

        # ground Group 2
        board.bus_servo_set_position(0.1, [[5, 195], [11, 800], [17, 800]])
        time.sleep(0.1)

def turnRight():

    # Group 1: (1, 2, 3) + (7, 8, 9) + (13, 14, 15)
    # Group 2: (4, 5, 6) + (10, 11, 12) + (16, 17, 18)

    for i in range(5):

        increment = 185

        board.bus_servo_set_position(0.1, [[1, 500], [4, 500], [7, 500], [10, 500], [13, 500], [16, 500],
                                       [2, 200], [5, 195], [8, 200], [11, 800], [14, 790], [17, 800]])
        time.sleep(0.1)

        # CW everything
        board.bus_servo_set_position(0.1, [[1, 500 + increment], [7, 500 + increment], [13, 500 + increment],
                                        [4, 500 + increment], [10, 500 + increment], [16, 500 + increment]])
        time.sleep(0.1) 

        # raise Group 1 and home
        board.bus_servo_set_position(0.1, [[2, 200 - LIFT], [8, 200 - LIFT], [14, 790 + LIFT],
                                        [1, 500], [7, 500], [13, 500]])
        time.sleep(0.1)

        # ground Group 1
        board.bus_servo_set_position(0.1, [[2, 200], [8, 200], [14, 790]])
        time.sleep(0.1)
        
        # raise Group 2 and home
        board.bus_servo_set_position(0.1, [[5, 195 - LIFT], [11, 800 + LIFT], [17, 800 + LIFT],
                                        [4, 500], [10, 500], [16, 500]])
        time.sleep(0.1)

        # ground Group 2
        board.bus_servo_set_position(0.1, [[5, 195], [11, 800], [17, 800]])
        time.sleep(0.1)

def sonar_feedback():
        
        s = sonar.Sonar()

        # set to default
        board.bus_servo_set_position(0.1, [[1, 500], [4, 500], [7, 500], [10, 500], [13, 500], [16, 500],
                                           [2, 200], [5, 195], [8, 200], [11, 800], [14, 790], [17, 800],
                                           [3, 100], [6, 100], [9, 100], [12, 900], [15, 900], [18, 900]])
        time.sleep(0.1)

        # move middle left and right legs back to give room for sonar
        board.bus_servo_set_position(0.2, [[4, 650], [13, 350]])
        time.sleep(0.3)

        board.bus_servo_set_position(0.2, [[21, 890]]) # turn sonar left
        time.sleep(1.5)
        left_read = s.getDistance()

        board.bus_servo_set_position(0.2, [[21, 110]])
        time.sleep(1.5)
        right_read = s.getDistance()

        board.bus_servo_set_position(0.2, [[21, 500]])
        time.sleep(0.5) # reset to home

        if (left_read > 550) and (right_read > 550):
            print("No walls detected!")
            return 0
        
        elif (left_read < 550) and (right_read > 550): # only left wall detect

            board.bus_servo_set_position(0.2, [[21, 890]]) # turn sonar left
            time.sleep(1.5)

            if left_read < DESIRED: # too close

                while s.getDistance() < DESIRED:
                    print("Left Wall Too Close. Walking Right!")
                    # crab walk right
                    defaultCrab()
                    crab("right")

            elif left_read > DESIRED: # too far
                
                while s.getDistance() > DESIRED:
                    print("Left Wall Too Far. Walking Left!")
                    # crab walk left
                    defaultCrab()
                    crab("left")

        elif (right_read < 550) and (left_read > 550):# only right wall detect

            board.bus_servo_set_position(0.2, [[21, 110]]) # turn sonar right
            time.sleep(1.5)

            if right_read < DESIRED: # too close

                while s.getDistance() < DESIRED:
                    print("Right Wall Too Close. Walking Left!")
                    # crab walk left
                    defaultCrab()
                    crab("left")

            elif right_read > DESIRED: # too far

                while s.getDistance() > DESIRED:
                    print("Right Wall Too Far. Walking Right!")
                    # crab walk right
                    defaultCrab()
                    crab("right")

        elif (right_read < 550) and (left_read < 550): # if both detected then just follow right side code
            
            board.bus_servo_set_position(0.2, [[21, 110]]) # turn sonar left
            time.sleep(1.5)

            if right_read < DESIRED: # too close

                while s.getDistance() < DESIRED:
                    print("Right Wall Too Close. Walking Left!")
                    # crab walk left
                    defaultCrab()
                    crab("left")

            elif right_read > DESIRED: # too far

                while s.getDistance() > DESIRED:
                    print("Right Wall Too Far. Walking Right!")
                    # crab walk right
                    defaultCrab()
                    crab("right")

        board.bus_servo_set_position(0.2, [[1, 500], [2, 200 - LIFT_HEIGHT], [16, 500], [17, 800 + LIFT_HEIGHT]])
        time.sleep(0.2)
        board.bus_servo_set_position(0.2, [[2, 200], [17, 800]])
        time.sleep(0.2)
        board.bus_servo_set_position(0.2, [[7, 500], [10, 500], [8, 200 - LIFT_HEIGHT], [11, 800 + LIFT_HEIGHT]])
        time.sleep(0.2)
        board.bus_servo_set_position(0.2, [[8, 200], [11, 800]])
        time.sleep(0.2)

        return 0

def tripod(increment):

    # Group 1: 4, 10, 16
    # Group 2: 1, 7, 13

    # Left Legs: 1, 4, 7
    # Right Legs: 10, 13, 16

    shift = SHIFT
    FEED_SHIFT = 0

    for i in range(5):

        board.bus_servo_set_position(0.3, [[10, 500 - increment], [4, 500 + increment - shift + FEED_SHIFT], [16, 500 - increment], # Group 1  Pushes Backwards
                                        [8, 200 - LIFT_HEIGHT], [14, 780 + LIFT_HEIGHT], [2, 200 - LIFT_HEIGHT], # Group 2 lifts
                                        [7, 500 -  increment + shift - FEED_SHIFT], [13, 500 + increment], [1, 500 - increment + shift - FEED_SHIFT]]) # Group 2 goes forward
        time.sleep(0.3)
        
        board.bus_servo_set_position(0.2, [[8, 200], [14, 780], [2, 200]]) # Ground Group 2
        time.sleep(0.2)

        # Reverse the Group Numbers

        FEED_SHIFT = sonar_feedback() # FEED_SHIFT is positive when robot is too close to left wall (DESIRED - reading)

        print(f"\nFEED_SHIFT VALUE: {FEED_SHIFT}")

        board.bus_servo_set_position(0.3, [[7, 500 + increment - shift + FEED_SHIFT], [13, 500 - increment], [1, 500 + increment - shift + FEED_SHIFT], # Group 2 Pushes Backwards
                                        [11, 800 + LIFT_HEIGHT], [5, 195 - LIFT_HEIGHT], [17, 800 + LIFT_HEIGHT], # Group 1 Lifts
                                        [10, 500 + increment], [4, 500 - increment + shift - FEED_SHIFT], [16, 500 + increment]]) # Group 1 goes Forward
        time.sleep(0.3)

        board.bus_servo_set_position(0.2, [[11, 800], [5, 195], [17, 800]]) # Ground Group 1
        time.sleep(0.2)

def new_tripod():

    # Group 1: 4, 10, 16 (should start at middle ground)
    # Group 2: 1, 7, 13 (should start at middle peak)

    # Left Legs: 1, 4, 7
    # Right Legs: 10, 13, 16

    # set initial position
    board.bus_servo_set_position(0.2, [[2, 200 - LIFT_HEIGHT], [8, 200 - LIFT_HEIGHT], [14, 790 + LIFT_HEIGHT]])
    time.sleep(0.2)

    FEED = sonar_feedback()
    SHIFT = BASE_SHIFT

    for _ in range(5):

        # Group 1 goes backward half line, Group 2 follow half curve forward down
        board.bus_servo_set_position(0.2, [[4, 500 + WALK_STEP + SHIFT], [10, 500 - WALK_STEP], [16, 500 - WALK_STEP], # Group 1 goes backward half line
                                        [1, 500 - WALK_STEP], [7, 500 - WALK_STEP - SHIFT], [13, 500 + WALK_STEP], # Group 2 goes forward half
                                        [2, 200], [8, 200], [14, 790]]) # Ground Group 2
        time.sleep(0.2)

        # Group 1 goes up forward half curve, Group 2 goes backward half line
        board.bus_servo_set_position(0.2, [[4, 500], [10, 500], [16, 500], # Group 1 goes forward half
                                        [5, 195 - LIFT_HEIGHT], [11, 800 + LIFT_HEIGHT], [17, 800 + LIFT_HEIGHT], # Group 1 raises half
                                        [1, 500], [7, 500], [13, 500]]) # Group 2 goes backward half back to middle ground
        time.sleep(0.2)

        # Group 1 goes forward down half curve, Group 2 goes backward second half line
        board.bus_servo_set_position(0.2, [[4, 500 - WALK_STEP - SHIFT], [10, 500 + WALK_STEP], [16, 500 + WALK_STEP], # Group 1 goes forward
                                        [5, 195], [11, 800], [17, 800], # Group1 1 grounded
                                        [1, 500 + WALK_STEP + SHIFT], [7, 500 + WALK_STEP + SHIFT], [13, 500 - WALK_STEP]]) # Group 2 goes back half line
        time.sleep(0.2)
        
        # Group 1 goes back half line to middle ground, Group 2 follows half curve up forward
        board.bus_servo_set_position(0.2, [[4, 500], [10, 500], [16, 500], # Group 1 goes backward half back to center ground
                                        [1, 500], [7, 500], [13, 500], # Group 2 goes forward half
                                        [2, 200 - LIFT_HEIGHT], [8, 200 - LIFT_HEIGHT], [14, 790 + LIFT_HEIGHT]]) # Group 2 goes up forward curve
        time.sleep(0.2)

def turn(head): # turn to goal

    global pos_k

    while pos_k != head:

        # (head - pos_k) % 4 returns shortest # of right turns are needed to reach the goal orientation (counting from 1 -> 4 -> 1...)

        if (head - pos_k) % 4 < 3: # if number of right turns required is 1 or 2
            turnRight()
            pos_k = (pos_k) % 4 + 1

        else: # if number of right turns is 3
            turnLeft()
            if pos_k == 1: # would be left turning from 1 -> 4 so pos_k updates to 4
                pos_k = 4
            else:
                pos_k -= 1 # would be left turning so pos_k -= 1

def motion(i, j, k):

    # i is north south
    # j is east west
    # k is heading (North = 1, East = 2, South = 3, West = 4)

    global pos_i
    global pos_j
    global pos_k

    if i < pos_i: # move north

        turn(1) # turn north

        for step in range(pos_i - i):
            new_tripod()
            pos_i -= 1
    
    elif i > pos_i: # if walking direction is south

        turn(3) # turn south

        for step in range(i - pos_i):
            new_tripod()
            pos_i += 1

    ###############################################################

    if j > pos_j: # if walking direction is east

        turn(2) # face east

        for step in range(j - pos_j):
            new_tripod()
            pos_j += 1 # update pos_j + 1

    elif j < pos_j: # if walking direction is west

        turn(4) # face west

        for step in range(pos_j - j):
            new_tripod()
            pos_j -= 1

    ####################################################

    # turn(k) # turn to final heading
    # this line is not necesary i think since the robot is heading in that direcion already

    print(f"\nMotion Done! Internal Global Position: {pos_i}, {pos_j}, {pos_k}")

def neighbors(i, j, dir):
    if dir == map.DIRECTION.North:
        return i-1, j
    elif dir == map.DIRECTION.South:
        return i+1, j
    elif dir == map.DIRECTION.West:
        return i, j-1
    elif dir == map.DIRECTION.East:
        return i, j+1
    
def cost_map(your_map, goal_i, goal_j, start_i, start_j):
    ### Creates a cost map starting at the goal location
    # 1) create a queue that appends the neighboring cells
    # 2) adds +1 cost to neighboring cells if not a wall and not already marked
    # 3) when all cells have been assigned values, presents a path from x_s tp x_g

    ########## 1) #########
    rows = your_map.getCostmapSize(True)
    cols = your_map.getCostmapSize(False)

    # clear old costs
    your_map.clearCostMap()

    # initialize goal
    your_map.setCost(goal_i, goal_j, 0)

    # set up double ended queue
    queue = deque()
    queue.append((goal_i, goal_j)) # start at goal

    directions = [
        map.DIRECTION.North,
        map.DIRECTION.South,
        map.DIRECTION.West,
        map.DIRECTION.East
    ]

    # print(directions)

    ########## 2) #########
    while queue:
        i, j = queue.popleft() # choose first goal
        current_cost = your_map.getCost(i, j) # gets the cost of the current cell you are in

        for dir in directions:
            # if neighbor is a wall, skip
            if your_map.getNeighborObstacle(i, j, dir) == 1:
                continue

            ni, nj = neighbors(i, j, dir)

            # make sure your in bounds
            if ni < 0 or ni >= rows or nj < 0 or nj >= cols:
                continue

            # Assign cost value if neighbor not already assigned and it isn't the goal location
            if your_map.getCost(ni, nj) == 0 and (ni, nj) != (goal_i, goal_j):
                your_map.setCost(ni, nj, current_cost + 1)
                queue.append((ni, nj))
    
    ########## 3) #########
    path = [[start_i, start_j]]
    current_i, current_j = start_i, start_j

    while (current_i, current_j) != (goal_i, goal_j):
        best_cost = float('inf')
        next_cell = None

        for dir in directions:
            # skip walls
            if your_map.getNeighborObstacle(current_i, current_j, dir) == 1:
                continue

            # get neighbor coordinates
            ni, nj = neighbors(current_i, current_j, dir)

            # bounds check
            if ni < 0 or ni >= rows or nj < 0 or nj >= cols:
                continue

            cost = your_map.getCost(ni, nj)

            # choose lowest-cost neighbor
            if cost >= 0 and cost < best_cost:
                best_cost = cost
                next_cell = [ni, nj]

        if next_cell is None:
            print("No valid path found")
            break

        current_i, current_j = next_cell
        path.append(next_cell)
        
    print("Best path:")
    for cell in path:
        print(cell)
    
    return path

def crab(dir):
    # Group 1: 4, 10, 16
    # Group 2: 1, 7, 13
    
    if dir == "right":
        board.bus_servo_set_position(0.3, [[3, 100+tibia], [9, 100+tibia], [15, 900+tibia]]) 
        time.sleep(.5)
        
        board.bus_servo_set_position(0.3, [[2, 200-lift], [8, 220-lift], [14, 780+lift],
                                           [5, 200+lift], [11, 800-lift], [17, 800-lift]])
        time.sleep(.5)

        board.bus_servo_set_position(0.3, [[6, 100+tibia], [12, 900+tibia], [18, 900+tibia]]) 
        time.sleep(.5)
        
        board.bus_servo_set_position(0.3, [[2, 200+lift], [8, 220+lift], [14, 780-lift],
                                           [5, 200-lift], [11, 800+lift], [17, 800+lift]])
        time.sleep(.5)
        
        defaultCrab()

    elif dir == "left":
        board.bus_servo_set_position(0.3, [[3, 100-tibia], [9, 100-tibia], [15, 900-tibia]]) 
        time.sleep(.5)
        
        board.bus_servo_set_position(0.3, [[2, 200-lift], [8, 220-lift], [14, 780+lift],
                                           [5, 200+lift], [11, 800-lift], [17, 800-lift]])
        time.sleep(.5)

        board.bus_servo_set_position(0.3, [[6, 100-tibia], [12, 900-tibia], [18, 900-tibia]]) 
        time.sleep(.5)
        
        board.bus_servo_set_position(0.3, [[2, 200+lift], [8, 220+lift], [14, 780-lift],
                                           [5, 200-lift], [11, 800+lift], [17, 800+lift]])
        time.sleep(.5)
        
        defaultCrab()

def defaultCrab():
    board.bus_servo_set_position(0.1, [[1, 300], [4, 500], [7, 700], [10, 700], [13, 500], [16, 300],
                                       [2, 200], [5, 200], [8, 220], [11, 800], [14, 780], [17, 800],
                                       [3, 100], [6, 100], [9, 100], [12, 900], [15, 900], [18, 900]])
    time.sleep(0.1)
    
signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':

    s = sonar.Sonar()

    # set to default
    board.bus_servo_set_position(0.1, [[1, 500], [4, 500], [7, 500], [10, 500], [13, 500], [16, 500],
                                       [2, 200], [5, 195], [8, 200], [11, 800], [14, 790], [17, 800],
                                       [3, 100], [6, 100], [9, 100], [12, 900], [15, 900], [18, 900]])
    time.sleep(0.1)

    your_map = map.CSME301Map() # generate map object

    your_map.printObstacleMap() # display map with walls and obstacles to terminal

    # Mapping Section: Ask for User Input for Start and End Position in Map
    si, sj, sk, ei, ej, ek = input("\nEnter the starting position and the goal position(si sj sk ei ej ek): ").split()
    si = int(si)
    sj = int(sj)
    sk = int(sk)
    ei = int(ei)
    ej = int(ej)
    ek = int(ek)

    # Generate fastest path from start to end position
    path = cost_map(your_map, ei, ej, si, sj)

    # Set Initial Robot Position to User Defined Positions
    pos_i = si
    pos_j = sj
    pos_k = sk

    # verify Starting Configuration using Print
    print(f"\nSTARTING CONFIG: ({pos_i}, {pos_j}, {pos_k})")

    # print costMap to verify successful path generation
    your_map.printCostMap()

    # clear CostMap
    your_map.clearCostMap()
    
    # go through every instruction in the path generation and perform the action
    # the first entry of path variable is just the user defined start so skipping first cell
    for p in path[1:]:

        # printing destination
        print(f"\nGoing to {p[0], p[1]}")

        # perform motion
        motion(p[0], p[1], pos_k)

        # place an X in the cost map to track robot position in the map and print
        your_map.costMap[p[0]][p[1]] = "X"
        your_map.printCostMap()

        print("="*20)

        # clear costMap
        your_map.clearCostMap()

    turn(ek)

    # valid = True
    # while valid: # check if input of heading is valid

    #     pos_i, pos_j, pos_k = input("Enter the Starting Configuration (i j k): ").split()

    #     pos_i = int(pos_i)
    #     pos_j = int(pos_j)
    #     pos_k = int(pos_k)
        
    #     if (int(pos_k) > 0) and (int(pos_k) < 5):
    #         valid = False
    #     time.sleep(0.1)

    # print(f"Global Position (i, j, k): {pos_i, pos_j, pos_k}")

    # while True:

    #     # ask for goal configuration
    #     end_i, end_j, end_k = input("Enter the End Configuration (i j k):").split()

    #     end_i = int(end_i)
    #     end_j = int(end_j)
    #     end_k = int(end_k)

    #     if not (1 <= end_k <= 4): # if end_k is not between 1 and 4 then skips this loop
    #         print("Heading must be 1 - 4")
    #         continue

    #     print(f"Going from ({pos_i}, {pos_j}, {pos_k}) to ({end_i}, {end_j}, {end_k})")

    #     motion(end_i, end_j, end_k)

    #     print(10 * "=")