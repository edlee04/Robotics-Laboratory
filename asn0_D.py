import sys
import time
import signal
import threading
import ros_robot_controller_sdk as rrc
import sonar

# constants
LIFT_HEIGHT = 80

print('''
**********************************************************
********CS/ME 301 Assignment Template*******
**********************************************************
----------------------------------------------------------
Usage:
    sudo python3 asn_template.py
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close the program. If it fails,
      please try multiple times！
----------------------------------------------------------
''')

board = rrc.Board()
start = True

def testlegs():

    board.bus_servo_set_position(1, [[2, 300]])
    time.sleep(1)
    board.bus_servo_set_position(1, [[2, 400]])
    time.sleep(1)

    board.bus_servo_set_position(1, [[5, 300]])
    time.sleep(1)
    board.bus_servo_set_position(1, [[5, 400]])
    time.sleep(1)

    board.bus_servo_set_position(1, [[8, 300]])
    time.sleep(1)
    board.bus_servo_set_position(1, [[8, 400]])
    time.sleep(1)

    board.bus_servo_set_position(1, [[11, 700]])
    time.sleep(1)
    board.bus_servo_set_position(1, [[11, 600]])
    time.sleep(1)

    board.bus_servo_set_position(1, [[14, 700]])
    time.sleep(1)
    board.bus_servo_set_position(1, [[14, 600]])
    time.sleep(1)

    board.bus_servo_set_position(1, [[17, 700]])
    time.sleep(1)
    board.bus_servo_set_position(1, [[17, 600]])
    time.sleep(1)

def readSonar():

    init_time = time.time()

    while time.time() - init_time < 1.0:
        print(f"Distance: {s.getDistance():.2f}")
        time.sleep(0.1)

def readMultipleSonar():

    for i in range(3):

        input(f"\nPlace Object at Desired Location (10cm, 20cm, or 30cm) and Press Enter")

        for j in range(10):
            print(f"Distance: {s.getDistance():.2f}")
            time.sleep(0.1)

def tripod(move):

    if move == "small":

        increment = 50
    
    else:

        increment = 100

    # Front is in Reference to the Sonar Sensor
    # tripod group 1: Front Right (10, 11, 12), Middle Left (4, 5, 6), Back Right (16, 17, 18)
    # tripod group 2: Front Left (7, 8, 9), Middle Right (13, 14, 15), Back Left (1, 2, 3)

    # Logic:
    # Keep Group 1 Grounded, and move first joint backwards to push body forwards
    # Lift Group 2 Legs with second joint and go forward with first joint
    # These two steps are at the same time

    # Then ground the Group 2 Legs with the second joint servo
    # Repeat these steps but alternate the Group Numbers

    board.bus_servo_set_position(0.5, [[10, 500 + increment], [4, 500 - increment], [16, 500 + increment], # Group 1  Pushes Backwards
                                       [8, 400 - LIFT_HEIGHT], [14, 600 + LIFT_HEIGHT], [2, 400 - LIFT_HEIGHT], # Group 2 lifts
                                       [7, 500 +  increment], [13, 500 - increment], [1, 500 + increment]]) # Group 2 goes forward
    time.sleep(0.5)
    
    board.bus_servo_set_position(0.2, [[8, 400], [14, 600], [2, 400]]) # Ground Group 2
    time.sleep(0.2)

    # Reverse the Group Numbers

    board.bus_servo_set_position(0.5, [[7, 500 - increment], [13, 500 + increment], [1, 500 - increment], # Group 2 Pushes Backwards
                                       [11, 600 + LIFT_HEIGHT], [5, 400 - LIFT_HEIGHT], [17, 600 + LIFT_HEIGHT], # Group 1 Lifts
                                       [10, 500 - increment], [4, 500 + increment], [16, 500 - increment]]) # Group 1 goes Forward
    time.sleep(0.5)

    board.bus_servo_set_position(0.2, [[11, 600], [5, 400], [17, 600]]) # Ground Group 1
    time.sleep(0.2)

#    board.bus_servo_set_position(0.2, [[1, 500], [4, 500], [7, 500], 
#                                       [10, 500], [13, 500], [16, 500]]) # reset all first servo to neutral
#    time.sleep(0.2)

def ripple(move):

    if move == "small":

        increment = 60

    else:

        increment = 110

    # Group 1: (7, 8,9), (13, 14, 15) (Front Left, Middle Right)
    # Group 2: (4, 5, 6), (10, 11, 12) (Middle Left, Bottom Right)
    # Group 3: (1, 2, 3), (16, 17, 18) (Bottom Left, Top Right)

    board.bus_servo_set_position(0.5, [[4, 500 - increment], [10, 500 + increment], [1, 500 - increment], [16, 500 + increment], # push Group 2 and 3 backwards
                                       [7, 500 + increment], [13, 500 - increment], # move Group 1 forward
                                       [8, 400 - LIFT_HEIGHT], [14, 600 + LIFT_HEIGHT]]) # move Group 1 up
    time.sleep(0.5)

    board.bus_servo_set_position(0.2, [[8, 400], [14, 600]]) # Ground Group 1
    board.bus_servo_set_position(0.2, [[7, 500], [13, 500]]) # Reset Group 1 First Joint
    time.sleep(0.2)

    board.bus_servo_set_position(0.5, [[7, 500 - increment], [13, 500 + increment], [1, 500 - increment], [16, 500 + increment], # push Group 1 and 3 backwards
                                       [4, 500 + increment], [10, 500 - increment], # move Group 2 forward
                                       [5, 400 - LIFT_HEIGHT], [11, 600 + LIFT_HEIGHT]]) # move Group 2 up
    time.sleep(0.5)

    board.bus_servo_set_position(0.2, [[5, 400], [11, 600]]) # Ground Group 2
    board.bus_servo_set_position(0.2, [[4, 500], [10, 500]]) # Reset Group 2 First Joint
    time.sleep(0.2)

    board.bus_servo_set_position(0.5, [[7, 500 - increment], [13, 500 + increment], [4, 500 - increment], [10, 500 + increment], # push Group 1 and 2 backwards
                                       [1, 500 + increment], [16, 500 - increment], # move Group 3 forward
                                       [2, 400 - LIFT_HEIGHT], [17, 600 + LIFT_HEIGHT]]) # move Group 3 up
    time.sleep(0.5)

    board.bus_servo_set_position(0.2, [[2, 400], [17, 600]]) # Ground Group 3
    board.bus_servo_set_position(0.2, [[1, 500], [16, 500]]) # Reset Group 3 First Joint
    time.sleep(0.2)

def wave(move):

    if move == "small":

        increment = 100

    else:

        increment = 200

    board.bus_servo_set_position(0.5, [[7, 500 - increment],
                                       [8, 400 - LIFT_HEIGHT]])
    time.sleep(0.5)
   
    board.bus_servo_set_position(0.2, [[8, 400]]) # Ground Group 2
    time.sleep(0.2)
   
    board.bus_servo_set_position(0.5, [[4, 500 - increment],
                                       [5, 400 - LIFT_HEIGHT]])
    time.sleep(0.5)
    board.bus_servo_set_position(0.2, [[5, 400]]) # Ground Group 2
    time.sleep(0.2)
   
    board.bus_servo_set_position(0.5, [[1, 500 - 2*increment],
                                       [2, 400 - LIFT_HEIGHT]])
    time.sleep(0.5)
    board.bus_servo_set_position(0.2, [[2, 400]]) # Ground Group 2
    time.sleep(0.2)
   
    board.bus_servo_set_position(0.5, [[16, 500 + increment],
                                       [17, 600 + LIFT_HEIGHT]])
    time.sleep(0.5)
    board.bus_servo_set_position(0.2, [[17, 600]]) # Ground Group 2
    time.sleep(0.2)
   
    board.bus_servo_set_position(0.5, [[13, 500 + increment],
                                       [14, 600 + LIFT_HEIGHT]])
    time.sleep(0.5)
    board.bus_servo_set_position(0.2, [[14, 600]]) # Ground Group 2
    time.sleep(0.2)
   
    board.bus_servo_set_position(0.5, [[10, 500 + 2*increment],
                                       [11, 600 + LIFT_HEIGHT]])
    time.sleep(0.5)
    board.bus_servo_set_position(0.2, [[11, 600]]) # Ground Group 2
    time.sleep(0.2)

    board.bus_servo_set_position(0.2, [[1, 500], [4, 500], [7, 500], 
                                        [10, 500], [13, 500], [16, 500]]) # reset all first servo to neutral
    time.sleep(0.2)

def Stop(signum, frame):
    global start
    start = False

signal.signal(signal.SIGINT, Stop)

# main code writing
if __name__ == '__main__':

    s = sonar.Sonar()
        
    print('\nAssignment 0 for Group D')      
    time.sleep(0.1)

    while True:

        desired = input("\nEnter the desired gait (tripod / ripple / wave): ")
        stepsize = input("\nEnter the desired step size (small / large): ")
        duration = input("\nInput Duration (10 / 30): ")

        duration = int(duration)

        if desired == "tripod":

            init_time = time.time()

            while time.time() - init_time < duration:
                tripod(stepsize)

            board.bus_servo_set_position(0.2, [[1, 500], [4, 500], [7, 500], 
                                              [10, 500], [13, 500], [16, 500]]) # reset all first servo to neutral
            time.sleep(0.2)
        
        elif desired == "ripple":

            init_time = time.time()

            while time.time() - init_time < duration:
                ripple(stepsize)

            board.bus_servo_set_position(0.2, [[1, 500], [4, 500], [7, 500], 
                                              [10, 500], [13, 500], [16, 500]]) # reset all first servo to neutral
            time.sleep(0.2)

        elif desired == "wave":

            init_time = time.time()

            while time.time() - init_time < duration:
                wave(stepsize)

        else:

            testlegs()

        time.sleep(0.1)

'''
Default Configurations

1: 500 (Less is Forwards)
2: 400 (Less is Up)
3: 
4: 500 (Less is Forwards)
5: 400 (Less is Up)
6:
7: 500 (Less is Forwards)
8: 400 (Less is Up)
9:
10: 500 (More is Forwards)
11: 600 (More is Up)
12:
13: 500 (More is Forwards)
14: 600 (More is Up)
15:
16: 500 (More is Forwards)
17: 600 (More is Up)
18:

'''
