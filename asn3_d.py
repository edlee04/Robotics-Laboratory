import numpy as np
import time
import random
import ros_robot_controller_sdk as rrc
import sonar

board = rrc.Board()

# Define Constants #

WALK_STEP   = 70
SHIFT       = 20
BACKSHIFT   = 1
LIFT_HEIGHT = 91
RIGHT_TURN  = 110

# Define Q-Learning Constants #

N_STATES  = 6
N_ACTIONS = 6
GOAL_STATE = 2  # Good Distance

LEARNING_RATE     = 0.7
DISCOUNT_FACTOR   = 0.9
EXPLORATION_START = 1.0
EXPLORATION_MIN   = 0.05
EXPLORATION_DECAY = 0.9

TRAIN_EPOCHS = 10
TRAIN_STEPS  = 10

Q_table = np.zeros((N_STATES, N_ACTIONS))

# State Space Labeling #

STATES = {
    0: "Very Close  (<150 mm)",
    1: "Close       (150–250 mm)",
    2: "Good        (250–450 mm)",
    3: "Far         (450–650 mm)",
    4: "Very Far    (650–1500 mm)",
    5: "Not Detected (>1500 mm)"
}

ACTION_NAMES = {
    0: "Slow Forward",
    1: "Fast Forward",
    2: "Slow Backward",
    3: "Fast Backward",
    4: "Turn Right",
    5: "Stop"
}

REWARDS = {
    0: -20,
    1: -10,
    2: +40,
    3: -10,
    4: -20,
    5: -2
}

# Movement Functions #

def forward(step_scale):
    WALK = int(WALK_STEP * step_scale)
    SHIFTING = int(SHIFT * step_scale)
    # set initial position
    board.bus_servo_set_position(0.2, [[2, 200 - LIFT_HEIGHT], [8, 200 - LIFT_HEIGHT], [14, 790 + LIFT_HEIGHT]])
    time.sleep(0.2)

    for _ in range(3):
        # Group 1 goes backward half line, Group 2 follow half curve forward down
        board.bus_servo_set_position(0.2, [[4, 500 + WALK + SHIFTING], [10, 500 - WALK], [16, 500 - WALK], # Group 1 goes backward half line
                                        [1, 500 - WALK], [7, 500 - WALK - SHIFTING], [13, 500 + WALK], # Group 2 goes forward half
                                        [2, 200], [8, 200], [14, 790]]) # Ground Group 2
        time.sleep(0.2)
        
        # Group 1 goes up forward half curve, Group 2 goes backward half line
        board.bus_servo_set_position(0.2, [[4, 500], [10, 500], [16, 500], # Group 1 goes forward half
                                        [5, 195 - LIFT_HEIGHT], [11, 800 + LIFT_HEIGHT], [17, 800 + LIFT_HEIGHT], # Group 1 raises half
                                        [1, 500], [7, 500], [13, 500]]) # Group 2 goes backward half back to middle ground
        time.sleep(0.2)

        # Group 1 goes forward down half curve, Group 2 goes backward second half line
        board.bus_servo_set_position(0.2, [[4, 500 - WALK - SHIFTING], [10, 500 + WALK], [16, 500 + WALK], # Group 1 goes forward
                                        [5, 195], [11, 800], [17, 800], # Group1 1 grounded
                                        [1, 500 + WALK + SHIFTING], [7, 500 + WALK + SHIFTING], [13, 500 - WALK]]) # Group 2 goes back half line
        time.sleep(0.2)

        # Group 1 goes back half line to middle ground, Group 2 follows half curve up forward
        board.bus_servo_set_position(0.2, [[4, 500], [10, 500], [16, 500], # Group 1 goes backward half back to center ground
                                        [1, 500], [7, 500], [13, 500], # Group 2 goes forward half
                                        [2, 200 - LIFT_HEIGHT], [8, 200 - LIFT_HEIGHT], [14, 790 + LIFT_HEIGHT]]) # Group 2 goes up forward curve
        time.sleep(0.2)

def backward(step_scale):
    WALK = int(WALK_STEP * step_scale)
    BACKSHIFTING = int(BACKSHIFT * step_scale)
    # set initial position
    board.bus_servo_set_position(0.2, [[2, 200 - LIFT_HEIGHT], [8, 200 - LIFT_HEIGHT], [14, 790 + LIFT_HEIGHT]])
    time.sleep(0.2)

    for _ in range(3):

        # Group 1 goes backward half line, Group 2 follow half curve forward down
        board.bus_servo_set_position(0.2, [[4, 500 - WALK - BACKSHIFTING], [10, 500 + WALK], [16, 500 + WALK], # Group 1 goes backward half line
                                        [1, 500 + WALK], [7, 500 + WALK + BACKSHIFTING], [13, 500 - WALK], # Group 2 goes forward half
                                        [2, 200], [8, 200], [14, 790]]) # Ground Group 2
        time.sleep(0.2)

        # Group 1 goes up forward half curve, Group 2 goes backward half line
        board.bus_servo_set_position(0.2, [[4, 500], [10, 500], [16, 500], # Group 1 goes forward half
                                        [5, 195 - LIFT_HEIGHT], [11, 800 + LIFT_HEIGHT], [17, 800 + LIFT_HEIGHT], # Group 1 raises half
                                        [1, 500], [7, 500], [13, 500]]) # Group 2 goes backward half back to middle ground
        time.sleep(0.2)

        # Group 1 goes forward down half curve, Group 2 goes backward second half line
        board.bus_servo_set_position(0.2, [[4, 500 + WALK + BACKSHIFTING], [10, 500 - WALK], [16, 500 - WALK], # Group 1 goes forward
                                        [5, 195], [11, 800], [17, 800], # Group1 1 grounded
                                        [1, 500 - WALK - BACKSHIFTING], [7, 500 - WALK - BACKSHIFTING], [13, 500 + WALK]]) # Group 2 goes back half line
        time.sleep(0.2)
        
        # Group 1 goes back half line to middle ground, Group 2 follows half curve up forward
        board.bus_servo_set_position(0.2, [[4, 500], [10, 500], [16, 500], # Group 1 goes backward half back to center ground
                                        [1, 500], [7, 500], [13, 500], # Group 2 goes forward half
                                        [2, 200 - LIFT_HEIGHT], [8, 200 - LIFT_HEIGHT], [14, 790 + LIFT_HEIGHT]]) # Group 2 goes up forward curve
        time.sleep(0.2)

def turnRight():

    # Group 1: (1, 2, 3) + (7, 8, 9) + (13, 14, 15)
    # Group 2: (4, 5, 6) + (10, 11, 12) + (16, 17, 18)

    # should turn by 9 degrees ish

    increment = RIGHT_TURN

    board.bus_servo_set_position(0.1, [[1, 500], [4, 500], [7, 500], [10, 500], [13, 500], [16, 500],
                                    [2, 200], [5, 195], [8, 200], [11, 800], [14, 790], [17, 800]])
    time.sleep(0.1)

    # CW everything
    board.bus_servo_set_position(0.1, [[1, 500 + increment], [7, 500 + increment], [13, 500 + increment],
                                    [4, 500 + increment], [10, 500 + increment], [16, 500 + increment]])
    time.sleep(0.1) 

    # raise Group 1 and home
    board.bus_servo_set_position(0.1, [[2, 200 - LIFT_HEIGHT], [8, 200 - LIFT_HEIGHT], [14, 790 + LIFT_HEIGHT],
                                    [1, 500], [7, 500], [13, 500]])
    time.sleep(0.1)

    # ground Group 1
    board.bus_servo_set_position(0.1, [[2, 200], [8, 200], [14, 790]])
    time.sleep(0.1)
    
    # raise Group 2 and home
    board.bus_servo_set_position(0.1, [[5, 195 - LIFT_HEIGHT], [11, 800 + LIFT_HEIGHT], [17, 800 + LIFT_HEIGHT],
                                    [4, 500], [10, 500], [16, 500]])
    time.sleep(0.1)

    # ground Group 2
    board.bus_servo_set_position(0.1, [[5, 195], [11, 800], [17, 800]])
    time.sleep(0.1)

def stop():

    board.bus_servo_set_position(0.2, [[2, 200 - LIFT_HEIGHT], [17, 800 + LIFT_HEIGHT]])
    time.sleep(0.2)
    board.bus_servo_set_position(0.2, [[2, 200], [17, 800], [1, 500], [16, 500]])
    time.sleep(0.2)

    board.bus_servo_set_position(0.2, [[5, 195 - LIFT_HEIGHT], [14, 790 + LIFT_HEIGHT]])
    time.sleep(0.2)
    board.bus_servo_set_position(0.2, [[5, 195], [14, 790], [4, 500], [13, 500]])
    time.sleep(0.2)

    board.bus_servo_set_position(0.2, [[8, 200 - LIFT_HEIGHT], [11, 800 + LIFT_HEIGHT]])
    time.sleep(0.2)
    board.bus_servo_set_position(0.2, [[8, 200], [11, 800], [7, 500], [10, 500]])
    time.sleep(0.2)

    # set to default
    board.bus_servo_set_position(0.1, [[1, 500], [4, 500], [7, 500], [10, 500], [13, 500], [16, 500],
                                       [2, 200], [5, 195], [8, 200], [11, 800], [14, 790], [17, 800],
                                       [3, 100], [6, 100], [9, 100], [12, 900], [15, 900], [18, 900]])
    time.sleep(0.1)

ACTIONS = {
    0: lambda: forward(0.5),
    1: lambda: forward(1),
    2: lambda: backward(0.5),
    3: lambda: backward(1),
    4: turnRight,
    5: stop
}

# Detect State Function #

def get_state():

    read = s.getDistance()

    if read < 230:
        state = 0
    elif read < 280:
        state = 1
    elif read < 520:
        state = 2
    elif read < 650:
        state = 3
    elif read < 1500:
        state = 4
    else:
        state = 5 # robot is essentially lost

    print(f"\nSonar Reading: {read}; State: {state}")

    return state

# Choose Action Function #

def choose_action(state, exp_prob):

    if np.random.rand() < exp_prob: # chooses randomly
        print("Exploring with Random Action")
        return np.random.randint(N_ACTIONS)
    
    print("Using Q-Table for Action") # chooses from Q-table
    row = Q_table[state]
    max_val = np.max(row)
    best = [a for a in range(N_ACTIONS) if row[a] == max_val]
    return random.choice(best) # chooses randomly among all the same highest values in a given state

# Q Table Updating (Bellman Equation) #

def updateQ(state, action, shaped_reward, next_state):

    Q_table[state, action] += LEARNING_RATE * (shaped_reward + (DISCOUNT_FACTOR * np.max(Q_table[next_state])) - Q_table[state, action])
    # Using Bellman Equation

# TRAINING PHASE #

# Setup: human stands still; robot starts not on optimal distance
#
# What the robot should learn:
#     State 4 (Very Far)     = Fast Forward   (action 1)
#     State 3 (Far)          = Slow Forward   (action 0)
#     State 2 (Good)         = Stop           (action 6)
#     State 1 (Close)        = Slow Backward  (action 2)
#     State 0 (Very Close)   = Fast Backward  (action 3)
#     State 5 (Not Detected) = Turn Right (action 4)

# At State 5, the robot should learn to turn right. In the optimal environment, the robot will just keep turning until it either
# finds the human or the human enters the range again.

def training():

    global Q_table

    exp_prob = EXPLORATION_START

    print("\nTraning Phase Starting!")
    
    for ep in range(TRAIN_EPOCHS):

        print(f"\nEpisode: {ep + 1}. Epsilon = {exp_prob:.3f}")
        current_state = get_state()
        no_detect = 0

        for step in range(TRAIN_STEPS):

            print(f"\nStep: {step + 1}")
            action = choose_action(current_state, exp_prob)
            print(f"Action Taken: {ACTION_NAMES[action]}")

            ACTIONS[action]()

            next_state = get_state()
            reward = REWARDS[next_state]

            # Pre-Emptive Exit

            if next_state == 5:
                no_detect += 1
                if no_detect >= 3:
                    print("Robot Lost.")
                    updateQ(current_state, action, reward, next_state)
                    break
            else:
                no_detect = 0

            # Loop Prevention

            if current_state == next_state:
                if action == 5:
                    if current_state == GOAL_STATE:
                        reward += 10
                else:
                    if current_state < 3:
                        reward -= 10

            # CASE A: "Found You" Bonus
            if current_state == 5 and next_state < 5:
                reward += 60 # big bonus for re-acquiring target
                print("BONUS: Target Re-Acquired")

            # CASE B: Getting "Warmer"
            elif abs(next_state - GOAL_STATE) < abs(current_state - GOAL_STATE):
                reward += 20
                print("BONUS: Getting Warmer")

            # CASE C: Getting Colder
            elif abs(next_state - GOAL_STATE) > abs(current_state - GOAL_STATE):
                reward -= 20
                print("PENALTY: Getting Colder")

            print(f"Reward: {reward}")

            updateQ(current_state, action, reward, next_state)
            current_state = next_state

        # Epsilon Decreases

        exp_prob = max(EXPLORATION_MIN, exp_prob * EXPLORATION_DECAY) # decreaes until minimum is reached

        input("Reposition the Robot! Press Enter to continue to next Episode: ")

        print(Q_table)

    # print Q-table

    print("\nTRAINNG COMPLETE!")
    np.savetxt("theqtable.csv", Q_table, delimiter=",")
    print(Q_table)

def testing(steps):

    print("\nTesting Phase Starting!")

    current_state = get_state()
    for step in range(steps):

        action = choose_action(current_state, exp_prob=0) # always use Q_table to pick action

        print(f"Action: {ACTION_NAMES[action]}")
        ACTIONS[action]()

        current_state = get_state()

        if current_state == GOAL_STATE:
            print("Reached Optimal Distance. Holding State")

    print("Testing Complete!")

if __name__ == '__main__':

    # set to default
    board.bus_servo_set_position(0.1, [[1, 500], [4, 500], [7, 500], [10, 500], [13, 500], [16, 500],
                                       [2, 200], [5, 195], [8, 200], [11, 800], [14, 790], [17, 800],
                                       [3, 100], [6, 100], [9, 100], [12, 900], [15, 900], [18, 900],
                                       [21, 500]])
    time.sleep(0.1)
    
    s = sonar.Sonar()

    phase = input("\nEnter 1 (Training) or 2 (Testing: will use theqtable.csv): ")

    if phase == "1":

        mode = input("\nEnter 1 (Start Fresh Q-Table) or 2 (From Given Q-Table): ")

        if mode == "1":

            training()
    
            input("\nPress Enter to Start Testing Phase (will use updated Q_table).")

            testing(10)

        elif mode == "2":

            Q_table = np.array([[0, 0, 68.91206265, 0, 0, 0],
                       [0, 0, 0, 299.36836949, 0, 0],
                       [227.69788401, 2.6849186, 116.26648695, 254.35903592, 206.19162027, 403.80334277],
                       [0, 317.61125451, 0,           0,         -25.69,       -7,        ],
                       [-14,          59.04979067,   0,         -10.69298827, -16.7998943,  -14        ],
                       [ 29.77474929, 21.28331659,  16.707019,    26.9489311,   39.18206749, 32.47607296]])
            
            training()

            input("\nPress Enter to Start Testing Phase (will use updated Q_table).")

            testing(10)

    elif phase == "2":

        mode = input("\nEnter 1 (Manual Q) or 2 (Training Q): ")

        if mode == "1":

            Q_table = np.array([[0, 0, 68.91206265, 0, 0, 0],
                       [0, 0, 0, 299.36836949, 0, 0],
                       [227.69788401, 2.6849186, 116.26648695, 254.35903592, 206.19162027, 403.80334277],
                       [0, 317.61125451, 0,           0,         -25.69,       -7,        ],
                       [-14,          59.04979067,   0,         -10.69298827, -16.7998943,  -14        ],
                       [ 29.77474929, 21.28331659,  16.707019,    26.9489311,   39.18206749, 32.47607296]])

        elif mode == "2":

            Q_table = np.loadtxt("theqtable.csv", delimiter=",")
    
        testing(50)
