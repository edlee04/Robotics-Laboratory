import numpy as np
import matplotlib.pyplot as plt
import time
import signal
import ros_robot_controller_sdk as rrc
import sonar

board = rrc.Board()
start = True

WALK_STEP = 70
SHIFT = 20
BACKSHIFT = 1
LIFT_HEIGHT = 91
LEFT_TURN = 110
RIGHT_TURN = 183


n_states = 4         
n_actions = 4
goal_state = 2

# Hyper parameters
#  Learning process:
    # learning_rate (α): How much new info overrides old info.
    # discount_factor (γ): How much future rewards are valued.
    # exploration_prob (ε): Probability of taking a random action.
    # epochs: Number of training episodes.

learning_rate = 0.8
discount_factor = 0.95
exploration_prob = 0.8
epochs = 1000

def Stop(signum, frame):
    global start
    start = False

signal.signal(signal.SIGINT, Stop)

# walk forward half a tile
def forward():

    # set initial position
    board.bus_servo_set_position(0.2, [[2, 200 - LIFT_HEIGHT], [8, 200 - LIFT_HEIGHT], [14, 790 + LIFT_HEIGHT]])
    time.sleep(0.2)

    for _ in range(3):

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

# walk backward half a tile
def backward():

    

    # set initial position
    board.bus_servo_set_position(0.2, [[2, 200 - LIFT_HEIGHT], [8, 200 - LIFT_HEIGHT], [14, 790 + LIFT_HEIGHT]])
    time.sleep(0.2)

    for _ in range(3):

        # Group 1 goes backward half line, Group 2 follow half curve forward down
        board.bus_servo_set_position(0.2, [[4, 500 - WALK_STEP - BACKSHIFT], [10, 500 + WALK_STEP], [16, 500 + WALK_STEP], # Group 1 goes backward half line
                                        [1, 500 + WALK_STEP], [7, 500 + WALK_STEP + BACKSHIFT], [13, 500 - WALK_STEP], # Group 2 goes forward half
                                        [2, 200], [8, 200], [14, 790]]) # Ground Group 2
        time.sleep(0.2)

        # Group 1 goes up forward half curve, Group 2 goes backward half line
        board.bus_servo_set_position(0.2, [[4, 500], [10, 500], [16, 500], # Group 1 goes forward half
                                        [5, 195 - LIFT_HEIGHT], [11, 800 + LIFT_HEIGHT], [17, 800 + LIFT_HEIGHT], # Group 1 raises half
                                        [1, 500], [7, 500], [13, 500]]) # Group 2 goes backward half back to middle ground
        time.sleep(0.2)

        # Group 1 goes forward down half curve, Group 2 goes backward second half line
        board.bus_servo_set_position(0.2, [[4, 500 + WALK_STEP + BACKSHIFT], [10, 500 - WALK_STEP], [16, 500 - WALK_STEP], # Group 1 goes forward
                                        [5, 195], [11, 800], [17, 800], # Group1 1 grounded
                                        [1, 500 - WALK_STEP - BACKSHIFT], [7, 500 - WALK_STEP - BACKSHIFT], [13, 500 + WALK_STEP]]) # Group 2 goes back half line
        time.sleep(0.2)
        
        # Group 1 goes back half line to middle ground, Group 2 follows half curve up forward
        board.bus_servo_set_position(0.2, [[4, 500], [10, 500], [16, 500], # Group 1 goes backward half back to center ground
                                        [1, 500], [7, 500], [13, 500], # Group 2 goes forward half
                                        [2, 200 - LIFT_HEIGHT], [8, 200 - LIFT_HEIGHT], [14, 790 + LIFT_HEIGHT]]) # Group 2 goes up forward curve
        time.sleep(0.2)

# return to home position
def stop():

    # (1, 16), (4, 13), (7, 10)

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

# checks left 45 deg, then checks right 45 deg, returns the sonar detection; if no detection returns 5000
def search():
    
    board.bus_servo_set_position(0.3, [[21, 500]])
    time.sleep(0.7)

    # Group 1: (1, 2, 3) + (7, 8, 9) + (13, 14, 15)
    # Group 2: (4, 5, 6) + (10, 11, 12) + (16, 17, 18)

    if s.getDistance() < 4000:
        return s.getDistance()

    for i in range(3):

        increment = LEFT_TURN

        board.bus_servo_set_position(0.1, [[1, 500], [4, 500], [7, 500], [10, 500], [13, 500], [16, 500],
                                       [2, 200], [5, 195], [8, 200], [11, 800], [14, 790], [17, 800]])
        time.sleep(0.1)

        # CCW everything
        board.bus_servo_set_position(0.1, [[1, 500 - increment], [7, 500 - increment], [13, 500 - increment],
                                       [4, 500 - increment], [10, 500 - increment], [16, 500 - increment]])
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

        if s.getDistance() < 4000:
            return s.getDistance()

    for i in range(5):

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

        if s.getDistance() < 4000:
            return s.getDistance()
    
    # if no reading
    return 5000

actions = {0:forward, 1:backward, 2:stop, 3:search}         
rewards = {0:10, 1:-5, 2:-5, 3:-10}
states = {0:"Too Close", 1:"Good Distance", 2:"Too Far", 3:"No Detection"}

Q_table = np.zeros((n_states, n_actions)) # combination of actions

def get_next_state(key):
    ### alternative approach (reward based on improvements)
    # sonar scan new - sonar scan old
    # --> reward +5 negative (getting closer)
    # --> penalize -5 positive (getting further)
    # --> penalize -1 no change (no detection)

    # perform action
    actions[key]()

    # check state
    sonar_reading = s.getDistance()

    if sonar_reading < 250:
        state = 0
    elif sonar_reading < 500:
        state = 1
    elif sonar_reading < 900:
        state = 2
    else:
        state = 3

    print("Sonar Reading: ", sonar_reading)
    print("State: ", state)
    return state

if __name__ == '__main__':

    s = sonar.Sonar()
    board.bus_servo_set_position(0.1, [[21, 500]])
    time.sleep(0.1)
    # set to default
    board.bus_servo_set_position(0.1, [[1, 500], [4, 500], [7, 500], [10, 500], [13, 500], [16, 500],
                                       [2, 200], [5, 195], [8, 200], [11, 800], [14, 790], [17, 800],
                                       [3, 100], [6, 100], [9, 100], [12, 900], [15, 900], [18, 900]])
    time.sleep(0.1)

    # forward()

    # a = input("Enter: ")

    # backward()

    # a = input("Enter: ")

    # stop()

    # a = input("Enter: ")

    # a = search()
    # print(f"Sonar Reading: {a}")


    for epoch in range(epochs):
        current_state = np.random.randint(0, n_states)  

    while True:
        board.bus_servo_set_position(0.1, [[21, 500]])

        if np.random.rand() < exploration_prob:
            key = np.random.randint(0, n_actions)
            print("** exploration **")
        else:
            key = np.argmax(Q_table[current_state])
            print("** exploitation **")

        next_state = get_next_state(key) # state correlated with action

        reward = rewards[next_state]

        Q_table[current_state, key] += learning_rate * (
            reward + discount_factor * np.max(Q_table[next_state]) - Q_table[current_state, key]
        )

        if next_state == goal_state:
            print("I found you!")
            time.sleep(1)

        current_state = next_state

        print(Q_table)


    # # Output the Learned Q-Table
    # q_values_grid = np.max(Q_table, axis=1).reshape((4, 4))

    # plt.figure(figsize=(6, 6))
    # plt.imshow(q_values_grid, cmap='coolwarm', interpolation='nearest')
    # plt.colorbar(label='Q-value')
    # plt.title('Learned Q-values for Each State')
    # plt.xticks(np.arange(4), ['0', '1', '2', '3'])
    # plt.yticks(np.arange(4), ['0', '1', '2', '3'])
    # plt.gca().invert_yaxis()
    # plt.grid(True)

    # for i in range(4):
    #     for j in range(4):
    #         plt.text(j, i, f'{q_values_grid[i, j]:.2f}', ha='center', va='center', color='black')

    # plt.show()

    # print("Learned Q-table:")
    # print(Q_table)