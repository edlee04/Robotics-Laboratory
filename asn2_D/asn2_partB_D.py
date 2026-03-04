import asn2_D
import sonar
import time
from asn2_D import board
import sys

COL = 8
ROW = 8

def enum(**enums):
    return type('Enum', (), enums)

DIRECTION = enum(North=1, East=2, South=3, West=4)

class CSME301GenerateMap():
    def __init__(self):

        n_row = ROW
        n_col = COL

        self.obstacle_size_row = n_row
        self.obstacle_size_col = n_col
        self.costmap_size_row = n_row
        self.costmap_size_col = n_col
        
        self.horizontalWalls = [[0 for x in range(n_col)] for x in range(n_row+1)]
        self.verticalWalls = [[0 for x in range(n_col+1)] for x in range(n_row)]
        self.costMap = [[0 for x in range(n_col)] for x in range(n_row)]

        # from IPython import embed; embed()
        
        for i in range(n_row+1):
            for j in range(n_col):
                self.horizontalWalls[i][j] = 1

        for i in range(n_row):
            for j in range(n_col+1):
                self.verticalWalls[i][j] = 1
        
    # ***********************************************************************
    # Function Name : getNeighborObstacle
    # Description   : Checks if the neighboring cell is blocked on the map.
    # Input         : i: The row coordinate of the current cell on the map.
    #               : j: The column coordinate of the current cell on the map
    #               : dir: A Direction enumeration (North, South, East, West)
    #               :      indicating which neighboring cell to check for
    #               :      obstacles
    # Output        : None
    # Return        : 1 if neighboring cell is blocked, 0 if neighboring cell
    #               : is clear, -1 if index i or j is out of bounds
    # ***********************************************************************/
    def getNeighborObstacle(self, i, j, dir):
        if (((i < 0 or i > (self.costmap_size_row - 1) or j < 0 or j >  (self.costmap_size_col))
             and (dir == DIRECTION.West or dir == DIRECTION.East)) 
            and ((j < 0 or j > (self.costmap_size_col - 1) or i < 0 or i > self.costmap_size_row)
                 and (dir == DIRECTION.North or dir == DIRECTION.South))):
            print("ERROR (getNeighborObstacle): index out of range")
            return -1

        isBlocked = 0
        if dir == DIRECTION.North:
            isBlocked = self.horizontalWalls[i][j]
        elif dir == DIRECTION.South:
            isBlocked = self.horizontalWalls[i+1][j]
        elif dir == DIRECTION.West:
            isBlocked = self.verticalWalls[i][j]
        elif dir == DIRECTION.East:
            isBlocked = self.verticalWalls[i][j+1]

        return isBlocked

    # ******************************************************************************
    # Function Name  : setObstacle
    # Description    : Used for map building, sets the obstacle status of a given map cell
    # Input          : i: The row coordinate of the current cell on the map.
    #                : j: The column coordinate of the current cell on the map
    #                : isBlocked: A boolean (0 or 1) value indicated if the cell is blocked
    #                : dir: A Direction enumeration (North, South, East, West) indicating
    #                :      which neighboring cell to set for obstacles
    # Output         : None
    # Return         : 0 if successful, -11 if i or j is out of map bounds, -2 if isBlocked is not 0 or 1
    # *****************************************************************************/
    def setObstacle(self, i, j, isBlocked, dir):
        if (((i < 0 or i > (self.costmap_size_row - 1) or j < 0 or j > (self.costmap_size_col))
             and (dir == DIRECTION.West or dir == DIRECTION.East))
             or ((j < 0 or j > (self.costmap_size_col - 1) or i < 0 or i > (self.costmap_size_row))
                 and (dir == DIRECTION.North or dir == DIRECTION.South))):
            print("ERROR (setObstacle): index out of range, obstacle not set")
            return -1

        if isBlocked > 1:
            print("ERROR (setObstacle): isBlocked not a valid input, obstacle not set")
            return -2

        if dir == DIRECTION.North:
            self.horizontalWalls[i][j] = isBlocked
        elif dir == DIRECTION.South:
            self.horizontalWalls[i+1][j] = isBlocked
        elif dir == DIRECTION.West:
            self.verticalWalls[i][j] = isBlocked
        elif dir == DIRECTION.East:
            self.verticalWalls[i][j+1] = isBlocked

        return 0

    # ******************************************************************************
    # Function Name  : getNeighborCost
    # Description    : Retrieves the calculated cost of a neighboring cell on the map.
    # Input          : i: The row coordinate of the current cell on the map.
    #                : j: The column coordinate of the current cell on the map
    #                : dir: A Direction enumeration (North, South, East, West) indicating
    #                :      which neighboring cell to retrieve the cost.
    # Output         : None
    # Return         : Positive float valued cost for the neighboring cell, -1 on error
    # *****************************************************************************/
    def getNeighborCost(self, i, j, dir):
        if (i < 0 or i > (self.costmap_size_row - 1) or j < 0 or j > (self.costmap_size_col - 1)):
            print("ERROR (getNeighborCost): index out of range")
            return -1

        cellValue = 0
        if dir == DIRECTION.North:
            if (i == 0):
                cellValue = 1000
            else:
                cellValue = self.costMap[i-1][j]
        elif dir == DIRECTION.South:
            if(i == (self.costmap_size_row - 1)):
                cellValue = 1000
            else:
                cellValue = self.costMap[i+1][j]
        elif dir == DIRECTION.West:
            if (j == 0):
                cellValue = 1000
            else:
                cellValue = self.costMap[i][j-1]
        elif dir == DIRECTION.East:
            if (j == (self.costmap_size_col - 1)):
                cellValue = 1000
            else:
                cellValue = self.costMap[i][j+1]

        return cellValue

    # ******************************************************************************
    # Function Name  : setNeighborCost
    # Description    : Sets the calculated cost of a neighboring cell on the map.
    # Input          : i: The row coordinate of the current cell on the map.
    #                : j: The column coordinate of the current cell on the map
    #                : dir: A Direction enumeration (North, South, East, West) indicating
    #                :      which neighboring cell to retrieve the cost.
    #                : val: Positive float valued cost for the neighboring cell
    # Output         : None
    # Return         : None
    # *****************************************************************************/
    def setNeighborCost(self, i, j, dir, val):
        if (i < 0 or i > (self.costmap_size_row - 1) or j < 0 or j > (self.costmap_size_col - 1)):
            print("ERROR (setNeighborCost): index out of range, value not set")
            return

        if dir == DIRECTION.North:
            if (i > 0):
                self.costMap[i-1][j] = val
        elif dir == DIRECTION.South:
            if (i < (self.costmap_size_row - 1)):
                self.costMap[i+1][j] = val
        elif dir == DIRECTION.West:
            if (j > 0):
                self.costMap[i][j-1] = val
        elif dir == DIRECTION.East:
            if (j < (self.costmap_size_col - 1)):
                self.costMap[i][j+1] = val

    # ******************************************************************************
    # Function Name  : setCost
    # Description    : Used for map building, sets the calculated cost of a given map cell
    # Input          : i: The row coordinate of the current cell on the map.
    #                : j: The column coordinate of the current cell on the map
    #                : val: An integer value (0 to 1023) indicated the cost of a map cell
    # Output         : None
    # Return         : 0 if successful, -1 if i or j is out of map bounds
    # *****************************************************************************/
    def setCost(self, i, j, val):
        if (i < 0 or i > (self.costmap_size_row - 1) or j < 0 or j > (self.costmap_size_col - 1)):
            print("ERROR (setCost): index out of range")
            return -1

        self.costMap[i][j] = val
        return 0

    # ******************************************************************************
    # Function Name  : getCost
    # Description    : Used for map building, gets the calculated cost of a given map cell
    # Input          : i: The row coordinate of the current cell on the map.
    #                : j: The column coordinate of the current cell on the map
    # Output         : None
    # Return         : cost >= 0 if successful, -1 if i or j is out of map bounds
    # *****************************************************************************/
    def getCost(self, i, j):
        if (i < 0 or i > (self.costmap_size_row - 1) or j < 0 or j > (self.costmap_size_col - 1)):
            print(f"ERROR (getCost): index out of range")
            return -1

        return self.costMap[i][j]

    # ******************************************************************************
    # Function Name  : clearCostMap
    # Description    : Sets all of the values in the cost map to 0
    # Input          : None
    # Output         : None
    # Return         : None
    # *****************************************************************************/
    def clearCostMap(self):
        for i in range(self.costmap_size_row):
            for j in range(self.costmap_size_col):
                self.costMap[i][j] = 0

    # ******************************************************************************
    # Function Name  : clearObstacleMap
    # Description    : Sets all of the values in the obstacle map to 0
    # Input          : None
    # Output         : None
    # Return         : None
    # *****************************************************************************/
    def clearObstacleMap(self):
        for i in range(self.costmap_size_row):
            for j in range(self.costmap_size_col + 1):
                self.verticalWalls[i][j] = 0

        for i in range(self.costmap_size_row + 1):
            for j in range(self.costmap_size_col):
                self.horizontalWalls[i][j] = 0

    # ******************************************************************************
    # Function Name  : printCostMap
    # Description    : When connected to a terminal, will print out the 4x6 cost map
    # Input          : None
    # Output         : None
    # Return         : None
    # *****************************************************************************/
    def printCostMap(self):
        print("Cost Map:")
        for i in range(self.costmap_size_row):
            for j in range(self.costmap_size_col):
                print(str(self.costMap[i][j]), end=" "),
            # from IPython import embed; embed()

            print(" ")

    # ******************************************************************************
    # Function Name  : printObstacleMap
    # Description    : When connected to a terminal, will print out the 4x6 obstacle map
    # Input          : None
    # Output         : None
    # Return         : None
    # *****************************************************************************/
    def printObstacleMap(self):
        print("Obstacle Map: ")
        for i in range(self.costmap_size_row):
            for j in range(self.costmap_size_col):
                if (self.horizontalWalls[i][j] == 0):
                    if i == 0:
                        sys.stdout.write(" ---")
                    else:
                        sys.stdout.write("    ")
                else:
                    sys.stdout.write(" ---")

            print(" ")
            for j in range(self.costmap_size_col):
                if (self.verticalWalls[i][j] == 0):
                    if j == self.costmap_size_col - 1:
                        sys.stdout.write("  O |")
                    elif j == 0:
                        sys.stdout.write("| O ")
                    else:
                        sys.stdout.write("  O ")
                else:
                    if j == self.costmap_size_col - 1:
                        sys.stdout.write("| O |")
                    else:
                        sys.stdout.write("| O ")
            print(" ")
        for j in range(self.costmap_size_col):
                sys.stdout.write(" ---")
        print(" ")

    # ******************************************************************************
    # Function Name  : getCostmapSize
    # Description    : Retrieve the size of a given dimension of the costmap
    # Input          : bool rowDim (true for row dimension, false for column dimension)
    # Output         : None
    # Return         : costmap size in the requested dimension
    # *****************************************************************************/
    def getCostmapSize(self, rowDim):
        if (rowDim):
            return self.costmap_size_row
        else:
            return self.costmap_size_col

    # ******************************************************************************
    # Function Name  : getObstacleMapSize
    # Description    : Retrieve the size of a given dimension of the Obstacle Map
    # Input          : bool rowDim (true for row dimension, false for col dimension)
    # Output         : None
    # Return         : obstacle map size in the requested dimension
    # *****************************************************************************/
    def getObstacleMapSize(self, rowDim):
        if rowDim:
            return self.obstacle_size_row
        else:
            return self.obstacle_size_col

def opposite(direc):

    if direc == 1:
        return 3
    
    elif direc == 2:
        return 4
    
    elif direc == 3:
        return 1
    
    elif direc == 4:
        return 2

def step_forward():

    asn2_D.new_tripod()

    if asn2_D.pos_k == 1:
        asn2_D.pos_i -= 1
    elif asn2_D.pos_k == 2:
        asn2_D.pos_j += 1
    elif asn2_D.pos_k == 3:
        asn2_D.pos_i += 1
    elif asn2_D.pos_k == 4:
        asn2_D.pos_j -= 1

    time.sleep(0.2)

    print(f"\nMoved to: ({asn2_D.pos_i}, {asn2_D.pos_j}, {asn2_D.pos_k})")

def sense_wall():

    if s.getDistance() < 350:
        return 1 # wall detected
    else:
        return 0

def dfs_search(gen_map, i, j, visited):

    visited.add((i, j)) # add location currently at into visited

    print(f"\nSearching cell: ({i}, {j})") # print searching cell

    directions = [
        DIRECTION.North,
        DIRECTION.East,
        DIRECTION.South,
        DIRECTION.West
    ]

    for d in directions:

        board.bus_servo_set_position(0.5, [[21, 500]])
        time.sleep(0.5)    

        asn2_D.turn(d)
        time.sleep(3)

        # if wall is detected, detect = 1; if no wall is detected, detect = 0
        detect = sense_wall()

        # update the cell wall being faced from the current tile; detect = 1 sets wall, detect = 0 removes wall
        gen_map.setObstacle(i, j, detect, d)

        ni, nj = asn2_D.neighbors(i, j, d)

        # update neighbor opposite wall
        if 0 <= ni < 8 and 0 <= nj < 8:
            gen_map.setObstacle(ni, nj, detect, opposite(d))

    gen_map.printObstacleMap() # print updated map

    for d in directions:

        board.bus_servo_set_position(0.5, [[21, 500]])
        time.sleep(0.5)

        if gen_map.getNeighborObstacle(i, j, d) == 1: # wall in that direction
            continue

        ni, nj = asn2_D.neighbors(i, j, d)

        if not (0 <= ni < 8 and 0 <= nj < 8): # outside boundaries
            continue

        if (ni, nj) not in visited:

            print(f"\nMoving to ({ni}, {nj})")

            asn2_D.turn(d)
            step_forward()

            dfs_search(gen_map, ni, nj, visited)

            # backtrack to previous node

            asn2_D.turn(opposite(d))
            step_forward()
            asn2_D.turn(d)

# if __name__ == "__main__":

#     global s
#     s = sonar.Sonar()

#     # set to default
#     board.bus_servo_set_position(0.1, [[1, 500], [4, 500], [7, 500], [10, 500], [13, 500], [16, 500],
#                                        [2, 200], [5, 195], [8, 200], [11, 800], [14, 790], [17, 800],
#                                        [3, 100], [6, 100], [9, 100], [12, 900], [15, 900], [18, 900]])
#     time.sleep(0.1)

#     a = input("\nEnter to Start: ")

#     # Map Defaults
#     gen_map = CSME301GenerateMap()
#     gen_map.printObstacleMap()
    
#     asn2_D.pos_i = 1
#     asn2_D.pos_j = 3
#     asn2_D.pos_k = 1

#     # create list of visited nodes
#     # set allows for no duplicates
#     visited = set()

#     dfs_search(gen_map, 1, 3, visited)

#     print("\nSearch Done!")
#     gen_map.printObstacleMap() # print updated map after searching

    