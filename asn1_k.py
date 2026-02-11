import map as mp
import sys
import time
import signal
import threading  # use threading to run PID control and tripod simultaneously
import ros_robot_controller_sdk as rrc
import sonar
import matplotlib.pyplot as plt
from collections import deque

#x is south north, south is positive. Y is east west, east is positive
#Algorithm follow the cost gradient
DIRECTION = mp.DIRECTION
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


def Stop(signum, frame):
    global start
    start = False

signal.signal(signal.SIGINT, Stop)

# Right Front Leg
RF_INNER_ID = 7     
RF_MIDDLE_ID = 8     
RF_OUTER_ID = 9      
RF_INNER_DEFAULT = 500
RF_MIDDLE_DEFAULT = 200
RF_OUTER_DEFAULT = 100

# Left Front Leg
LF_INNER_ID = 16
LF_MIDDLE_ID = 17
LF_OUTER_ID = 18
LF_INNER_DEFAULT = 500
LF_MIDDLE_DEFAULT = 800
LF_OUTER_DEFAULT = 900

# Right Middle Leg 
RM_INNER_ID = 4
RM_MIDDLE_ID = 5
RM_OUTER_ID = 6
RM_INNER_DEFAULT = 500
RM_MIDDLE_DEFAULT = 200
RM_OUTER_DEFAULT = 100

# Left Middle Leg
LM_INNER_ID = 13
LM_MIDDLE_ID = 14
LM_OUTER_ID = 15
LM_INNER_DEFAULT = 500
LM_MIDDLE_DEFAULT = 800
LM_OUTER_DEFAULT = 900

# Right Back Leg
RB_INNER_ID = 1
RB_MIDDLE_ID = 2
RB_OUTER_ID = 3
RB_INNER_DEFAULT = 500
RB_MIDDLE_DEFAULT = 200
RB_OUTER_DEFAULT = 100

# Left Back Leg
LB_INNER_ID = 10
LB_MIDDLE_ID = 11
LB_OUTER_ID = 12
LB_INNER_DEFAULT = 500
LB_MIDDLE_DEFAULT = 800
LB_OUTER_DEFAULT = 900

#Platform's value
P_ID = 21
P_DEFAULT = 500
P_RIGHT = 130
P_LEFT = 870


#Centering control parameters
CORRIDOR_HALF_WIDTH = 200  # mm, target distance from side wall
CENTERING_KP = 0.5
CENTERING_KD = 2.0
MAX_CORRECTION = 100

# North=1, East=2, South=3, West=4

def set_all_default():
    board.bus_servo_set_position(1, [
        # Right Front
        [RF_INNER_ID, RF_INNER_DEFAULT], 
        [RF_MIDDLE_ID, RF_MIDDLE_DEFAULT], 
        [RF_OUTER_ID, RF_OUTER_DEFAULT],
        # Right Middle
        [RM_INNER_ID, RM_INNER_DEFAULT], 
        [RM_MIDDLE_ID, RM_MIDDLE_DEFAULT], 
        [RM_OUTER_ID, RM_OUTER_DEFAULT],
        # Right Back
        [RB_INNER_ID, RB_INNER_DEFAULT], 
        [RB_MIDDLE_ID, RB_MIDDLE_DEFAULT], 
        [RB_OUTER_ID, RB_OUTER_DEFAULT],
        # Left Front
        [LF_INNER_ID, LF_INNER_DEFAULT], 
        [LF_MIDDLE_ID, LF_MIDDLE_DEFAULT], 
        [LF_OUTER_ID, LF_OUTER_DEFAULT],
        # Left Middle
        [LM_INNER_ID, LM_INNER_DEFAULT], 
        [LM_MIDDLE_ID, LM_MIDDLE_DEFAULT], 
        [LM_OUTER_ID, LM_OUTER_DEFAULT],
        # Left Back
        [LB_INNER_ID, LB_INNER_DEFAULT], 
        [LB_MIDDLE_ID, LB_MIDDLE_DEFAULT], 
        [LB_OUTER_ID, LB_OUTER_DEFAULT],
        #Platform
        [P_ID, P_DEFAULT]
    ])
    time.sleep(1)

def tripod(dur=0.3, pu=0.3, lif=100, rot=105):
    duration = dur
    pause = pu
    lift = lif
    rotation = rot

    t0 = time.perf_counter()
    board.bus_servo_set_position(duration, [[RB_MIDDLE_ID, RB_MIDDLE_DEFAULT-lift], [RF_MIDDLE_ID, RF_MIDDLE_DEFAULT-lift], [LM_MIDDLE_ID, LM_MIDDLE_DEFAULT+lift], [RB_INNER_ID, RB_INNER_DEFAULT-rotation+7], [RF_INNER_ID, RF_INNER_DEFAULT-rotation+7], [LM_INNER_ID, LM_INNER_DEFAULT+rotation+7], [RM_INNER_ID, RM_INNER_DEFAULT+rotation], [LB_INNER_ID, LB_INNER_DEFAULT-rotation], [LF_INNER_ID, LF_INNER_DEFAULT-rotation]]) # Initial lift of legs and rotation
    time.sleep(pause)
    print(f"\n[t = {time.perf_counter() - t0:.1f} s]")
    print(s.getDistance())

    board.bus_servo_set_position(duration, [[RB_MIDDLE_ID, RB_MIDDLE_DEFAULT], [RF_MIDDLE_ID, RF_MIDDLE_DEFAULT], [LM_MIDDLE_ID, LM_MIDDLE_DEFAULT]]) # Putting legs back down
    time.sleep(pause)
    print(f"\n[t = {time.perf_counter() - t0:.1f} s]")
    print(s.getDistance())

    board.bus_servo_set_position(duration, [[RM_MIDDLE_ID, RM_MIDDLE_DEFAULT-lift], [LB_MIDDLE_ID, LB_MIDDLE_DEFAULT+lift], [LF_MIDDLE_ID, LF_MIDDLE_DEFAULT+lift], [RM_INNER_ID, RM_INNER_DEFAULT-rotation], [LB_INNER_ID, LB_INNER_DEFAULT+rotation], [LF_INNER_ID, LF_INNER_DEFAULT+rotation], [RB_INNER_ID, RB_INNER_DEFAULT+rotation], [RF_INNER_ID, RF_INNER_DEFAULT+rotation], [LM_INNER_ID, LM_INNER_DEFAULT-rotation]]) # Lifting second set of legs and rotation
    time.sleep(pause)
    print(f"\n[t = {time.perf_counter() - t0:.1f} s]")
    print(s.getDistance())

    board.bus_servo_set_position(duration, [[RM_MIDDLE_ID, RM_MIDDLE_DEFAULT], [LB_MIDDLE_ID, LB_MIDDLE_DEFAULT], [LF_MIDDLE_ID, LF_MIDDLE_DEFAULT]]) # Putting down
    time.sleep(pause)
    print(f"\n[t = {time.perf_counter() - t0:.1f} s]")
    print(s.getDistance())

def tripod_with_correction(correction, dur=0.3, pu=0.3, lif=100, rot=105):

    duration = dur
    pause = pu
    lift = lif
    rotation = rot

    t0 = time.perf_counter()

    # First tripod group with correction
    board.bus_servo_set_position(duration, [
        [RB_MIDDLE_ID, RB_MIDDLE_DEFAULT-lift],
        [RF_MIDDLE_ID, RF_MIDDLE_DEFAULT-lift],
        [LM_MIDDLE_ID, LM_MIDDLE_DEFAULT+lift],
        [RB_INNER_ID, RB_INNER_DEFAULT-rotation+7 - correction],
        [RF_INNER_ID, RF_INNER_DEFAULT-rotation+7 - correction],
        [LM_INNER_ID, LM_INNER_DEFAULT+rotation+7 - correction],
        [RM_INNER_ID, RM_INNER_DEFAULT+rotation],
        [LB_INNER_ID, LB_INNER_DEFAULT-rotation],
        [LF_INNER_ID, LF_INNER_DEFAULT-rotation]
    ])
    time.sleep(pause)
    print(f"\n[t = {time.perf_counter() - t0:.1f} s] correction={correction}")

    board.bus_servo_set_position(duration, [
        [RB_MIDDLE_ID, RB_MIDDLE_DEFAULT],
        [RF_MIDDLE_ID, RF_MIDDLE_DEFAULT],
        [LM_MIDDLE_ID, LM_MIDDLE_DEFAULT]
    ])
    time.sleep(pause)

    # Second tripod group
    board.bus_servo_set_position(duration, [
        [RM_MIDDLE_ID, RM_MIDDLE_DEFAULT-lift],
        [LB_MIDDLE_ID, LB_MIDDLE_DEFAULT+lift],
        [LF_MIDDLE_ID, LF_MIDDLE_DEFAULT+lift],
        [RM_INNER_ID, RM_INNER_DEFAULT-rotation],
        [LB_INNER_ID, LB_INNER_DEFAULT+rotation],
        [LF_INNER_ID, LF_INNER_DEFAULT+rotation],
        [RB_INNER_ID, RB_INNER_DEFAULT+rotation],
        [RF_INNER_ID, RF_INNER_DEFAULT+rotation],
        [LM_INNER_ID, LM_INNER_DEFAULT-rotation]
    ])
    time.sleep(pause)

    board.bus_servo_set_position(duration, [
        [RM_MIDDLE_ID, RM_MIDDLE_DEFAULT],
        [LB_MIDDLE_ID, LB_MIDDLE_DEFAULT],
        [LF_MIDDLE_ID, LF_MIDDLE_DEFAULT]
    ])
    time.sleep(pause)

def platform_left(dur):
    board.bus_servo_set_position(dur, [[P_ID, P_LEFT]])

def platform_right(dur):
    board.bus_servo_set_position(dur, [[P_ID, P_RIGHT]])

def platform_default(dur):
    board.bus_servo_set_position(dur, [[P_ID, P_DEFAULT]])

def turn_left(dur, pu, rot,lif):
    duration = dur
    pause = pu
    rotation = rot
    lift = lif

    board.bus_servo_set_position(duration,[[2, RB_MIDDLE_DEFAULT-lift], [8, RF_MIDDLE_DEFAULT-lift], [14, LM_MIDDLE_DEFAULT+lift], [1, RB_INNER_DEFAULT+rotation], [7, RF_INNER_DEFAULT+rotation], [13, LM_INNER_DEFAULT+rotation]])
    time.sleep(pause)
    board.bus_servo_set_position(duration, [[2,RB_MIDDLE_DEFAULT], [8, RF_MIDDLE_DEFAULT], [14, LM_MIDDLE_DEFAULT]])
    time.sleep(pause)
    board.bus_servo_set_position(duration,[[5, RM_MIDDLE_DEFAULT-lift], [11, LB_MIDDLE_DEFAULT+lift], [17,LF_MIDDLE_DEFAULT+lift], [1,RB_INNER_DEFAULT], [7, RF_INNER_DEFAULT], [13, LM_INNER_DEFAULT]])
    time.sleep(pause)
    board.bus_servo_set_position(duration, [[5, RM_MIDDLE_DEFAULT], [11,LB_MIDDLE_DEFAULT], [17, LF_MIDDLE_DEFAULT]])


def turn_right(dur, pu, rot,lif):
    duration = dur
    pause = pu
    rotation = rot
    lift = lif

    board.bus_servo_set_position(duration,[[5,RM_MIDDLE_DEFAULT-lift], [11, LB_MIDDLE_DEFAULT+lift], [17, LF_MIDDLE_DEFAULT+lift], [4, RM_INNER_DEFAULT-rotation], [10, LB_INNER_DEFAULT-rotation], [16, LF_INNER_DEFAULT-rotation]])
    time.sleep(pause)
    board.bus_servo_set_position(duration,[[5, RM_MIDDLE_DEFAULT], [11,LB_MIDDLE_DEFAULT], [17,LF_MIDDLE_DEFAULT]])
    time.sleep(pause)
    board.bus_servo_set_position(duration, [[2,RB_MIDDLE_DEFAULT-lift], [8, RF_MIDDLE_DEFAULT-lift], [14, LM_MIDDLE_DEFAULT+lift], [4,RM_INNER_DEFAULT], [10,LB_INNER_DEFAULT], [16,LF_INNER_DEFAULT]])
    time.sleep(pause)
    board.bus_servo_set_position(duration, [[2, RB_MIDDLE_DEFAULT], [8,RF_MIDDLE_DEFAULT], [14,LM_MIDDLE_DEFAULT]])

def turn_left_90():
    for i in range(4):
        turn_left(0.3,0.3,197,100)
        time.sleep(0.3)

def turn_right_90():
    for i in range(4):
        turn_right(0.3,0.3,197,100)
        time.sleep(0.3)

def turn_around_180():
    for i in range(8):
        turn_left(0.3,0.3,197,100)
        time.sleep(0.3)

#test the accuracy of moving one tile, important for adjusting the compounding errors
def move_one_tile(reps=1):
    repetitions = reps * 4
    if repetitions>4:
        for j in range(repetitions):
            rotation = 105 - 2 * (reps-1)
            tripod(rot=rotation)
    else:
        for j in range(repetitions):
            tripod()

#dead reckoning algorithm, needs to update current position and heading after each movement, print current position and heading, maitain in a form of tuple (x,y,heading)
#return number of forward steps taken
def move_with_target(start, goal):
    heading = start[2]
    cur_x = start[0]
    cur_y = start[1]
    dif_NS = goal[0] - cur_x
    dif_EW = goal[1] - cur_y
    print(f"Starting at ({cur_x}, {cur_y}), heading={heading}")

    # North-South movement
    # dif_NS > 0 means travel south, dif_NS < 0 means travel north
    if dif_NS != 0:
        target_dir = DIRECTION.South if dif_NS > 0 else DIRECTION.North
        diff = (target_dir - heading) % 4
        if diff == 1:
            turn_right_90()
            heading = target_dir
        elif diff == 2:
            turn_around_180()
            heading = target_dir
        elif diff == 3:
            turn_left_90()
            heading = target_dir
        move_one_tile(abs(dif_NS))
        cur_x = goal[0]
        print(f"After NS move: ({cur_x}, {cur_y}), heading={heading}")

    # East-West movement
    # dif_EW > 0 means travel east, dif_EW < 0 means travel west
    if dif_EW != 0:
        target_dir = DIRECTION.East if dif_EW > 0 else DIRECTION.West
        diff = (target_dir - heading) % 4
        if diff == 1:
            turn_right_90()
            heading = target_dir
        elif diff == 2:
            turn_around_180()
            heading = target_dir
        elif diff == 3:
            turn_left_90()
            heading = target_dir
        move_one_tile(abs(dif_EW))
        cur_y = goal[1]
        print(f"After EW move: ({cur_x}, {cur_y}), heading={heading}")

    # Align goal heading
    if heading != goal[2]:
        diff = (goal[2] - heading) % 4
        if diff == 1:
            turn_right_90()
        elif diff == 2:
            turn_around_180()
        elif diff == 3:
            turn_left_90()
    print(f"Final position: ({cur_x}, {cur_y}), heading={goal[2]}")

    return abs(dif_NS) + abs(dif_EW)

#Ask user for start and goal position and then execute the movement, assume no obstacle in the path.
def move_to_target_with_input():
    start_x = int(input("Enter the starting x coordinate: "))
    start_y = int(input("Enter the starting y coordinate: "))
    start_heading = int(input("Enter the start heading: "))
    goal_x = int(input("Enter the goal x coordinate: "))
    goal_y = int(input("Enter the goal y coordinate: "))
    goal_heading = int(input("Enter the goal heading: "))
    start = (start_x, start_y, start_heading)  
    goal = (goal_x, goal_y, goal_heading)    
    move_with_target(start, goal)

#Return the cost map after wavefront propagation
def wavefront_propagation(given_map, goal_x, goal_y):
    # Clear the cost map first
    given_map.clearCostMap()

    # Validate goal position
    if goal_x < 0 or goal_x >= given_map.costmap_size_row or goal_y < 0 or goal_y >= given_map.costmap_size_col:
        print(f"ERROR: Goal position ({goal_x}, {goal_y}) is out of bounds")
        return None

    #set the goal position's cost to 2, since 1 and 0 is usually indicated as something else
    if given_map.setCost(goal_x, goal_y, 2) == -1:
        print(f"ERROR: Failed to set goal cost at ({goal_x}, {goal_y})")
        return None

    # Wavefront propagation using iterative approach
    # Continue until no more changes occur
    changed = True
    while changed:
        changed = False
        # Scan through all cells
        for i in range(given_map.costmap_size_row):
            for j in range(given_map.costmap_size_col):
                current_cost = given_map.getCost(i, j)
                if current_cost == -1:
                    continue
                # If cell has been assigned a value (> 2, since the goal position is 2, so the cost value starts at 2 and increasig), try to propagate
                if current_cost >= 2:
                    # Increment cost value of neighbor to the current cost cell
                    for direction in [DIRECTION.North, DIRECTION.South, DIRECTION.East, DIRECTION.West]:
                        # Skip directions that would go out of bounds
                        if direction == DIRECTION.North and i == 0:
                            continue
                        if direction == DIRECTION.South and i >= given_map.costmap_size_row - 1:
                            continue
                        if direction == DIRECTION.West and j == 0:
                            continue
                        if direction == DIRECTION.East and j >= given_map.costmap_size_col - 1:
                            continue

                        # Check wall existence in different directions, if no:
                        obstacle = given_map.getNeighborObstacle(i, j, direction)
                        if obstacle == 0:  # No wall
                            neighbor_cost = given_map.getNeighborCost(i, j, direction)
                            # If neighbor is unvisited (0) or has higher cost, update it
                            if neighbor_cost == 0 or neighbor_cost > current_cost + 1:
                                given_map.setNeighborCost(i, j, direction, current_cost + 1)
                                changed = True
                
    #for not possible path, set a extremely high value so that the robot won't be interfere
    for i in range(given_map.costmap_size_row):
        for j in range(given_map.costmap_size_col):
            if given_map.getCost(i,j) == 0:
                given_map.setCost(i,j,10000)
    return given_map.costMap

#Traverse the cost map from the current position, and find the optimal direction to travel for the next step,
# which by finding the lowest non obstacle cost cell near the position.
def next_step(given_map, cur_x, cur_y):
    cur_cost = given_map.getCost(cur_x, cur_y)
    cur_min_cost = cur_cost
    best_direction = None
    max_row = given_map.costmap_size_row - 1
    max_col = given_map.costmap_size_col - 1

    for direction in [DIRECTION.North, DIRECTION.South, DIRECTION.East, DIRECTION.West]:
        # Skip directions that would go out of bounds
        if direction == DIRECTION.North and cur_x == 0:
            continue
        if direction == DIRECTION.South and cur_x >= max_row:
            continue
        if direction == DIRECTION.West and cur_y == 0:
            continue
        if direction == DIRECTION.East and cur_y >= max_col:
            continue

        #must be non obstacle
        obstacle = given_map.getNeighborObstacle(cur_x, cur_y, direction)
        if obstacle == -1:  # Out of bounds error from getNeighborObstacle
            continue
        if obstacle == 0:  # No wall
            neighbor_cost = given_map.getNeighborCost(cur_x, cur_y, direction)
            #must be smaller than the current cost
            if neighbor_cost < cur_cost and neighbor_cost < cur_min_cost:
                cur_min_cost = neighbor_cost
                best_direction = direction
    return best_direction

# Determine which side wall to use for referecing by the controller so that it can keep the robot away from serious lateral drift that caused hitting to the wall
# Checks both current position and next position to ensure reliable wall reference, return the direction of the reliable wall, so the sensor platform can rotate accordingly
def get_reliable_wall_side(given_map, cur_x, cur_y, next_direction):

    # Calculate next position
    next_x, next_y = cur_x, cur_y
    if next_direction == DIRECTION.North:
        next_x = cur_x - 1
    elif next_direction == DIRECTION.South:
        next_x = cur_x + 1
    elif next_direction == DIRECTION.East:
        next_y = cur_y + 1
    elif next_direction == DIRECTION.West:
        next_y = cur_y - 1

    max_col = given_map.costmap_size_col - 1

    # Check if West wall exists at both current and next position
    west_wall_current = False
    west_wall_next = False
    if cur_y > 0:
        west_obs = given_map.getNeighborObstacle(cur_x, cur_y, DIRECTION.West)
        west_wall_current = (west_obs == 1)  # 1 means wall exists
    if next_y > 0:
        west_obs = given_map.getNeighborObstacle(next_x, next_y, DIRECTION.West)
        west_wall_next = (west_obs == 1)

    # Check if East wall exists at both current and next position
    east_wall_current = False
    east_wall_next = False
    if cur_y < max_col:
        east_obs = given_map.getNeighborObstacle(cur_x, cur_y, DIRECTION.East)
        east_wall_current = (east_obs == 1)
    if next_y < max_col:
        east_obs = given_map.getNeighborObstacle(next_x, next_y, DIRECTION.East)
        east_wall_next = (east_obs == 1)

    # Prefer side with walls at both positions
    if west_wall_current and west_wall_next:
        return 'left'
    if east_wall_current and east_wall_next:
        return 'right'

    # If no side has walls at both positions, try single position
    if west_wall_current or west_wall_next:
        return 'left'
    if east_wall_current or east_wall_next:
        return 'right'

    return None  # No reliable wall detected

def centering_pid_update(sonar_distance, previous_error):

    error = sonar_distance - CORRIDOR_HALF_WIDTH

    # Proportional term
    p_term = CENTERING_KP * error


    # Derivative term
    derivative = error - previous_error
    d_term = CENTERING_KD * derivative

    correction = int(p_term + d_term)

    # Limit correction
    correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))

    return correction, error

def move_one_tile_with_centering(given_map, cur_x, cur_y, next_direction, pid_prev_error, current_correction, current_wall_side, platform_positioned):

    # Apply the correction that was computed from PREVIOUS step's sonar reading
    correction = current_correction

    # Determine which side wall to use for NEXT step
    wall_side = get_reliable_wall_side(given_map, cur_x, cur_y, next_direction)

    new_correction = current_correction  # Default: keep same correction if no wall
    new_error = pid_prev_error
    new_platform_positioned = platform_positioned
    new_wall_side = current_wall_side

    if wall_side:
        # Check if we need to rotate platform (only when wall side changes)
        if wall_side != current_wall_side:
            # First return to center if previously positioned
            if platform_positioned:
                platform_default(0.5)
                time.sleep(0.5)
                new_platform_positioned = False

            # Rotate platform to read from appropriate side
            if wall_side == 'left':
                platform_left(0.5)
            else:
                platform_right(0.5)
            time.sleep(0.6)
            new_platform_positioned = True
            new_wall_side = wall_side
            print(f"  Platform rotated to: {wall_side}")

        # Take sonar reading for NEXT step's correction
        sonar_distance = s.getDistance()
        print(f"  Wall side: {wall_side}, Sonar: {sonar_distance}mm")

        # Compute PID correction for NEXT step
        new_correction, new_error = centering_pid_update(sonar_distance, pid_prev_error)
        print(f"  PID: next_correction={new_correction}, error={new_error}")
    else:
        print(f"  No reliable wall detected, using dead reckoning")
        # No wall, no correction for next step
        new_correction = 0
        new_error = pid_prev_error  # Keep error state consistent

    # Execute the tripod step with correction computed from previous step
    tripod_with_correction(correction)

    return new_error, new_correction, new_wall_side, new_platform_positioned

#execute and return the path, ask user to input start and goal position
def path_generating(given_map):

    max_row = given_map.costmap_size_row - 1
    max_col = given_map.costmap_size_col - 1
    print(f"Map dimensions: x (row) in [0, {max_row}], y (col) in [0, {max_col}]")

    #create variables to store current state and keep updating until the robot walk to the destination
    start_x = int(input("Enter the starting x coordinate: "))
    start_y = int(input("Enter the starting y coordinate: "))
    start_heading = int(input("Enter the start heading: "))
    goal_x = int(input("Enter the goal x coordinate: "))
    goal_y = int(input("Enter the goal y coordinate: "))
    goal_heading = int(input("Enter the goal heading: "))

    # Validate coordinates
    if not (0 <= start_x <= max_row and 0 <= start_y <= max_col):
        print(f"ERROR: Start position ({start_x}, {start_y}) is out of bounds!")
        return None
    if not (0 <= goal_x <= max_row and 0 <= goal_y <= max_col):
        print(f"ERROR: Goal position ({goal_x}, {goal_y}) is out of bounds!")
        return None

    #First make sure the cost map is correctly set up
    cost_map = wavefront_propagation(given_map, goal_x, goal_y)
    if cost_map is None:
        print("ERROR: Failed to generate cost map")
        return None
    given_map.costMap = cost_map

    cur_x = start_x
    cur_y = start_y
    cur_heading = start_heading
    #store the direction of travel one tile for each step
    path = []

    # Initialize PID state and platform tracking
    pid_prev_error = 0
    current_correction = 0  # Correction to apply (computed from previous step)
    current_wall_side = None
    platform_positioned = False

    # Take initial sonar reading before first step
    initial_wall_side = get_reliable_wall_side(given_map, cur_x, cur_y, DIRECTION.South)
    if initial_wall_side:
        if initial_wall_side == 'left':
            platform_left(0.5)
        else:
            platform_right(0.5)
        time.sleep(0.6)
        platform_positioned = True
        current_wall_side = initial_wall_side
        initial_distance = s.getDistance()
        print(f"Initial: Wall side: {initial_wall_side}, Sonar: {initial_distance}mm")
        # Compute initial correction for first step
        current_correction, pid_prev_error = centering_pid_update(initial_distance, pid_prev_error)
        print(f"Initial PID: correction={current_correction}, error={pid_prev_error}")

    while cur_x != goal_x or cur_y != goal_y:
        #find the next direction to travel for one tile
        next_direction = next_step(given_map, cur_x, cur_y)
        print(f"Current position: ({cur_x}, {cur_y}), heading={cur_heading}, next direction={next_direction}")
        if next_direction is None:
            print(f"Error: No valid path found - stuck at position ({cur_x}, {cur_y})")
            break
        path.append(next_direction)
        #if heading is the same, then no need to update heading but only positilon
        if next_direction == cur_heading:
            pid_prev_error, current_correction, current_wall_side, platform_positioned = move_one_tile_with_centering(given_map, cur_x, cur_y, next_direction, pid_prev_error, current_correction, current_wall_side, platform_positioned)
            #update current position
            if next_direction == DIRECTION.South:
                cur_x+=1
            elif next_direction == DIRECTION.North:
                cur_x-=1
            elif next_direction == DIRECTION.East:
                cur_y+=1
            elif next_direction == DIRECTION.West:
                cur_y-=1
            print(f"Moved to position: ({cur_x}, {cur_y}), heading={cur_heading}")
        # if direction is opposite, update position and update current heading
        elif abs(next_direction - cur_heading) == 2:
            turn_around_180()
            pid_prev_error, current_correction, current_wall_side, platform_positioned = move_one_tile_with_centering(given_map, cur_x, cur_y, next_direction, pid_prev_error, current_correction, current_wall_side, platform_positioned)
            if next_direction == DIRECTION.South:
                cur_x+=1
                cur_heading = DIRECTION.South
            elif next_direction == DIRECTION.North:
                cur_x-=1
                cur_heading = DIRECTION.North
            elif next_direction == DIRECTION.East:
                cur_y+=1
                cur_heading = DIRECTION.East
            elif next_direction == DIRECTION.West:
                cur_y-=1
                cur_heading = DIRECTION.West
            print(f"Moved to position: ({cur_x}, {cur_y}), heading={cur_heading}")
        #turn left and only update current state of y position and heading
        elif cur_heading == DIRECTION.South and next_direction == DIRECTION.East:
            turn_left_90()
            pid_prev_error, current_correction, current_wall_side, platform_positioned = move_one_tile_with_centering(given_map, cur_x, cur_y, next_direction, pid_prev_error, current_correction, current_wall_side, platform_positioned)
            cur_y+=1
            cur_heading = DIRECTION.East
            print(f"Moved to position: ({cur_x}, {cur_y}), heading={cur_heading}")
        elif cur_heading == DIRECTION.South and next_direction == DIRECTION.West:
            turn_right_90()
            pid_prev_error, current_correction, current_wall_side, platform_positioned = move_one_tile_with_centering(given_map, cur_x, cur_y, next_direction, pid_prev_error, current_correction, current_wall_side, platform_positioned)
            cur_y-=1
            cur_heading = DIRECTION.West
            print(f"Moved to position: ({cur_x}, {cur_y}), heading={cur_heading}")
        elif cur_heading == DIRECTION.North and next_direction == DIRECTION.East:
            turn_right_90()
            pid_prev_error, current_correction, current_wall_side, platform_positioned = move_one_tile_with_centering(given_map, cur_x, cur_y, next_direction, pid_prev_error, current_correction, current_wall_side, platform_positioned)
            cur_y+=1
            cur_heading = DIRECTION.East
            print(f"Moved to position: ({cur_x}, {cur_y}), heading={cur_heading}")
        elif cur_heading == DIRECTION.North and next_direction == DIRECTION.West:
            turn_left_90()
            pid_prev_error, current_correction, current_wall_side, platform_positioned = move_one_tile_with_centering(given_map, cur_x, cur_y, next_direction, pid_prev_error, current_correction, current_wall_side, platform_positioned)
            cur_y-=1
            cur_heading = DIRECTION.West
            print(f"Moved to position: ({cur_x}, {cur_y}), heading={cur_heading}")
        elif cur_heading == DIRECTION.East and next_direction == DIRECTION.North:
            turn_left_90()
            pid_prev_error, current_correction, current_wall_side, platform_positioned = move_one_tile_with_centering(given_map, cur_x, cur_y, next_direction, pid_prev_error, current_correction, current_wall_side, platform_positioned)
            cur_x-=1
            cur_heading = DIRECTION.North
            print(f"Moved to position: ({cur_x}, {cur_y}), heading={cur_heading}")
        elif cur_heading == DIRECTION.East and next_direction == DIRECTION.South:
            turn_right_90()
            pid_prev_error, current_correction, current_wall_side, platform_positioned = move_one_tile_with_centering(given_map, cur_x, cur_y, next_direction, pid_prev_error, current_correction, current_wall_side, platform_positioned)
            cur_x+=1
            cur_heading = DIRECTION.South
            print(f"Moved to position: ({cur_x}, {cur_y}), heading={cur_heading}")
        elif cur_heading == DIRECTION.West and next_direction == DIRECTION.North:
            turn_right_90()
            pid_prev_error, current_correction, current_wall_side, platform_positioned = move_one_tile_with_centering(given_map, cur_x, cur_y, next_direction, pid_prev_error, current_correction, current_wall_side, platform_positioned)
            cur_x-=1
            cur_heading = DIRECTION.North
            print(f"Moved to position: ({cur_x}, {cur_y}), heading={cur_heading}")
        elif cur_heading == DIRECTION.West and next_direction == DIRECTION.South:
            turn_left_90()
            pid_prev_error, current_correction, current_wall_side, platform_positioned = move_one_tile_with_centering(given_map, cur_x, cur_y, next_direction, pid_prev_error, current_correction, current_wall_side, platform_positioned)
            cur_x+=1
            cur_heading = DIRECTION.South
            print(f"Moved to position: ({cur_x}, {cur_y}), heading={cur_heading}")
    #finally after arriving the goal position, adjust the goal heading
    if goal_heading == cur_heading:
        pass  # Already facing the correct direction
    elif abs(goal_heading - cur_heading) == 2:
        turn_around_180()
    elif goal_heading == DIRECTION.South and cur_heading == DIRECTION.East:
        turn_left_90()
    elif goal_heading == DIRECTION.South and cur_heading == DIRECTION.West:
        turn_right_90()
    elif goal_heading == DIRECTION.North and cur_heading == DIRECTION.East:
        turn_right_90()
    elif goal_heading == DIRECTION.North and cur_heading == DIRECTION.West:
        turn_left_90()
    elif goal_heading == DIRECTION.East and cur_heading == DIRECTION.North:
        turn_left_90()
    elif goal_heading == DIRECTION.East and cur_heading == DIRECTION.South:
        turn_right_90()
    elif goal_heading == DIRECTION.West and cur_heading == DIRECTION.North:
        turn_right_90()
    elif goal_heading == DIRECTION.West and cur_heading == DIRECTION.South:
        turn_left_90()

    

#find the optimal path given the map and return the number of steps to the goal
def bfs_shortest_path(given_map, start_x, start_y, goal_x, goal_y):
    max_row = given_map.costmap_size_row - 1
    max_col = given_map.costmap_size_col - 1

    queue = deque([(start_x, start_y)])
    visited = set([(start_x, start_y)])
    parent = {(start_x, start_y): None}

    while queue:
        x, y = queue.popleft()
        if (x, y) == (goal_x, goal_y):
            path = []
            while (x, y) is not None:
                path.append((x, y))
                x, y = parent[(x, y)]
            return len(path) - 1

        for direction in [DIRECTION.North, DIRECTION.South, DIRECTION.East, DIRECTION.West]:
            # Skip directions that would go out of bounds
            if direction == DIRECTION.North and x == 0:
                continue
            if direction == DIRECTION.South and x >= max_row:
                continue
            if direction == DIRECTION.West and y == 0:
                continue
            if direction == DIRECTION.East and y >= max_col:
                continue

            obstacle = given_map.getNeighborObstacle(x, y, direction)
            if obstacle == 0:  # No wall
                if direction == DIRECTION.North:
                    neighbor = (x - 1, y)
                elif direction == DIRECTION.South:
                    neighbor = (x + 1, y)
                elif direction == DIRECTION.East:
                    neighbor = (x, y + 1)
                elif direction == DIRECTION.West:
                    neighbor = (x, y - 1)

                if neighbor not in visited:
                    visited.add(neighbor)
                    parent[neighbor] = (x, y)
                    queue.append(neighbor)

    return None     

  
                
def manhatten_distance(start_x, start_y, goal_x, goal_y):
    return abs(start_x - goal_x) + abs(start_y - goal_y)
        


if __name__ == "__main__":
    set_all_default()
    s = sonar.Sonar()
    start_time = time.time()
    #Create the map object
    map1 = mp.CSME301Map()
    map1.printObstacleMap()
    map1.printCostMap()

    # turn_left_90()
    # # move_to_target_with_input()
    path_generating(map1)

    #See the cost map after planning.
    map1.printCostMap()
   


    #Calculate the ratio between manhatten distance to actual localization steps
    # ML_ratio = manhatten_distance(start_x, start_y, goal_x, goal_y) / move_with_target(start,goal)
    # print(f"Manhatten distance to actual steps ratio: {ML_ratio:.2f}")

    #Calculate the ratio between optimal path steps and actual wavefront path steps
    # optimal_steps = bfs_shortest_path(map1, start_x, start_y, goal_x, goal_y)/len(path)
    # print(f"Optimal path steps to wavefront path steps ratio: {optimal_steps:.2f}")
    


    




