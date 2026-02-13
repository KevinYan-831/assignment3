import map as mp
import sys
import time
import signal
import threading  # use threading to run PID control and tripod simultaneously
import ros_robot_controller_sdk as rrc
import sonar
import matplotlib.pyplot as plt
from collections import deque

# x is south north, south is positive. Y is east west, east is positive
# Algorithm follow the cost gradient
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

# Platform's value
P_ID = 21
P_DEFAULT = 500
P_RIGHT = 130
P_LEFT = 870

# North=1, East=2, South=3, West=4

#Reactive control, distance threshold, used to avoid hitting the wall. If exceed threshold, just immediately skip the current step and move on
DISTANCE = 340

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
        # Platform
        [P_ID, P_DEFAULT]
    ])
    time.sleep(1)


def tripod(dur=0.3, pu=0.3, lif=100, rot=105):
    duration = dur
    pause = pu
    lift = lif
    rotation = rot

    t0 = time.perf_counter()

    board.bus_servo_set_position(duration, [
        [RB_MIDDLE_ID, RB_MIDDLE_DEFAULT - lift],
        [RF_MIDDLE_ID, RF_MIDDLE_DEFAULT - lift],
        [LM_MIDDLE_ID, LM_MIDDLE_DEFAULT + lift],
        [RB_INNER_ID, RB_INNER_DEFAULT - rotation],
        [RF_INNER_ID, RF_INNER_DEFAULT - rotation],
        [LM_INNER_ID, LM_INNER_DEFAULT + rotation ],
        [RM_INNER_ID, RM_INNER_DEFAULT + rotation],
        [LB_INNER_ID, LB_INNER_DEFAULT - rotation],
        [LF_INNER_ID, LF_INNER_DEFAULT - rotation]
    ])  # Initial lift of legs and rotation
    time.sleep(pause)
    print(f"\n[t = {time.perf_counter() - t0:.1f} s]")
    print(s.getDistance())

    board.bus_servo_set_position(duration, [
        [RB_MIDDLE_ID, RB_MIDDLE_DEFAULT],
        [RF_MIDDLE_ID, RF_MIDDLE_DEFAULT],
        [LM_MIDDLE_ID, LM_MIDDLE_DEFAULT]
    ])  # Putting legs back down
    time.sleep(pause)
    print(f"\n[t = {time.perf_counter() - t0:.1f} s]")
    print(s.getDistance())

    board.bus_servo_set_position(duration, [
        [RM_MIDDLE_ID, RM_MIDDLE_DEFAULT - lift],
        [LB_MIDDLE_ID, LB_MIDDLE_DEFAULT + lift],
        [LF_MIDDLE_ID, LF_MIDDLE_DEFAULT + lift],
        [RM_INNER_ID, RM_INNER_DEFAULT - rotation],
        [LB_INNER_ID, LB_INNER_DEFAULT + rotation],
        [LF_INNER_ID, LF_INNER_DEFAULT + rotation],
        [RB_INNER_ID, RB_INNER_DEFAULT + rotation],
        [RF_INNER_ID, RF_INNER_DEFAULT + rotation],
        [LM_INNER_ID, LM_INNER_DEFAULT - rotation]
    ])  # Lifting second set of legs and rotation
    time.sleep(pause)
    print(f"\n[t = {time.perf_counter() - t0:.1f} s]")
    print(s.getDistance())

    board.bus_servo_set_position(duration, [
        [RM_MIDDLE_ID, RM_MIDDLE_DEFAULT],
        [LB_MIDDLE_ID, LB_MIDDLE_DEFAULT],
        [LF_MIDDLE_ID, LF_MIDDLE_DEFAULT]
    ])  # Putting down
    time.sleep(pause)
    print(f"\n[t = {time.perf_counter() - t0:.1f} s]")
    print(s.getDistance())


def turn_left(dur, pu, rot, lif):
    duration = dur
    pause = pu
    rotation = rot
    lift = lif

    board.bus_servo_set_position(duration, [
        [2, RB_MIDDLE_DEFAULT - lift],
        [8, RF_MIDDLE_DEFAULT - lift],
        [14, LM_MIDDLE_DEFAULT + lift],
        [1, RB_INNER_DEFAULT + rotation],
        [7, RF_INNER_DEFAULT + rotation],
        [13, LM_INNER_DEFAULT + rotation]
    ])
    time.sleep(pause)

    board.bus_servo_set_position(duration, [
        [2, RB_MIDDLE_DEFAULT],
        [8, RF_MIDDLE_DEFAULT],
        [14, LM_MIDDLE_DEFAULT]
    ])
    time.sleep(pause)

    board.bus_servo_set_position(duration, [
        [5, RM_MIDDLE_DEFAULT - lift],
        [11, LB_MIDDLE_DEFAULT + lift],
        [17, LF_MIDDLE_DEFAULT + lift],
        [1, RB_INNER_DEFAULT],
        [7, RF_INNER_DEFAULT],
        [13, LM_INNER_DEFAULT]
    ])
    time.sleep(pause)

    board.bus_servo_set_position(duration, [
        [5, RM_MIDDLE_DEFAULT],
        [11, LB_MIDDLE_DEFAULT],
        [17, LF_MIDDLE_DEFAULT]
    ])


def turn_right(dur, pu, rot, lif):
    duration = dur
    pause = pu
    rotation = rot
    lift = lif

    board.bus_servo_set_position(duration, [
        [5, RM_MIDDLE_DEFAULT - lift],
        [11, LB_MIDDLE_DEFAULT + lift],
        [17, LF_MIDDLE_DEFAULT + lift],
        [4, RM_INNER_DEFAULT - rotation],
        [10, LB_INNER_DEFAULT - rotation],
        [16, LF_INNER_DEFAULT - rotation]
    ])
    time.sleep(pause)

    board.bus_servo_set_position(duration, [
        [5, RM_MIDDLE_DEFAULT],
        [11, LB_MIDDLE_DEFAULT],
        [17, LF_MIDDLE_DEFAULT]
    ])
    time.sleep(pause)

    board.bus_servo_set_position(duration, [
        [2, RB_MIDDLE_DEFAULT - lift],
        [8, RF_MIDDLE_DEFAULT - lift],
        [14, LM_MIDDLE_DEFAULT + lift],
        [4, RM_INNER_DEFAULT],
        [10, LB_INNER_DEFAULT],
        [16, LF_INNER_DEFAULT]
    ])
    time.sleep(pause)

    board.bus_servo_set_position(duration, [
        [2, RB_MIDDLE_DEFAULT],
        [8, RF_MIDDLE_DEFAULT],
        [14, LM_MIDDLE_DEFAULT]
    ])


def turn_left_90():
    for i in range(4):
        turn_left(0.3, 0.3, 197, 100)
        time.sleep(0.3)


def turn_right_90():
    for i in range(4):
        turn_right(0.3, 0.3, 197, 100)
        time.sleep(0.3)


def turn_around_180():
    for i in range(8):
        turn_left(0.3, 0.3, 197, 100)
        time.sleep(0.3)


# test the accuracy of moving one tile, important for adjusting the compounding
# errors
def move_one_tile(reps=1):
    repetitions = reps * 4
    if repetitions > 4:
        for j in range(repetitions):
            rotation = 105 - 2 * (j - 1)
            tripod(rot=rotation)
            distance = s.getDistance()
            if distance < DISTANCE:
                break
            
    else:
        for j in range(repetitions):
            tripod()
            distance = s.getDistance()
            if distance < DISTANCE:
                break


# dead reckoning algorithm, needs to update current position and heading after each
# movement, print current position and heading, maitain in a form of tuple
# (x,y,heading)
# return number of forward steps taken
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


# Ask user for start and goal position and then execute the movement, assume no
# obstacle in the path.
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


# Return the cost map after wavefront propagation
def wavefront_propagation(given_map, goal_x, goal_y):
    # Clear the cost map first
    given_map.clearCostMap()

    # Validate goal position
    if (goal_x < 0 or goal_x >= given_map.costmap_size_row or
            goal_y < 0 or goal_y >= given_map.costmap_size_col):
        print(f"ERROR: Goal position ({goal_x}, {goal_y}) is out of bounds")
        return None

    # set the goal position's cost to 2, since 1 and 0 is usually indicated as
    # something else
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

                # If cell has been assigned a value (> 2, since the goal position
                # is 2, so the cost value starts at 2 and increasing), try to
                # propagate
                if current_cost >= 2:
                    # Increment cost value of neighbor to the current cost cell
                    for direction in [DIRECTION.North, DIRECTION.South,
                                      DIRECTION.East, DIRECTION.West]:
                        # Skip directions that would go out of bounds
                        if direction == DIRECTION.North and i == 0:
                            continue
                        if (direction == DIRECTION.South and
                                i >= given_map.costmap_size_row - 1):
                            continue
                        if direction == DIRECTION.West and j == 0:
                            continue
                        if (direction == DIRECTION.East and
                                j >= given_map.costmap_size_col - 1):
                            continue

                        # Check wall existence in different directions, if no:
                        obstacle = given_map.getNeighborObstacle(i, j, direction)
                        if obstacle == 0:  # No wall
                            neighbor_cost = given_map.getNeighborCost(
                                i, j, direction)
                            # If neighbor is unvisited (0) or has higher cost,
                            # update it
                            if (neighbor_cost == 0 or
                                    neighbor_cost > current_cost + 1):
                                given_map.setNeighborCost(
                                    i, j, direction, current_cost + 1)
                                changed = True

    # for not possible path, set a extremely high value so that the robot won't be
    # interfere
    for i in range(given_map.costmap_size_row):
        for j in range(given_map.costmap_size_col):
            if given_map.getCost(i, j) == 0:
                given_map.setCost(i, j, 10000)

    return given_map.costMap


# Traverse the cost map from the current position, and find the optimal direction
# to travel for the next step, which by finding the lowest non obstacle cost cell
# near the position.
def next_step(given_map, cur_x, cur_y):
    cur_cost = given_map.getCost(cur_x, cur_y)
    cur_min_cost = cur_cost
    best_direction = None
    max_row = given_map.costmap_size_row - 1
    max_col = given_map.costmap_size_col - 1

    for direction in [DIRECTION.North, DIRECTION.South, DIRECTION.East,
                      DIRECTION.West]:
        # Skip directions that would go out of bounds
        if direction == DIRECTION.North and cur_x == 0:
            continue
        if direction == DIRECTION.South and cur_x >= max_row:
            continue
        if direction == DIRECTION.West and cur_y == 0:
            continue
        if direction == DIRECTION.East and cur_y >= max_col:
            continue

        # must be non obstacle
        obstacle = given_map.getNeighborObstacle(cur_x, cur_y, direction)
        if obstacle == -1:  # Out of bounds error from getNeighborObstacle
            continue
        if obstacle == 0:  # No wall
            neighbor_cost = given_map.getNeighborCost(cur_x, cur_y, direction)
            # must be smaller than the current cost
            if neighbor_cost < cur_cost and neighbor_cost < cur_min_cost:
                cur_min_cost = neighbor_cost
                best_direction = direction

    return best_direction

#print the path
# execute and return the path, ask user to input start and goal position
def print_path_generating(given_map):
    max_row = given_map.costmap_size_row - 1
    max_col = given_map.costmap_size_col - 1
    print(f"Map dimensions: x (row) in [0, {max_row}], y (col) in [0, {max_col}]")

    # create variables to store current state and keep updating until the robot
    # walk to the destination
    start_x = int(input("Enter the starting x coordinate: "))
    start_y = int(input("Enter the starting y coordinate: "))
    start_heading = int(input("Enter the start heading: "))
    goal_x = int(input("Enter the goal x coordinate: "))
    goal_y = int(input("Enter the goal y coordinate: "))
    

    # Validate coordinates
    if not (0 <= start_x <= max_row and 0 <= start_y <= max_col):
        print(f"ERROR: Start position ({start_x}, {start_y}) is out of bounds!")
        return None
    if not (0 <= goal_x <= max_row and 0 <= goal_y <= max_col):
        print(f"ERROR: Goal position ({goal_x}, {goal_y}) is out of bounds!")
        return None

    # First make sure the cost map is correctly set up
    cost_map = wavefront_propagation(given_map, goal_x, goal_y)
    if cost_map is None:
        print("ERROR: Failed to generate cost map")
        return None

    given_map.costMap = cost_map

    cur_x = start_x
    cur_y = start_y
    cur_heading = start_heading

    # store the direction of travel one tile for each step
    path = []

    while cur_x != goal_x or cur_y != goal_y:
        # find the next direction to travel for one tile
        next_direction = next_step(given_map, cur_x, cur_y)
        print(f"Current position: ({cur_x}, {cur_y}), heading={cur_heading}, "
              f"next direction={next_direction}")

        if next_direction is None:
            print(f"Error: No valid path found - stuck at position "
                  f"({cur_x}, {cur_y})")
            break

        path.append(next_direction)

        # if heading is the same, then no need to update heading but only position
        if next_direction == cur_heading:
            
            # update current position
            if next_direction == DIRECTION.South:
                cur_x += 1
            elif next_direction == DIRECTION.North:
                cur_x -= 1
            elif next_direction == DIRECTION.East:
                cur_y += 1
            elif next_direction == DIRECTION.West:
                cur_y -= 1

        # if direction is opposite, update position and update current heading
        elif abs(next_direction - cur_heading) == 2:
            
            if next_direction == DIRECTION.South:
                cur_x += 1
                cur_heading = DIRECTION.South
            elif next_direction == DIRECTION.North:
                cur_x -= 1
                cur_heading = DIRECTION.North
            elif next_direction == DIRECTION.East:
                cur_y += 1
                cur_heading = DIRECTION.East
            elif next_direction == DIRECTION.West:
                cur_y -= 1
                cur_heading = DIRECTION.West

        # turn left and only update current state of y position and heading
        elif (cur_heading == DIRECTION.South and
              next_direction == DIRECTION.East):
            
            cur_y += 1
            cur_heading = DIRECTION.East


        elif (cur_heading == DIRECTION.South and
              next_direction == DIRECTION.West):
            
            cur_y -= 1
            cur_heading = DIRECTION.West


        elif (cur_heading == DIRECTION.North and
              next_direction == DIRECTION.East):
            
            cur_y += 1
            cur_heading = DIRECTION.East


        elif (cur_heading == DIRECTION.North and
              next_direction == DIRECTION.West):
            
            cur_y -= 1
            cur_heading = DIRECTION.West


        elif (cur_heading == DIRECTION.East and
              next_direction == DIRECTION.North):
            
            cur_x -= 1
            cur_heading = DIRECTION.North


        elif (cur_heading == DIRECTION.East and
              next_direction == DIRECTION.South):
            
            cur_x += 1
            cur_heading = DIRECTION.South


        elif (cur_heading == DIRECTION.West and
              next_direction == DIRECTION.North):
            
            cur_x -= 1
            cur_heading = DIRECTION.North


        elif (cur_heading == DIRECTION.West and
              next_direction == DIRECTION.South):
            
            cur_x += 1
            cur_heading = DIRECTION.South
    return path


# execute and return the path, ask user to input start and goal position
def path_generating(given_map):
    max_row = given_map.costmap_size_row - 1
    max_col = given_map.costmap_size_col - 1
    print(f"Map dimensions: x (row) in [0, {max_row}], y (col) in [0, {max_col}]")

    # create variables to store current state and keep updating until the robot
    # walk to the destination
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

    # First make sure the cost map is correctly set up
    cost_map = wavefront_propagation(given_map, goal_x, goal_y)
    if cost_map is None:
        print("ERROR: Failed to generate cost map")
        return None

    given_map.costMap = cost_map

    cur_x = start_x
    cur_y = start_y
    cur_heading = start_heading

    # store the direction of travel one tile for each step
    path = []

    while cur_x != goal_x or cur_y != goal_y:
        # find the next direction to travel for one tile
        next_direction = next_step(given_map, cur_x, cur_y)
        print(f"Current position: ({cur_x}, {cur_y}), heading={cur_heading}, "
              f"next direction={next_direction}")

        if next_direction is None:
            print(f"Error: No valid path found - stuck at position "
                  f"({cur_x}, {cur_y})")
            break

        path.append(next_direction)

        # if heading is the same, then no need to update heading but only position
        if next_direction == cur_heading:
            move_one_tile()
            # update current position
            if next_direction == DIRECTION.South:
                cur_x += 1
            elif next_direction == DIRECTION.North:
                cur_x -= 1
            elif next_direction == DIRECTION.East:
                cur_y += 1
            elif next_direction == DIRECTION.West:
                cur_y -= 1
            print(f"Moved to position: ({cur_x}, {cur_y}), "
                  f"heading={cur_heading}")

        # if direction is opposite, update position and update current heading
        elif abs(next_direction - cur_heading) == 2:
            turn_around_180()
            move_one_tile()
            if next_direction == DIRECTION.South:
                cur_x += 1
                cur_heading = DIRECTION.South
            elif next_direction == DIRECTION.North:
                cur_x -= 1
                cur_heading = DIRECTION.North
            elif next_direction == DIRECTION.East:
                cur_y += 1
                cur_heading = DIRECTION.East
            elif next_direction == DIRECTION.West:
                cur_y -= 1
                cur_heading = DIRECTION.West
            print(f"Moved to position: ({cur_x}, {cur_y}), "
                  f"heading={cur_heading}")

        # turn left and only update current state of y position and heading
        elif (cur_heading == DIRECTION.South and
              next_direction == DIRECTION.East):
            turn_left_90()
            move_one_tile()
            cur_y += 1
            cur_heading = DIRECTION.East
            print(f"Moved to position: ({cur_x}, {cur_y}), "
                  f"heading={cur_heading}")

        elif (cur_heading == DIRECTION.South and
              next_direction == DIRECTION.West):
            turn_right_90()
            move_one_tile()
            cur_y -= 1
            cur_heading = DIRECTION.West
            print(f"Moved to position: ({cur_x}, {cur_y}), "
                  f"heading={cur_heading}")

        elif (cur_heading == DIRECTION.North and
              next_direction == DIRECTION.East):
            turn_right_90()
            move_one_tile()
            cur_y += 1
            cur_heading = DIRECTION.East
            print(f"Moved to position: ({cur_x}, {cur_y}), "
                  f"heading={cur_heading}")

        elif (cur_heading == DIRECTION.North and
              next_direction == DIRECTION.West):
            turn_left_90()
            move_one_tile()
            cur_y -= 1
            cur_heading = DIRECTION.West
            print(f"Moved to position: ({cur_x}, {cur_y}), "
                  f"heading={cur_heading}")

        elif (cur_heading == DIRECTION.East and
              next_direction == DIRECTION.North):
            turn_left_90()
            move_one_tile()
            cur_x -= 1
            cur_heading = DIRECTION.North
            print(f"Moved to position: ({cur_x}, {cur_y}), "
                  f"heading={cur_heading}")

        elif (cur_heading == DIRECTION.East and
              next_direction == DIRECTION.South):
            turn_right_90()
            move_one_tile()
            cur_x += 1
            cur_heading = DIRECTION.South
            print(f"Moved to position: ({cur_x}, {cur_y}), "
                  f"heading={cur_heading}")

        elif (cur_heading == DIRECTION.West and
              next_direction == DIRECTION.North):
            turn_right_90()
            move_one_tile()
            cur_x -= 1
            cur_heading = DIRECTION.North
            print(f"Moved to position: ({cur_x}, {cur_y}), "
                  f"heading={cur_heading}")

        elif (cur_heading == DIRECTION.West and
              next_direction == DIRECTION.South):
            turn_left_90()
            move_one_tile()
            cur_x += 1
            cur_heading = DIRECTION.South
            print(f"Moved to position: ({cur_x}, {cur_y}), "
                  f"heading={cur_heading}")

    # finally after arriving the goal position, adjust the goal heading
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


# find the optimal path given the map and return the number of steps to the goal
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
            while True:
                path.append((x, y))
                if (x, y) not in parent or parent[(x, y)] is None:
                    break
                x, y = parent[(x, y)]
            return len(path) - 1

        for direction in [DIRECTION.North, DIRECTION.South, DIRECTION.East,
                          DIRECTION.West]:
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

#using dfs approach to naviagte every non-visited cell, assume the robot always start at (0,0,1)
#every step moving, the robot stop to let the sensor detect wall existence in each direction
#should return the constructed map
def mapping(given_map):
    start_x = 0
    start_y = 0
    start_heading = 1

    #keep track of robot current position
    cur_x = 0
    cur_y = 0
    cur_heading = 1

    #initialize the constructed map
    gen_map = 
    #initialize the stack for depth first search


    


if __name__ == "__main__":
    set_all_default()
    s = sonar.Sonar()
    start_time = time.time()

    # Create the map object
    map1 = mp.CSME301Map()
    map1.printObstacleMap()
    map1.printCostMap()

    # turn_right_90()
    # # move_to_target_with_input()
    path = print_path_generating(map1)
    # path_generating(map1)

    # See the cost map after planning.
    map1.printCostMap()
    # move_one_tile(2)

    # Calculate the ratio between manhatten distance to actual localization steps
    # ML_ratio = manhatten_distance(0, 0, 3, 0) / 
    #     move_with_target(start, goal)
    # print(f"Manhatten distance to actual steps ratio: {ML_ratio:.2f}")

    # Calculate the ratio between optimal path steps and actual wavefront path steps
    # optimal_steps = bfs_shortest_path(map1, 3, 4, 3, 3) / len(path)
    # print(f"Optimal path steps to wavefront path steps ratio:{optimal_steps:.2f}")