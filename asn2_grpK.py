import map as mp
import sys
import time
import signal
import threading  # use threading to run PID control and tripod simultaneously
import ros_robot_controller_sdk as rrc
import sonar
import matplotlib.pyplot as plt

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
RF_MIDDLE_DEFAULT = 400
RF_OUTER_DEFAULT = 300

# Left Front Leg
LF_INNER_ID = 16
LF_MIDDLE_ID = 17
LF_OUTER_ID = 18
LF_INNER_DEFAULT = 500
LF_MIDDLE_DEFAULT = 600
LF_OUTER_DEFAULT = 700

# Right Middle Leg 
RM_INNER_ID = 4
RM_MIDDLE_ID = 5
RM_OUTER_ID = 6
RM_INNER_DEFAULT = 500
RM_MIDDLE_DEFAULT = 400
RM_OUTER_DEFAULT = 300

# Left Middle Leg
LM_INNER_ID = 13
LM_MIDDLE_ID = 14
LM_OUTER_ID = 15
LM_INNER_DEFAULT = 500
LM_MIDDLE_DEFAULT = 600
LM_OUTER_DEFAULT = 700

# Right Back Leg
RB_INNER_ID = 1
RB_MIDDLE_ID = 2
RB_OUTER_ID = 3
RB_INNER_DEFAULT = 500
RB_MIDDLE_DEFAULT = 400
RB_OUTER_DEFAULT = 300

# Left Back Leg
LB_INNER_ID = 10
LB_MIDDLE_ID = 11
LB_OUTER_ID = 12
LB_INNER_DEFAULT = 500
LB_MIDDLE_DEFAULT = 600
LB_OUTER_DEFAULT = 700

#Platform's value
P_ID = 21
P_DEFAULT = 500
P_RIGHT = 130
P_LEFT = 870



#Desired State, in this case is the distance threshold 35cm, times 10 because of th readding of the sonar use milimeter
DISTANCE = 350

#Direction value
NORTH = 1
EAST = 2
SOUTH = 3
WEST = 4

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

def tripod(dur=0.3, pu=0.3, lif=100, rot=90):
    duration = dur
    pause = pu
    lift = lif
    rotation = rot

    t0 = time.perf_counter()
    board.bus_servo_set_position(duration, [[RB_MIDDLE_ID, RB_MIDDLE_DEFAULT-lift], [RF_MIDDLE_ID, RF_MIDDLE_DEFAULT-lift], [LM_MIDDLE_ID, LM_MIDDLE_DEFAULT+lift], [RB_INNER_ID, RB_INNER_DEFAULT-rotation], [RF_INNER_ID, RF_INNER_DEFAULT-rotation], [LM_INNER_ID, LM_INNER_DEFAULT+rotation], [RM_INNER_ID, RM_INNER_DEFAULT+rotation], [LB_INNER_ID, LB_INNER_DEFAULT-rotation], [LF_INNER_ID, LF_INNER_DEFAULT-rotation]]) # Initial lift of legs and rotation
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
        turn_left(0.3,0.3,162,200)
        time.sleep(0.5)

def turn_right_90():
    for i in range(4):
        turn_right(0.3,0.3,162,200)
        time.sleep(0.5)

def turn_around_180():
    for i in range(8):
        turn_left(0.3,0.3,162,200)
        time.sleep(0.5)

#test the accuracy of moving one tile, important for adjusting the compounding errors
def move_one_tile(reps=1):
    repetitions = reps * 3
    for j in range(repetitions):
        tripod()

def move_with_target(start, goal):
    heading = start[2]
    
    dif_NS = goal[0] - start[0] 
    print(dif_NS)
    dif_EW = goal[1] - start[1] 
    print(dif_EW)

    if dif_NS > 0:
        move_one_tile(dif_NS)
    elif dif_NS == 0:
        pass 
    else:
        turn_around_180()
        move_one_tile(abs(dif_NS))

      
    #which means the robot need to travel east
    if dif_EW > 0:
        turn_left_90()
        #when turn left, the heading needs to be updated
        heading -= 1
        move_one_tile(dif_EW)
    elif dif_EW == 0:
        pass
    else:
        turn_right_90()
        heading += 1
        move_one_tile(abs(dif_EW))
        
    if heading == 1:
        if goal[2] == 3:
            turn_around_180()
        elif goal[2] == 2:
            turn_right_90()
        elif goal[2] == 4:
            turn_left_90()
    elif heading == 2:
        if goal[2] == 3:
            turn_right_90()
        elif goal[2] == 1:
            turn_left_90()
        elif goal[2] == 4:
            turn_around_180()
    elif heading == 3:
        if goal[2] == 1:
            turn_around_180() 
        elif goal[2] == 2:
            turn_left_90()
        elif goal[2] == 4:
            turn_right_90()   
    elif heading == 4:
        if goal[2] == 1:
            turn_right_90() 
        elif goal[2] == 2:
            turn_around_180()
        elif goal[2] == 3:
            turn_left_90()  

            


if __name__ == "__main__":
    set_all_default()
    s = sonar.Sonar()
    start_time = time.time()
    map1 = mp.CSME301Map()
    map1.printObstacleMap()
    # move_one_tile(1)
    move_with_target((0,0,3), (2,1,1))




