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

# Jixin (Kevin) Yan's code. The reactive control and turning, walking gait section is in my code. It also includes PID controller function, but it's only for my self-exploration. 
# The final version of it for evaluaton should be in my partner's code


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


#For turning, there are different stages: First, lift and rotate three legs. Second, put down the legs. Third, lift other three legs and rotate previous three legs back. Last, put down
#These four stages form a turning gait

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



#default takes 2.5 seconds
def tripod(dur=0.625, pu=0.625, lif=100, rot=100):
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


#Turning gait

def turn_left_90():
    for i in range(3):
        turn_left(0.7,0.5,226,200)
        time.sleep(0.5)

def turn_right_90():
    for i in range(3):
        turn_right(0.7,0.5,226,200)
        time.sleep(0.5)

def turn_around_180():
    for i in range(6):
        turn_left(0.7,0.5,226,200)
        time.sleep(0.5)


#For reactive control, we determine to use subsumption architecture that has different layers
#Base layer is keep walking forward

#helper function that return boolean to determin whether the robot is blocked. Return True if the robot is blocked
def is_blocked(distance):
    return distance <= DISTANCE

def platform_left(dur):
    duration = dur
    board.bus_servo_set_position(duration, [[P_ID, P_LEFT]])

def platform_right(dur):
    duration = dur
    board.bus_servo_set_position(duration, [[P_ID, P_RIGHT]])

def platform_default(dur):
    duration = dur
    board.bus_servo_set_position(duration, [[P_ID, P_DEFAULT]])


def reactive_control():
    while True:
        distance = s.getDistance()
        if not is_blocked(distance):
            tripod()
            distance = s.getDistance()
        else:
            platform_left(0.5)
            time.sleep(1)
            left_distance = s.getDistance()

            platform_right(0.8)
            time.sleep(1.2)
            right_distance = s.getDistance()

            platform_default(0.5)
            time.sleep(1)

            left_blocked = is_blocked(left_distance)
            right_blocked = is_blocked(right_distance)

            #Front, left, and right blocked
            if left_blocked and right_blocked:
                set_all_default()
                turn_around_180()
            elif right_blocked: #Front and right blocked
                set_all_default()
                turn_left_90()
            elif left_blocked: #front and left blocked
                set_all_default()
                turn_right_90()
            else: #only front blocked, randomly turn right
                set_all_default()
                turn_right_90()


def PID_control_threaded(kp, ki, kd):
    global correction_v
    global correction 

    previous_error = 0
    integral = 0

    while True:
        current_distance = s.getDistance()
        #if the error is positive, then it means the robot is too far away from the wall
        error = current_distance - DISTANCE
        #proportional term
        proportional_term = int(kp * error)
        #derivitive term
        derivitive = error - previous_error
        derivitive_term = int(kd * derivitive)
        #integral term
        integral = integral + error
        integral_term = int(ki * integral)
        #correcttion, when positive, also mean the robot is too far away from the wall
        correction = proportional_term + derivitive_term + integral_term
        #restrict correction to maximum of 150
        correction = max(-100, min(100, correction))
        #record the correction, error, distance from the wall, and time
        correction_v.append(correction)
        distance_correction.append(s.getDistance())
        error_v.append(error)
        time_v.append(time.time())
        #update previous error
        previous_error = error
        #print error and correction
        print(f"DISTENCE: {s.getDistance()}")
        print(f"ERROR: {error}, CORRECTION: {correction}")
        time.sleep(0.5)


def tripod_correction(correction, dur=0.625, pu=0.625, lif=100, rot=100):
    duration = dur
    pause = pu
    lift = lif
    rotation = rot

    board.bus_servo_set_position(duration, [
        [RB_MIDDLE_ID, RB_MIDDLE_DEFAULT - lift], 
        [RF_MIDDLE_ID, RF_MIDDLE_DEFAULT - lift], 
        [LM_MIDDLE_ID, LM_MIDDLE_DEFAULT + lift],
        [RB_INNER_ID, RB_INNER_DEFAULT - rotation - correction],
        [RF_INNER_ID, RF_INNER_DEFAULT - rotation - correction],
        [LM_INNER_ID, LM_INNER_DEFAULT + rotation - correction],
        [RM_INNER_ID, RM_INNER_DEFAULT + rotation ],
        [LB_INNER_ID, LB_INNER_DEFAULT - rotation ],
        [LF_INNER_ID, LF_INNER_DEFAULT - rotation ]
    ])
    time.sleep(pause)

    board.bus_servo_set_position(duration, [
        [RB_MIDDLE_ID, RB_MIDDLE_DEFAULT], 
        [RF_MIDDLE_ID, RF_MIDDLE_DEFAULT], 
        [LM_MIDDLE_ID, LM_MIDDLE_DEFAULT]
    ])
    time.sleep(pause)

    board.bus_servo_set_position(duration, [
        [RM_MIDDLE_ID, RM_MIDDLE_DEFAULT - lift], 
        [LB_MIDDLE_ID, LB_MIDDLE_DEFAULT + lift], 
        [LF_MIDDLE_ID, LF_MIDDLE_DEFAULT + lift],
        [RM_INNER_ID, RM_INNER_DEFAULT - rotation ],
        [LB_INNER_ID, LB_INNER_DEFAULT + rotation ],
        [LF_INNER_ID, LF_INNER_DEFAULT + rotation ],
        [RB_INNER_ID, RB_INNER_DEFAULT + rotation ],
        [RF_INNER_ID, RF_INNER_DEFAULT + rotation ],
        [LM_INNER_ID, LM_INNER_DEFAULT - rotation ]
    ])
    time.sleep(pause)

    board.bus_servo_set_position(duration, [
        [RM_MIDDLE_ID, RM_MIDDLE_DEFAULT], 
        [LB_MIDDLE_ID, LB_MIDDLE_DEFAULT], 
        [LF_MIDDLE_ID, LF_MIDDLE_DEFAULT]
    ])
    time.sleep(pause)


#Parameters still need to tune
def PID_control(direction, kp,ki,kd):
    #depends on direction of the wall
    if (direction == "left"):
        platform_left(0.5)
    elif (direction == "right"):
        platform_right(0.5)

    time.sleep(0.6)


    t1 = threading.Thread(target=PID_control_threaded, args=(kp, ki, kd), daemon=True)
    t1.start()
    time.sleep(0.5)

    while True:
        tripod_correction(correction)





if __name__ == '__main__':
    s = sonar.Sonar()
    start_time = time.time()
    set_all_default()

    #Initialize the Global Varible Correction, so that the PID control function update the correction every loop
    correction = 0
    correction_v = []
    distance_correction = []
    error_v=[]
    time_v=[]

    # reactive_control()
    PID_control(1.1, 0, 5)
    # while True:
    #     tripod(0.5, 0.4, 100, 120)
    


    
    
    













        









