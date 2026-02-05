import map as mp
#x is south north, south is positive. Y is east west, east is positive
#Algorithm follow the cost gradient
DIRECTION = mp.DIRECTION




def wavefront_propagation(given_map, goal_x, goal_y):
    # Clear the cost map first
    given_map.clearCostMap()
    #set the goal position's cost to 2, since 1 and 0 is usually indicated as something else
    given_map.setCost(goal_x, goal_y, 2)
    # Wavefront propagation using iterative approach
    # Continue until no more changes occur
    changed = True
    while changed:
        changed = False
        # Scan through all cells
        for i in range(given_map.costmap_size_row):
            for j in range(given_map.costmap_size_col):
                current_cost = given_map.getCost(i, j)
                # If cell has been assigned a value (> 0), try to propagate
                if current_cost > 0:
                    # Increment cost value of neighbor to the current cost cell
                    for direction in [DIRECTION.North, DIRECTION.South, DIRECTION.East, DIRECTION.West]:
                        # Check wall existence in differen directions, if no:
                        if given_map.getNeighborObstacle(i, j, direction) == 0:
                            neighbor_cost = given_map.getNeighborCost(i, j, direction)
                            # If neighbor is unvisited (0) or has higher cost, update it
                            if neighbor_cost == 0 or neighbor_cost > current_cost + 1:
                                given_map.setNeighborCost(i, j, direction, current_cost + 1)
                                changed = True
                else:
                    #for not possible path, set a extremely high value so that the robot won't be interfere
                    given_map.setCost(i,j,10000)
    return given_map.costMap

#Traverse the cost map from the current position, and find the optimal direction to travel for the next step, 
# which by finding the lowest non obstacle cost cell near the position.
def next_step(given_map, cur_x, cur_y):
    cur_cost = given_map.getCost(cur_x, cur_y)
    cur_min_cost = cur_cost
    best_direction = None
    for direction in [DIRECTION.North, DIRECTION.South, DIRECTION.East, DIRECTION.West]:
        #must be non obstacle
        if given_map.getNeighborObstacle(cur_x, cur_y, direction) == 0:
            #must be smaller than the current cost 
            if given_map.getNeighborCost(cur_x, cur_y, direction) < cur_cost:
                #must be the smallest, which is the best, if equally small, then just go to the first find one
                if given_map.getNeighborCost(cur_x, cur_y, direction) < cur_min_cost:
                    cur_min_cost = given_map.getNeighborCost(cur_x, cur_y, direction)
                    best_direction = direction
    return best_direction



           
    

            


if __name__ == "__main__":
    your_map = mp.CSME301Map()
    your_map.printCostMap()

    your_map.costMap = wavefront_propagation(your_map,3,5)
    your_map.printCostMap()
    print(next_step(your_map, 0,0))
