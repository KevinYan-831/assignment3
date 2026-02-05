#!/usr/bin/env python3
"""
Wavefront Propagation Planner for CS/ME 301 Assignment

This module implements wavefront propagation for path planning on the hexapod robot.
"""

import map as mp

# DIRECTION enum is defined in map module
DIRECTION = mp.DIRECTION


def wavefront_propagation(map_obj, goal_row, goal_col):
    """
    Perform wavefront propagation from the goal cell.
    Sets up the costMap with increasing values from the goal.
    Cells with obstacles or unreachable areas get value 1000 (impassable).

    Args:
        map_obj: CSME301Map object
        goal_row: Row index of goal (0-based)
        goal_col: Column index of goal (0-based)

    Returns:
        The costMap 2D list with propagation values
    """
    # Clear the cost map first
    map_obj.clearCostMap()

    # Set goal to value 2 (allows 1-step neighbors to be 3, etc.)
    map_obj.setCost(goal_row, goal_col, 2)

    # Wavefront propagation using iterative approach
    # Continue until no more changes occur
    changed = True
    iteration = 0
    while changed:
        changed = False
        iteration += 1

        # Scan through all cells
        for i in range(map_obj.costmap_size_row):
            for j in range(map_obj.costmap_size_col):
                current_cost = map_obj.getCost(i, j)

                # If cell has been assigned a value (> 0), try to propagate
                if current_cost > 0:
                    # Try to propagate to all neighbors
                    for direction in [DIRECTION.North, DIRECTION.South, DIRECTION.East, DIRECTION.West]:
                        # Check if there's no wall in this direction
                        if map_obj.getNeighborObstacle(i, j, direction) == 0:
                            neighbor_cost = map_obj.getNeighborCost(i, j, direction)

                            # If neighbor is unvisited (0) or has higher cost, update it
                            if neighbor_cost == 0 or neighbor_cost > current_cost + 1:
                                map_obj.setNeighborCost(i, j, direction, current_cost + 1)
                                changed = True

    print(f"Wavefront propagation complete in {iteration} iterations")
    return map_obj.costMap


def get_next_step(map_obj, current_row, current_col):
    """
    Get the next step (direction to move) based on wavefront cost map.
    Returns the direction to move (neighbor with lowest cost).

    Args:
        map_obj: CSME301Map object with populated costMap
        current_row: Current row position
        current_col: Current column position

    Returns:
        DIRECTION enum value for the best next move, or None if no path
    """
    min_cost = float('inf')
    best_direction = None

    for direction in [DIRECTION.North, DIRECTION.South, DIRECTION.East, DIRECTION.West]:
        # Check if there's no wall in this direction
        if map_obj.getNeighborObstacle(current_row, current_col, direction) == 0:
            neighbor_cost = map_obj.getNeighborCost(current_row, current_col, direction)

            # Find neighbor with lowest cost (gradient descent)
            if 0 < neighbor_cost < min_cost:
                min_cost = neighbor_cost
                best_direction = direction

    return best_direction


def get_path_from_wavefront(map_obj, start_row, start_col, goal_row, goal_col):
    """
    Extract the complete path from start to goal using the wavefront cost map.

    Args:
        map_obj: CSME301Map object with populated costMap
        start_row: Starting row position
        start_col: Starting column position
        goal_row: Goal row position
        goal_col: Goal column position

    Returns:
        List of (row, col) tuples representing the path from start to goal
    """
    path = []
    current_row, current_col = start_row, start_col

    while (current_row != goal_row or current_col != goal_col):
        path.append((current_row, current_col))

        next_direction = get_next_step(map_obj, current_row, current_col)

        if next_direction is None:
            print("No path found!")
            return path

        # Update current position
        if next_direction == DIRECTION.North:
            current_row -= 1
        elif next_direction == DIRECTION.South:
            current_row += 1
        elif next_direction == DIRECTION.East:
            current_col += 1
        elif next_direction == DIRECTION.West:
            current_col -= 1

    path.append((goal_row, goal_col))
    return path


def plan_wavefront_path(start_row, start_col, start_heading,
                        goal_row, goal_col, goal_heading):
    """
    Complete wavefront planning function.
    Generates the cost map and path, then returns navigation commands.

    Args:
        start_row: Starting row (0-based, 0=top)
        start_col: Starting column (0-based, 0=left)
        start_heading: Starting heading (1=North, 2=East, 3=South, 4=West)
        goal_row: Goal row
        goal_col: Goal column
        goal_heading: Desired final heading

    Returns:
        Tuple of (cost_map, path, commands)
        - cost_map: 2D list of propagation values
        - path: List of (row, col) positions
        - commands: List of movement commands to execute
    """
    # Create map object
    map_obj = mp.CSME301Map()

    # Run wavefront propagation
    print(f"\nPlanning path from ({start_row}, {start_col}) to ({goal_row}, {goal_col})")
    cost_map = wavefront_propagation(map_obj, goal_row, goal_col)

    print("\n=== Wavefront Cost Map ===")
    map_obj.printCostMap()

    # Get the path
    path = get_path_from_wavefront(map_obj, start_row, start_col, goal_row, goal_col)

    print(f"\n=== Path Found ({len(path)} steps) ===")
    for i, (row, col) in enumerate(path):
        cost = map_obj.getCost(row, col)
        print(f"Step {i}: ({row}, {col}) - cost: {cost}")

    # Generate navigation commands
    commands = generate_navigation_commands(path, start_heading, goal_heading)

    print(f"\n=== Navigation Commands ({len(commands)} actions) ===")
    for i, cmd in enumerate(commands):
        print(f"{i+1}. {cmd}")

    return cost_map, path, commands


def generate_navigation_commands(path, start_heading, goal_heading):
    """
    Generate movement commands from a path.

    Args:
        path: List of (row, col) tuples
        start_heading: Initial heading (1=North, 2=East, 3=South, 4=West)
        goal_heading: Desired final heading

    Returns:
        List of command strings
    """
    if len(path) < 2:
        return []

    commands = []
    current_heading = start_heading

    for i in range(len(path) - 1):
        current_row, current_col = path[i]
        next_row, next_col = path[i + 1]

        # Determine direction of movement
        if next_row < current_row:
            move_direction = "North"
        elif next_row > current_row:
            move_direction = "South"
        elif next_col > current_col:
            move_direction = "East"
        elif next_col < current_col:
            move_direction = "West"
        else:
            continue  # Should not happen

        # Determine turn needed
        turn_needed = get_turn_command(current_heading, move_direction)
        if turn_needed:
            commands.append(turn_needed)

        # Move forward
        commands.append("move_forward")

        # Update heading
        if move_direction == "North":
            current_heading = 1
        elif move_direction == "East":
            current_heading = 2
        elif move_direction == "South":
            current_heading = 3
        elif move_direction == "West":
            current_heading = 4

    # Adjust final heading if needed
    final_turn = get_turn_command(current_heading, goal_heading)
    if final_turn:
        commands.append(final_turn)

    return commands


def get_turn_command(from_heading, to_heading_or_direction):
    """
    Get the turn command to change from one heading to another.

    Args:
        from_heading: Current heading (1-4 or string name)
        to_heading_or_direction: Target heading (1-4 or string name)

    Returns:
        Command string or None if no turn needed
    """
    # Convert to numeric
    heading_map = {"North": 1, "East": 2, "South": 3, "West": 4}
    reverse_map = {1: "North", 2: "East", 3: "South", 4: "West"}

    if isinstance(from_heading, str):
        from_heading = heading_map.get(from_heading, from_heading)
    if isinstance(to_heading_or_direction, str):
        to_heading_or_direction = heading_map.get(to_heading_or_direction, to_heading_or_direction)

    from_heading = int(from_heading)
    to_heading = int(to_heading_or_direction)

    if from_heading == to_heading:
        return None

    # Calculate turn direction
    diff = (to_heading - from_heading) % 4
    if diff == 1:
        return "turn_left_90"
    elif diff == 2:
        return "turn_around_180"
    elif diff == 3:
        return "turn_right_90"

    return None


# Test function
def test_wavefront():
    """Test the wavefront propagation with sample start/goal positions."""
    print("=== Testing Wavefront Propagation ===\n")

    # Test case 1: Navigate from top-left to bottom-right
    print("\n--- Test 1: (0,0) to (3,5) ---")
    cost_map, path, commands = plan_wavefront_path(
        start_row=0, start_col=0, start_heading=3,  # Start facing South
        goal_row=3, goal_col=5, goal_heading=3
    )

    # Test case 2: Navigate with obstacles
    print("\n--- Test 2: (0,2) to (3,2) ---")
    cost_map, path, commands = plan_wavefront_path(
        start_row=0, start_col=2, start_heading=3,
        goal_row=3, goal_col=2, goal_heading=1
    )

    # Test case 3: Short path
    print("\n--- Test 3: (1,1) to (1,2) ---")
    cost_map, path, commands = plan_wavefront_path(
        start_row=1, start_col=1, start_heading=2,
        goal_row=1, goal_col=2, goal_heading=2
    )


if __name__ == "__main__":
    test_wavefront()
