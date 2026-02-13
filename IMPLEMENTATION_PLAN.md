# Frontier-Based Exploration Implementation Plan

## Overview
Implement frontier-based exploration for the hexapod robot to build an unknown 8x8 map using wall detection and greedy frontier selection.

## Changes to asn2_grpK.py

### 1. Complete `select_next_frontier()` (line 1046-1049)
```python
def select_next_frontier(cur_x, cur_y, frontiers, given_map):
    """Select the nearest frontier cell using BFS shortest path distance."""
    if not frontiers:
        return None

    best_coor = None
    min_distance = float('inf')

    for (fx, fy) in frontiers:
        # Compute shortest path distance respecting walls
        dist = computer_path_distance(cur_x, cur_y, fx, fy, given_map)
        if dist is not None and dist < min_distance:
            min_distance = dist
            best_coor = (fx, fy)

    return best_coor
```

### 2. Complete `computer_path_distance()` (line 1054-1055)
```python
def computer_path_distance(cur_x, cur_y, frontier_x, frontier_y, given_map):
    """Compute shortest path distance using BFS."""
    steps = bfs_shortest_path(given_map, cur_x, cur_y, frontier_x, frontier_y)
    return steps if steps is not None else float('inf')
```

### 3. Complete `frontier_mapping()` (line 891-917)
```python
def frontier_mapping(given_map):
    """Explore and build unknown map using frontier-based approach."""
    # Initialize robot state
    cur_x, cur_y, cur_heading = 0, 0, 1
    visited = set()
    frontiers = set()

    visited.add((cur_x, cur_y))

    print("Starting frontier-based exploration...")

    while True:
        # 1. Scan current cell to detect walls
        print(f"\n=== At position ({cur_x}, {cur_y}), heading={cur_heading} ===")
        scan_and_detect_walls(cur_x, cur_y, cur_heading, given_map)

        # 2. Update frontier set with newly discovered neighbors
        update_frontier(given_map, visited, frontiers, cur_x, cur_y)

        print(f"Visited: {len(visited)}, Frontiers: {frontiers}")

        # 3. Check completion condition
        if not frontiers and len(visited) > 1:
            print("\n=== Exploration complete! All reachable cells visited. ===")
            break

        # 4. Select nearest frontier
        next_target = select_next_frontier(cur_x, cur_y, frontiers, given_map)
        if next_target is None:
            print("\n=== No more reachable frontiers. Exploration complete. ===")
            break

        print(f"Next target: {next_target}")

        # 5. Navigate to target
        start_state = (cur_x, cur_y, cur_heading)
        goal_state = (next_target[0], next_target[1], cur_heading)

        # move_with_target returns steps but we also need final heading
        # We'll update position based on target reached
        move_with_target(start_state, goal_state)

        # 6. Update position after movement
        cur_x, cur_y = next_target
        # Heading is maintained from move_with_target's final state
        # For simplicity, we'll track it through the function

        visited.add((cur_x, cur_y))
        frontiers.discard((cur_x, cur_y))

    # Print final constructed map
    print("\n=== Final constructed map ===")
    given_map.printObstacleMap()
    print(f"Total cells visited: {len(visited)}")

    return visited, frontiers
```

### 4. Update `move_with_target()` to return final heading (line 333-389)
Change return from:
```python
return abs(dif_NS) + abs(dif_EW)
```
To:
```python
return abs(dif_NS) + abs(dif_EW), heading
```

### 5. Update `__main__` section (line 1070+)
Add after `map2 = mp.CSME301Map(8,8)`:
```python
# Run frontier-based exploration
print("\n=== Frontier-Based Mapping ===")
frontier_mapping(map2)
```

## Testing Strategy
1. Start with robot at (0,0), verify initial scan works
2. Verify frontier selection picks nearest accessible cell
3. Verify navigation reaches each frontier correctly
4. Verify exploration terminates when map is complete
5. Print final map for verification

## Order of Implementation
1. `computer_path_distance()` - simplest, wraps existing BFS
2. `select_next_frontier()` - uses above
3. `move_with_target()` return value - small change
4. `frontier_mapping()` - main loop integration
5. Test and verify
