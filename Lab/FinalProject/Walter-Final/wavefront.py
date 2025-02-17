from collections import deque
import numpy as np

def wavefront_algorithm(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
    
    # Initialize wavefront
    queue = deque([goal])
    grid[goal[0]][goal[1]] = 1  # Set goal value to 1
    
    # Wave Expansion
    while queue:
        r, c = queue.popleft()
        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:  # Check valid empty cell
                grid[nr][nc] = grid[r][c] + 1
                queue.append((nr, nc))

    # Path Backtracking
    path = []
    if grid[start[0]][start[1]] == 0:
        return None  # No path found

    r, c = start
    while (r, c) != goal:
        path.append((r, c))
        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == grid[r][c] - 1:
                r, c = nr, nc
                break
    path.append(goal)  # Add goal to path
    return path

# Example Grid (0 = free, -1 = obstacle)
grid = np.array([
    [0,  0,  0,  0, -1],
    [0, -1, -1,  0, -1],
    [0,  0,  0,  0,  0],
    [-1, 0, -1, -1,  0],
    [0,  0,  0,  0,  0]
])

start = (4, 0)
goal = (0, 4)

# Run Wavefront Algorithm
path = wavefront_algorithm(grid, start, goal)

# Print Path
if path:
    print("Path found:", path)
else:
    print("No path available.")
