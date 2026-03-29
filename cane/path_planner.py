import numpy as np
import heapq
import math

class PathPlanner:
    def __init__(self, width=6.0, height=6.0, resolution=0.1, inflation_radius=0.4):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.inflation_radius = inflation_radius
        
        self.cols = int(width / resolution)
        self.rows = int(height / resolution)
        
        # Origin is bottom center of the grid [rows-1, cols//2]
        self.origin_x = width / 2.0
        self.origin_y = 0.0
        
        self.grid = np.zeros((self.rows, self.cols), dtype=np.uint8)

    def reset_map(self):
        self.grid.fill(0)

    def world_to_grid(self, x, y):
        # x is right-left (positive is right), y is forward (positive is forward)
        gx = int((x + self.origin_x) / self.resolution)
        gy = int((self.height - y) / self.resolution)
        
        # Clamp off-by-one errors (like robot starting at y=0 resulting in gy=rows)
        gx = max(0, min(gx, self.cols - 1))
        gy = max(0, min(gy, self.rows - 1))
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = (gx * self.resolution) - self.origin_x
        y = self.height - (gy * self.resolution)
        return x, y

    def add_obstacle(self, x, y):
        # Ignore obstacles physically outside our 4x4 tracking grid
        if x < -self.origin_x or x > self.origin_x or y < 0 or y > self.height:
            return
            
        cx, cy = self.world_to_grid(x, y)
        inflation_cells = int(self.inflation_radius / self.resolution)
        
        for dy in range(-inflation_cells, inflation_cells + 1):
            for dx in range(-inflation_cells, inflation_cells + 1):
                if dx*dx + dy*dy <= inflation_cells*inflation_cells:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < self.cols and 0 <= ny < self.rows:
                        self.grid[ny, nx] = 1 # Blocked

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def astar(self, start_world, goal_world):
        start = self.world_to_grid(start_world[0], start_world[1])
        goal = self.world_to_grid(goal_world[0], goal_world[1])
        
        if not (0 <= start[0] < self.cols and 0 <= start[1] < self.rows): return []
        if not (0 <= goal[0] < self.cols and 0 <= goal[1] < self.rows): return []

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        
        motions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                # Return path in world coordinates
                return [self.grid_to_world(p[0], p[1]) for p in path]
                
            for dx, dy in motions:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < self.cols and 0 <= neighbor[1] < self.rows:
                    if self.grid[neighbor[1], neighbor[0]] == 1:
                        continue
                        
                    cost = math.hypot(dx, dy)
                    tentative_g_score = g_score[current] + cost
                    
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score, neighbor))
                        
        return []

    def pure_pursuit(self, current_pos, path, lookahead=0.5):
        """Returns the angle (-180 to 180) to steer towards the lookahead point."""
        if not path:
            return 0.0 # Straight ahead
            
        target = path[-1]
        for pt in path:
            dist = math.hypot(pt[0] - current_pos[0], pt[1] - current_pos[1])
            if dist >= lookahead:
                target = pt
                break
                
        # Calculate angle to target relative to robot's forward vector (which is +Y)
        dx = target[0] - current_pos[0]
        dy = target[1] - current_pos[1]
        # Atan2 standard is (y,x). We want angle from +Y axis, so we swap them.
        angle_rad = math.atan2(dx, dy) 
        return math.degrees(angle_rad)
