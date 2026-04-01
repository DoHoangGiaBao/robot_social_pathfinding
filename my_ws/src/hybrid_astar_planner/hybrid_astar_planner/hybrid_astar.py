import numpy as np
import heapq
import math

class Node:
    def __init__(self, x, y, theta, g=0.0, h=0.0, parent=None):
        self.x = x
        self.y = y
        self.theta = (theta + np.pi) % (2 * np.pi) - np.pi
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
    def __lt__(self, other): return self.f < other.f

class HybridAStar:
    def __init__(self):
        self.res = 0.1          # Hashing resolution (larger is faster)
        self.theta_res = 0.3    
        self.move_step = 0.2    
        self.wheelbase = 0.3    
        self.steer_set = [-0.6, 0.0, 0.6]

    def plan(self, start, goal, grid, w, h, ox, oy, res):
        start_node = Node(start[0], start[1], start[2])
        
        # 1. NEW: Check the GOAL immediately. 
        # If the goal is a wall, don't even bother starting the search.
        goal_gx = int((goal[0] - ox) / res)
        goal_gy = int((goal[1] - oy) / res)
        if 0 <= goal_gx < w and 0 <= goal_gy < h:
            if grid[goal_gy * w + goal_gx] > 80: # If it's a hard wall
                print(f"DEBUG: Goal {goal[:2]} is a WALL (Val {grid[goal_gy * w + goal_gx]})")
                return None
        
        # 2. Start the Open Set with the start node REGARDLESS of validity
        # This allows the robot to 'escape' an invalid starting position.
        open_set = {self.hash(start_node): start_node}
        pq = [(start_node.f, start_node)]
        closed_set = {}

        # 3. Limit iterations to prevent freezing if no path exists
        iterations = 0
        while pq and iterations < 5000:
            iterations += 1
            _, current = heapq.heappop(pq)
            
            # Distance to goal check (using a slightly larger 0.5m bubble)
            if math.hypot(current.x - goal[0], current.y - goal[1]) < 0.5:
                return self.reconstruct_path(current)

            for steer in self.steer_set:
                child = self.step(current, steer)
                
                if self.is_valid(child, grid, w, h, ox, oy, res):
                    idx = self.hash(child)
                    if idx in closed_set: continue
                    
                    # Heuristic
                    child.h = math.hypot(child.x - goal[0], child.y - goal[1])
                    child.f = child.g + child.h
                    
                    if idx not in open_set or child.g < open_set[idx].g:
                        open_set[idx] = child
                        heapq.heappush(pq, (child.f, child))
            
            closed_set[self.hash(current)] = current
        
        print("DEBUG: Planner exhausted search or timed out.")
        return None

    def step(self, node, steer):
        nx = node.x + self.move_step * math.cos(node.theta)
        ny = node.y + self.move_step * math.sin(node.theta)
        nt = node.theta + (self.move_step / self.wheelbase) * math.tan(steer)
        return Node(nx, ny, nt, g=node.g + self.move_step, parent=node)

    def is_valid(self, node, grid, w, h, ox, oy, res):
        gx = int((node.x - ox) / res)
        gy = int((node.y - oy) / res)

        if not (0 <= gx < w and 0 <= gy < h): return True # Allow exploring outside
        
        # Check center
        if grid[gy * w + gx] > 60: return False

        # Small inflation check (1 cell around center)
        for i in range(-1, 2):
            for j in range(-1, 2):
                nx, ny = gx + i, gy + j
                if 0 <= nx < w and 0 <= ny < h:
                    if 40 < grid[ny * w + nx] <= 100: return False
        return True

    def hash(self, node):
        # Increased bin size slightly for better performance
        return (int(node.x/0.1), int(node.y/0.1), int(node.theta/0.4))

    def reconstruct_path(self, node):
        path = []
        while node:
            path.append((node.x, node.y, node.theta))
            node = node.parent
        return path[::-1]