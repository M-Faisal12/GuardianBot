import heapq
import math

WALL = 1
FREE = 0

# 8 possible movements (row change, col change, name, angle)
# 0° = UP, clockwise direction
MOVES = [
    (-1,  0, "UP", 0),
    ( 1,  0, "DOWN", 180),
    ( 0, -1, "LEFT", 270),
    ( 0,  1, "RIGHT", 90),
    (-1, -1, "UP_LEFT", 315),
    (-1,  1, "UP_RIGHT", 45),
    ( 1, -1, "DOWN_LEFT", 225),
    ( 1,  1, "DOWN_RIGHT", 135),
]

SQRT2 = math.sqrt(2)


# -------------------------------------------------------
# NODE (used in A*)
# -------------------------------------------------------

class Node:
    def __init__(self, pos, g=0, h=0, parent=None):
        self.pos = pos
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f


# -------------------------------------------------------
# HEURISTIC (estimated distance to goal)
# -------------------------------------------------------

def heuristic(a, b):
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return dx + dy


# -------------------------------------------------------
# GRID HELPERS
# -------------------------------------------------------

def is_free(grid, r, c):
    return 0 <= r < len(grid) and 0 <= c < len(grid[0]) and grid[r][c] != WALL


def get_neighbors(grid, pos):
    r, c = pos
    neighbors = []

    for dr, dc, name, angle in MOVES:
        nr, nc = r + dr, c + dc

        if not is_free(grid, nr, nc):
            continue

        cost = SQRT2 if dr != 0 and dc != 0 else 1

        neighbors.append(((nr, nc), name, angle, cost))

    return neighbors


# -------------------------------------------------------
# A* SEARCH ALGORITHM
# -------------------------------------------------------

def astar(grid, start, goal):

    open_list = []
    heapq.heappush(open_list, Node(start, 0, heuristic(start, goal)))

    visited_cost = {start: 0}

    while open_list:
        current = heapq.heappop(open_list)

        if current.pos == goal:
            path = []
            while current:
                path.append(current.pos)
                current = current.parent
            return path[::-1]

        for nxt, name, angle, cost in get_neighbors(grid, current.pos):
            new_cost = current.g + cost

            if nxt not in visited_cost or new_cost < visited_cost[nxt]:
                visited_cost[nxt] = new_cost
                h = heuristic(nxt, goal)
                heapq.heappush(open_list, Node(nxt, new_cost, h, current))

    return []


# -------------------------------------------------------
# PATH TO ROBOT COMMANDS
# ONLY RETURNS: angle, command, distance
# -------------------------------------------------------

def generate_commands(path):
    commands = []

    for i in range(len(path) - 1):
        r1, c1 = path[i]
        r2, c2 = path[i + 1]

        dr = r2 - r1
        dc = c2 - c1

        for mdr, mdc, name, angle in MOVES:
            if dr == mdr and dc == mdc:
                dist = SQRT2 if dr != 0 and dc != 0 else 1
                commands.append({
                    "command": name,
                    "angle": angle,
                    "distance": round(dist, 2)
                })
                break

    return commands

# -------------------------------------------------------
# SIMPLE TEST DRIVER
# -------------------------------------------------------

def print_path(path):
    print("\nPATH:")
    for p in path:
        print(p)


def print_commands(commands):
    print("\nCOMMANDS:")
    for i, cmd in enumerate(commands, 1):
        print(
            f"{i}. {cmd['command']} "
            f"angle={cmd['angle']}° "
            f"distance={cmd['distance']}"
        )


def run_test():
    # -------------------------------
    # 1. GRID (0 = free, 1 = wall)
    # -------------------------------
    grid = [
        [0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 0, 0, 0],
    ]

    # -------------------------------
    # 2. DEFINED STOPS
    # -------------------------------
    stops = {
        "A": (0, 4),
        "B": (2,0 ),
        "C": (2, 2),
    }

    # -------------------------------
    # 3. USER INPUT (choose start/goal)
    # -------------------------------
    start = stops["A"]
    goal = stops["B"]

    print("START:", start)
    print("GOAL :", goal)

    # -------------------------------
    # 4. RUN A*
    # -------------------------------
    path = astar(grid, start, goal)

    if not path:
        print("\nNo path found!")
        return

    # -------------------------------
    # 5. SHOW RESULTS
    # -------------------------------
    print_path(path)

    commands = generate_commands(path)
    print_commands(commands)


# -------------------------------------------------------
# ENTRY POINT
# -------------------------------------------------------

if __name__ == "__main__":
    run_test()
