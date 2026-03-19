import heapq
import random
import time
import math

ROWS, COLS = 40, 70
KM_PER_CELL = 70 / COLS

DENSITY = {"low": 0.15, "medium": 0.30, "high": 0.45}

FREE = "."
OBS  = "#"
START = "S"
GOAL  = "G"
PATH  = "*"
EXPLORED = "~"

def generate_grid(density):
    return [[OBS if random.random() < density else FREE for _ in range(COLS)] for _ in range(ROWS)]

def print_grid(grid, path=set(), explored=set(), start=None, goal=None):
    for r in range(ROWS):
        row = ""
        for c in range(COLS):
            pos = (r, c)
            if pos == start:
                row += "S"
            elif pos == goal:
                row += "G"
            elif pos in path:
                row += "*"
            elif pos in explored:
                row += "~"
            elif grid[r][c] == OBS:
                row += "#"
            else:
                row += "."
        print(row)

def dijkstra(grid, start, goal):
    dist = [[math.inf] * COLS for _ in range(ROWS)]
    prev = [[None] * COLS for _ in range(ROWS)]
    sr, sc = start
    dist[sr][sc] = 0
    heap = [(0.0, start)]
    visited = set()
    dirs = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

    while heap:
        cost, (r, c) = heapq.heappop(heap)
        if (r, c) in visited:
            continue
        visited.add((r, c))
        if (r, c) == goal:
            break
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if 0 <= nr < ROWS and 0 <= nc < COLS and grid[nr][nc] != OBS and (nr, nc) not in visited:
                w = math.sqrt(2) if dr != 0 and dc != 0 else 1.0
                nd = cost + w
                if nd < dist[nr][nc]:
                    dist[nr][nc] = nd
                    prev[nr][nc] = (r, c)
                    heapq.heappush(heap, (nd, (nr, nc)))

    if math.isinf(dist[goal[0]][goal[1]]):
        return None, visited, dist

    path = []
    cur = goal
    while cur:
        path.append(cur)
        r, c = cur
        cur = prev[r][c]
    path.reverse()
    return path, visited, dist

def get_valid_input(prompt, grid):
    while True:
        try:
            raw = input(prompt).strip().split()
            r, c = int(raw[0]), int(raw[1])
            if not (0 <= r < ROWS and 0 <= c < COLS):
                print(f"  Out of bounds. Enter row 0-{ROWS-1} and col 0-{COLS-1}.")
            elif grid[r][c] == OBS:
                print("  That cell is an obstacle. Choose another.")
            else:
                return (r, c)
        except (ValueError, IndexError):
            print("  Invalid input. Enter row and col as two numbers, e.g.: 5 10")

def main():
    print("=" * COLS)
    print("UGV BATTLEFIELD PATHFINDER — Dijkstra's Algorithm")
    print(f"Grid: {ROWS}x{COLS} cells  |  Scale: 1 cell = {KM_PER_CELL:.2f} km")
    print("=" * COLS)

    print("\nObstacle density:  1) Low (15%)   2) Medium (30%)   3) High (45%)")
    choice = input("Choose [1/2/3]: ").strip()
    density = {"1": "low", "2": "medium", "3": "high"}.get(choice, "medium")
    print(f"Using: {density} density ({int(DENSITY[density]*100)}%)\n")

    grid = generate_grid(DENSITY[density])

    print_grid(grid)
    print(f"\nLegend:  # obstacle  . free  S start  G goal  * path  ~ explored")
    print(f"Enter positions as: row col  (row 0-{ROWS-1}, col 0-{COLS-1})\n")

    start = get_valid_input("Start node (row col): ", grid)
    goal  = get_valid_input("Goal node  (row col): ", grid)

    print("\nRunning Dijkstra...")
    t0 = time.perf_counter()
    path, explored, dist = dijkstra(grid, start, goal)
    t1 = time.perf_counter()

    print()
    if path is None:
        print_grid(grid, explored=explored, start=start, goal=goal)
        print("\nNo path found — goal is unreachable.")
    else:
        path_set = set(path)
        print_grid(grid, path=path_set, explored=explored, start=start, goal=goal)

        path_cost   = dist[goal[0]][goal[1]]
        path_km     = path_cost * KM_PER_CELL
        straight    = math.hypot(goal[0]-start[0], goal[1]-start[1]) * KM_PER_CELL
        efficiency  = (straight / path_km * 100) if path_km > 0 else 100
        obs_count   = sum(grid[r][c] == OBS for r in range(ROWS) for c in range(COLS))

        print("\n" + "=" * 40)
        print("MEASURES OF EFFECTIVENESS")
        print("=" * 40)
        print(f"  Path length (km)      : {path_km:.2f} km")
        print(f"  Path nodes            : {len(path)}")
        print(f"  Nodes explored        : {len(explored)}")
        print(f"  Straight-line dist    : {straight:.2f} km")
        print(f"  Path efficiency       : {efficiency:.1f}%")
        print(f"  Obstacle density      : {obs_count/(ROWS*COLS)*100:.1f}%")
        print(f"  Compute time          : {(t1-t0)*1000:.2f} ms")
        print("=" * 40)

if __name__ == "__main__":
    main()