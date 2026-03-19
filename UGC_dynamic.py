import heapq
import random
import time
import math
import os

ROWS, COLS = 30, 60
KM_PER_CELL = 70 / COLS
SENSE_RADIUS = 5
DYNAMIC_OBS_INTERVAL = 3

DENSITY = {"1": 0.10, "2": 0.20, "3": 0.35}

def clear():
    os.system('cls' if os.name == 'nt' else 'clear')

def generate_grid(density):
    return [[1 if random.random() < density else 0 for _ in range(COLS)] for _ in range(ROWS)]

def heuristic(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

def neighbors(r, c):
    dirs = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    for dr, dc in dirs:
        nr, nc = r+dr, c+dc
        if 0 <= nr < ROWS and 0 <= nc < COLS:
            yield (nr, nc), (math.sqrt(2) if dr and dc else 1.0)

class DStarLite:
    def __init__(self, grid, start, goal):
        self.grid = [row[:] for row in grid]
        self.start = start
        self.goal = goal
        self.km = 0
        self.g = {}
        self.rhs = {}
        self.heap = []
        for r in range(ROWS):
            for c in range(COLS):
                self.g[(r,c)] = math.inf
                self.rhs[(r,c)] = math.inf
        self.rhs[goal] = 0
        heapq.heappush(self.heap, (self.key(goal), goal))

    def key(self, u):
        k2 = min(self.g[u], self.rhs[u])
        return (k2 + heuristic(self.start, u) + self.km, k2)

    def update_vertex(self, u):
        if u != self.goal:
            self.rhs[u] = min(
                (self.g[nb] + w for nb, w in neighbors(*u) if self.grid[nb[0]][nb[1]] == 0),
                default=math.inf
            )
        self.heap = [(k, v) for k, v in self.heap if v != u]
        heapq.heapify(self.heap)
        if self.g[u] != self.rhs[u]:
            heapq.heappush(self.heap, (self.key(u), u))

    def compute_path(self):
        while self.heap and (
            self.heap[0][0] < self.key(self.start) or
            self.rhs[self.start] != self.g[self.start]
        ):
            k_old, u = heapq.heappop(self.heap)
            if k_old < self.key(u):
                heapq.heappush(self.heap, (self.key(u), u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for nb, _ in neighbors(*u):
                    self.update_vertex(nb)
            else:
                self.g[u] = math.inf
                self.update_vertex(u)
                for nb, _ in neighbors(*u):
                    self.update_vertex(nb)

    def extract_path(self):
        path = [self.start]
        cur = self.start
        for _ in range(ROWS * COLS):
            if cur == self.goal:
                break
            best, best_cost = None, math.inf
            for nb, w in neighbors(*cur):
                if self.grid[nb[0]][nb[1]] == 0:
                    cost = self.g[nb] + w
                    if cost < best_cost:
                        best_cost = cost
                        best = nb
            if best is None:
                return None
            path.append(best)
            cur = best
        return path if cur == self.goal else None

    def update_obstacle(self, cell, is_obs):
        r, c = cell
        self.grid[r][c] = 1 if is_obs else 0
        for nb, _ in neighbors(r, c):
            self.update_vertex(nb)
        self.update_vertex(cell)

    def move_start(self, new_start):
        self.km += heuristic(self.start, new_start)
        self.start = new_start

def sense(true_grid, known_grid, pos, radius, dynamic_obs):
    newly = []
    r0, c0 = pos
    for r in range(max(0, r0-radius), min(ROWS, r0+radius+1)):
        for c in range(max(0, c0-radius), min(COLS, c0+radius+1)):
            if math.hypot(r-r0, c-c0) <= radius:
                if true_grid[r][c] == 1 and known_grid[r][c] == 0:
                    known_grid[r][c] = 1
                    newly.append((r, c))
                elif true_grid[r][c] == 0 and known_grid[r][c] == 1 and (r,c) not in dynamic_obs:
                    known_grid[r][c] = 0
                    newly.append((r, c))
    return newly

def spawn_dynamic(true_grid, known_grid, static_obs, ugv_pos, goal):
    for _ in range(100):
        r, c = random.randint(0, ROWS-1), random.randint(0, COLS-1)
        if (true_grid[r][c] == 0 and (r,c) != ugv_pos and (r,c) != goal
                and (r,c) not in static_obs
                and math.hypot(r-ugv_pos[0], c-ugv_pos[1]) > SENSE_RADIUS):
            return (r, c)
    return None

def render(true_grid, known_grid, path, pos, goal, start, step, replans, dynamic_obs, total_km):
    path_set = set(path) if path else set()
    for r in range(ROWS):
        row = ""
        for c in range(COLS):
            p = (r, c)
            if p == pos:   row += "U"
            elif p == goal: row += "G"
            elif p == start and p != pos: row += "s"
            elif p in dynamic_obs and known_grid[r][c] == 1: row += "D"
            elif true_grid[r][c] == 1 and known_grid[r][c] == 1: row += "#"
            elif p in path_set: row += "*"
            else: row += "."
        print(row)
    print(f"\nLegend: U=UGV  G=goal  s=start  #=obstacle  D=dynamic  *=path  .=free/unknown")
    print(f"Step:{step}  Dist:{total_km:.1f}km  Replans:{replans}  SenseR:{SENSE_RADIUS}")

def get_pos(prompt, grid):
    while True:
        try:
            r, c = map(int, input(prompt).split())
            if 0 <= r < ROWS and 0 <= c < COLS and grid[r][c] == 0:
                return (r, c)
            print(f"  Obstacle or out of bounds. Try again.")
        except (ValueError, IndexError):
            print("  Enter as: row col")

def main():
    print("=" * COLS)
    print("UGV DYNAMIC PATHFINDER  —  D* Lite")
    print(f"Grid {ROWS}x{COLS} | {KM_PER_CELL:.2f} km/cell | Sense radius {SENSE_RADIUS}")
    print("=" * COLS)
    print("\nDensity:  1) Low 10%   2) Medium 20%   3) High 35%")
    density = DENSITY.get(input("Choose [1/2/3]: ").strip(), 0.20)

    true_grid = generate_grid(density)
    known_grid = [[0]*COLS for _ in range(ROWS)]

    print(f"\nGrid ready. Enter positions as: row col")
    start = get_pos(f"Start (0-{ROWS-1}, 0-{COLS-1}): ", true_grid)
    goal  = get_pos(f"Goal  (0-{ROWS-1}, 0-{COLS-1}): ", true_grid)
    true_grid[start[0]][start[1]] = 0
    true_grid[goal[0]][goal[1]]   = 0

    static_obs = {(r,c) for r in range(ROWS) for c in range(COLS) if true_grid[r][c]==1}
    dynamic_obs = {}

    dstar = DStarLite(known_grid, start, goal)
    pos, step, replans, total_dist = start, 0, 0, 0.0
    t0 = time.perf_counter()

    newly = sense(true_grid, known_grid, pos, SENSE_RADIUS, dynamic_obs)
    for cell in newly:
        dstar.update_obstacle(cell, True)
    dstar.compute_path()
    replans += 1

    while pos != goal:
        path = dstar.extract_path()
        clear()
        render(true_grid, known_grid, path[1:] if path else [], pos, goal, start,
               step, replans, dynamic_obs, total_dist)

        if not path or len(path) < 2:
            print("\nNo path — goal unreachable.")
            break

        next_pos = path[1]

        if true_grid[next_pos[0]][next_pos[1]] == 1:
            known_grid[next_pos[0]][next_pos[1]] = 1
            dstar.update_obstacle(next_pos, True)
            dstar.compute_path()
            replans += 1
            continue

        total_dist += math.hypot(next_pos[0]-pos[0], next_pos[1]-pos[1]) * KM_PER_CELL
        dstar.move_start(next_pos)
        pos = next_pos
        step += 1

        newly = sense(true_grid, known_grid, pos, SENSE_RADIUS, dynamic_obs)
        need_replan = bool(newly)
        for cell in newly:
            dstar.update_obstacle(cell, known_grid[cell[0]][cell[1]] == 1)

        if step % DYNAMIC_OBS_INTERVAL == 0:
            cell = spawn_dynamic(true_grid, known_grid, static_obs, pos, goal)
            if cell:
                true_grid[cell[0]][cell[1]] = 1
                dynamic_obs[cell] = step
                if math.hypot(cell[0]-pos[0], cell[1]-pos[1]) <= SENSE_RADIUS:
                    known_grid[cell[0]][cell[1]] = 1
                    dstar.update_obstacle(cell, True)
                    need_replan = True
            for cell, born in list(dynamic_obs.items()):
                if step - born > 8:
                    true_grid[cell[0]][cell[1]] = 0
                    del dynamic_obs[cell]
                    if known_grid[cell[0]][cell[1]] == 1:
                        known_grid[cell[0]][cell[1]] = 0
                        dstar.update_obstacle(cell, False)
                        need_replan = True

        if need_replan:
            dstar.compute_path()
            replans += 1

        time.sleep(0.12)

    t1 = time.perf_counter()
    if pos == goal:
        clear()
        render(true_grid, known_grid, [], pos, goal, start, step, replans, dynamic_obs, total_dist)
        straight = math.hypot(goal[0]-start[0], goal[1]-start[1]) * KM_PER_CELL
        eff = (straight / total_dist * 100) if total_dist > 0 else 100
        print("\n" + "="*45)
        print("MEASURES OF EFFECTIVENESS")
        print("="*45)
        print(f"  Path length (km)     : {total_dist:.2f} km")
        print(f"  Steps taken          : {step}")
        print(f"  Straight-line dist   : {straight:.2f} km")
        print(f"  Path efficiency      : {eff:.1f}%")
        print(f"  Replans triggered    : {replans}")
        print(f"  Dynamic obs spawned  : {len(dynamic_obs)}")
        print(f"  Total time           : {(t1-t0):.2f} s")
        print("="*45)

if __name__ == "__main__":
    main()