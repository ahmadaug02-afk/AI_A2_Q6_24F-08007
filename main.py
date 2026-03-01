# main.py — Informed Pathfinder | AI Assignment 2
import time, math, random, heapq, sys, pygame

FREE_SPACE, OBSTACLE, ORIGIN, TARGET = 0, 1, 2, 3

class Node:
    def __init__(self, y_idx: int, x_idx: int):
        self.y, self.x = y_idx, x_idx
        self.state = FREE_SPACE
        self.cost_g, self.score_f = float('inf'), float('inf')
        self.heur_h = 0.0
        self.came_from = None
        self.is_frontier = self.is_explored = self.is_route = False

    @property
    def coords(self): return (self.y, self.x)
    def is_passable(self): return self.state != OBSTACLE
    def clear_search_data(self):
        self.cost_g, self.score_f = float('inf'), float('inf')
        self.heur_h, self.came_from = 0.0, None
        self.is_frontier = self.is_explored = self.is_route = False
    def __lt__(self, other): return self.score_f < other.score_f

class Environment:
    def __init__(self, height: int, width: int):
        self.height, self.width = height, width
        self.matrix = [[Node(r, c) for c in range(width)] for r in range(height)]
        self.origin_node = self.target_node = None
        self._init_endpoints()

    def fetch_node(self, y, x):
        return self.matrix[y][x] if 0 <= y < self.height and 0 <= x < self.width else None

    def get_adjacent(self, current_node: Node):
        neighbors = []
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            adj = self.fetch_node(current_node.y + dy, current_node.x + dx)
            if adj and adj.is_passable(): neighbors.append(adj)
        return neighbors

    def _init_endpoints(self):
        self.set_origin(self.matrix[0][0])
        self.set_target(self.matrix[self.height - 1][self.width - 1])

        def set_origin(self, node: Node):
        if self.origin_node: self.origin_node.state = FREE_SPACE
        self.origin_node, node.state = node, ORIGIN

    def set_target(self, node: Node):
        if self.target_node: self.target_node.state = FREE_SPACE
        self.target_node, node.state = node, TARGET

    def force_obstacle(self, y, x):
        n = self.fetch_node(y, x)
        if n and n.state not in (ORIGIN, TARGET): n.state = OBSTACLE

    def clear_obstacle(self, y, x):
        n = self.fetch_node(y, x)
        if n and n.state == OBSTACLE: n.state = FREE_SPACE

    def randomize_terrain(self, wall_chance=0.30):
        for row in self.matrix:
            for n in row:
                if n.state not in (ORIGIN, TARGET):
                    n.state = OBSTACLE if random.random() < wall_chance else FREE_SPACE

                    def wipe_search_history(self):
        for row in self.matrix:
            for n in row: n.clear_search_data()

    def factory_reset(self):
        for row in self.matrix:
            for n in row:
                if n.state not in (ORIGIN, TARGET): n.state = FREE_SPACE
                n.clear_search_data()

    def spawn_dynamic_block(self):
        open_spaces = [n for row in self.matrix for n in row if n.state == FREE_SPACE and not n.is_route]
        if open_spaces:
            chosen = random.choice(open_spaces)
            chosen.state = OBSTACLE
            return chosen
        return None
