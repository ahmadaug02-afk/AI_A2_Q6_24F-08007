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
