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
