from heapq import heappush as Push
from heapq import heappop as Pop
import numpy as np
from math import *
from typing import Optional
from robot_class import robot as Robot


class Node:
    def __init__(self, parent: Optional['Node'], coordinate: tuple, cost: int, heuristic: int, dir: int):
        self.PARENT = parent
        self.COORDINATE = coordinate
        self.COST = cost
        self.HEURISTIC = heuristic
        self.DIRECTION = dir  # 0b00 : front  0b11 : back 0b01 : left 0b10: right

    def __lt__(self, other):
        return (self.COST + self.HEURISTIC) < (other.COST + other.HEURISTIC)

    def __eq__(self, other) -> bool:
        if isinstance(other, Node):
            if self.COORDINATE == other.COORDINATE and self.HEURISTIC == other.HEURISTIC and self.COST == other.COST:
                return True
        return False

    def __hash__(self) -> int:
        return self.COST + self.HEURISTIC


class TimeAstar:
    ## Size, Radius , robots( center coordinate:tuple , direction:int,straight_speed:int,rotate_speed:int, stop:int, color:str) goal: ㄷ자 , Obstacle [ [ㄷ자],[ㄷ자]]
    def __init__(self, SIZE: int , Radius : int, robots: list, goal : list, obstacles: list) -> None:
        self.SIZE = SIZE
        self.robots = robots.copy()
        self.MAP = [[0] * SIZE for _ in range(SIZE)]
        self.TimeTable = [[[[0, 0] for _ in range(len(robots))] for _ in range(SIZE)] for _ in range(SIZE)]
        self.COST_RATIO = 5
        self.Radius = Radius
        self.set_goal(tuple(np.mean(goal,axis=0).astype(int)))
        obstacles.append(goal)
        self.set_obstacle(obstacles)

    def set_goal(self, goal: tuple):
        for robot in self.robots:
            robot.GOAL = goal

    def distance(self, A: tuple, B: tuple) -> int:  # 맨하튼 거리
        return abs(A[0] - B[0]) + abs(A[1] - B[1])

    def put_robots(self, robotlist: list):
        self.robots = robotlist.copy()

    def draw_line(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while x0 != x1 or y0 != y1:
            self.MAP[y0][x0] = 1
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def set_obstacle(self, obstacles) -> None:
        for obstacle in obstacles:
            self.draw_line(obstacle[0][0], obstacle[0][1], obstacle[1][0], obstacle[1][1])
            self.draw_line(obstacle[1][0], obstacle[1][1], obstacle[2][0], obstacle[2][1])
            self.draw_line(obstacle[2][0], obstacle[2][1], obstacle[3][0], obstacle[3][1])
            self.draw_line(obstacle[3][0], obstacle[3][1], obstacle[0][0], obstacle[0][1])

    def CCW(self, A, B):
        # A와 B의 외적 계산
        cross_product = np.cross(A, B)

        if 0 < cross_product:
            return 0
        elif 0 > cross_product:
            return 1
        return 2

    def ToCommand(self, idx: int):
        # i = 0
        # modified_list = []
        # command_list = []
        # dir = { 
        #     (0,0) : 'S',
        #     (0,1) : 'F', 
        #     (1,0) : 'F',
        #     (0,-1) : 'B',
        #     (-1,0) : 'B',
        #     }

        # while i < len(vector_list) - 1:
        #     j = i + 1
        #     while j < len(vector_list) and vector_list[i] == vector_list[j]:
        #         j += 1

        #     ccw = self.CCW(modified_list[-1], vector_list[j - 1]) if modified_list else None
        #     R = ("R90", "R-90", "")[ccw] if ccw is not None else ""
        #     if R:
        #         command_list.append(R)

        #     if vector_list[i] != (0, 0):
        #         modified_list.append(vector_list[i])
        #     command_list.append(dir[vector_list[i]] + str(j - i))
        #     i = j

        # else:
        #     if i < len(vector_list):
        #         command_list.append(dir[tuple(vector_list[i])] + "1")

        # return '/'.join(command_list)
        return 'ab'

    def path_tracking(self, idx: int, T_Node: Node) -> None:
        List = []
        Table = self.TimeTable

        while T_Node.PARENT is not None:
            x, y = T_Node.COORDINATE
            Table[y][x][idx] = [(T_Node.COST - T_Node.PARENT.COST)//self.COST_RATIO, T_Node.COST//self.COST_RATIO]
            print(T_Node.COORDINATE, T_Node.COST)
            List.append((T_Node.PARENT.COORDINATE[0], T_Node.PARENT.COORDINATE[1]))
            T_Node = T_Node.PARENT
        List.reverse()
        List.append(self.GOAL)
        print(List)
        # self.robots[idx].put_path(self.ToCommand(List))

    def Robot_sort(self):
        self.robots.sort(key=lambda x: self.distance(x.coordinate, x.GOAL))
    def is_Range(self,A:tuple,B:tuple):
        dy = B[1] - A[1]; dx = B[0] - A[0]
        return dy*dy + dx *dx <= self.Redius*self.Redius

    def Search(self, idx: int) -> None:  # Robot Path finding
        # Heuristic = Distance  // F = G(현재까지 온 거리) + H(맨하튼 거리)

        ROBOT = self.robots[idx]
        GOAL = ROBOT.GOAL
        SPEED = ROBOT.STRAIGHT * self.COST_RATIO
        ROTATE = ROBOT.ROTATE * self.COST_RATIO
        STOP = ROBOT.STOP * self.COST_RATIO

        Q = [Node(None, ROBOT.coordinate, 0, self.distance(ROBOT.coordinate, GOAL), ROBOT.direction)]
        visited = set()
        while Q:
            Top = Pop(Q)
            visited.add(Top.COORDINATE)

            for dir, MOV in enumerate([[0, 1], [0, -1], [1, 0], [-1, 0], [0, 0]]):
                y = MOV[1] + Top.COORDINATE[1]
                x = MOV[0] + Top.COORDINATE[0]
                if dir == 4:
                    Push(Q, Node(Top, Top.COORDINATE, Top.COST + STOP, Top.HEURISTIC, Top.DIRECTION))
                else:
                    if x < 0 or y < 0 or x > self.SIZE - 1 or y > self.SIZE - 1 or self.MAP[y][x] == -1 or (
                    x, y) in visited: continue
                    st = Top.COST + ROTATE + SPEED
                    if dir ^ Top.DIRECTION in (0, 3):
                        st = Top.COST + SPEED

                    if all(map(lambda r: (st < r[0] or st > r[1]), self.TimeTable[y][x])):
                        Heuristic = self.distance( (x, y), GOAL)
                        if Heuristic==0:  # success path find!
                            self.path_tracking(idx, Node(Top, (x, y), st, Heuristic, dir))
                            Q.clear()
                            break
                        else:
                            Push(Q, Node(Top, (x, y), st, Heuristic, dir))


n = int(input())

robots = [Robot((1, 0), 1, 1, 2, 1, 'G'), Robot((0, 0), 1, 1, 2, 1, 'R'), Robot((0, 4), 0, 1, 2, 1, 'B')]
astar = TimeAstar(n, robots, (4, 4),7)
astar.Robot_sort()
# astar.MAP[3][2] = astar.MAP[3][3] = astar.MAP[3][1] = astar.MAP[3][4] = -1
Ob = [(0,0),(0,4),(7,3),(4,0)]
astar.set_obstacle([Ob])
print(np.matrix(astar.MAP))
# for i in range(len(astar.robots)):
#     astar.Search(i)
    # print(astar.robots[i].path)
# for y in range(n):
#     for x in range(n):
        # print(astar.TimeTable[y][x],end='    ')
    # print()

# for i in range(len(robots)):
#     astar.Search(i)

# for i in astar.robots:
#     print(i.path)
'''
5 
0 0 0 0 0    
0 0 0 0 0
0 0 0 0 0
0 0 0 0 0
0 0 0 0 0

'''
