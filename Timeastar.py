from collections import deque
from heapq import heappush as Push
from heapq import heappop as Pop
import numpy as np
from math import *
from typing import Optional
import copy
from robot_class import robot as Robot
# import matplotlib.pyplot as plt
import time

class Node:
    def __init__(self, parent, coordinate: tuple, cost: int, heuristic: int, dir: int):
        self.PARENT : Node = parent
        self.COORDINATE = tuple(coordinate)
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
    # Size, Radius , robots( center coordinate:tuple , direction:int,straight_speed:int,rotate_speed:int, stop:int, color:str)
    # , goal: ㄷ자 , Obstacle [ [ㄷ자],[ㄷ자]]
    def __init__(self, SIZE: int, Radius: int, robots: list, goal: list, obstacles: list) -> None:
        self.SIZE = SIZE
        self.robots : Robot= robots.copy()
        self.CONST_TRANSFER = 2 / SIZE
        self.MAP = [[0] * SIZE for _ in range(SIZE)]
        self.COST_RATIO = 5
        self.RANGE = Radius * Radius
        self.AgentTable = [[] for _ in range(len(robots))]  # [ [], [], [], [], [] ]

        L = [*map(tuple,goal)]
        min_x,max_x = min(L)[0],max(L)[0]
        min_y,max_y = min(L,key= lambda x : x[1])[1],max(L,key= lambda x : x[1])[1],
        goal = [(min_x,max_y),(max_x,max_y),(max_x,min_y),(min_x,min_y)]
        self.set_obstacle([goal])
        xsize = max_x - min_x
        ysize = max_y - min_y
        dx = xsize//8
        dy = ysize//8

        ggoal = [(min_x + dx, max_y + 3 * dy), (max_x - dx, max_y + 3 * dy), (max_x - dx, min_y - 6 * dy), (min_x + dx, min_y - 6 * dy)]
        goal = [(L[0][0], ggoal[0][1] - 1), (L[1][0], ggoal[1][1] - 1), (L[2][0], ggoal[2][1] + 1), (L[3][0], ggoal[3][1] - 1)]
        self.init_Goal(ggoal)
        self.set_obstacle([goal])
        self.set_obstacle(obstacles)

        self.Robot_sort()
        for i in range(len(robots)):
            self.AgentTable[i].append(self.robots[i].coordinate)




    def set_goal(self, goal: tuple):
        for robot in self.robots:
            robot.GOAL = goal
    def draw_line2(self,x0,y0,x1,y1):
        if x1 < x0: x0,x1 = x1,x0
        if y1 < y0: y0,y1 = y1,y0

        if x0==x1:
            for y in range(y0,y1+1):
                self.MAP[y][x0] = -1
        elif y0==y1:
            for x in range(x0, x1 + 1):
                self.MAP[y0][x] = -1
    def set_obstacle(self, obstacles) -> None:
        for obstacle in obstacles:
            self.draw_line2(obstacle[0][0], obstacle[0][1], obstacle[1][0], obstacle[1][1])
            self.draw_line2(obstacle[1][0], obstacle[1][1], obstacle[2][0], obstacle[2][1])
            self.draw_line2(obstacle[2][0], obstacle[2][1], obstacle[3][0], obstacle[3][1])
            self.draw_line2(obstacle[3][0], obstacle[3][1], obstacle[0][0], obstacle[0][1])

    @staticmethod
    def distance(A: tuple, B: tuple) -> int:  # 맨하튼 거리
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
            self.MAP[y0][x0] = -1
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def Robot_sort(self):
        self.robots.sort(key=lambda x : self.distance(x.coordinate,x.GOAL))
    def init_Goal(self,List: list) -> None:
        if not isinstance(List,list): List = List.tolist()
        RobotArray = []
        a = set()
        b = set()
        for i in range(len(self.robots)):
            coo = self.robots[i].coordinate
            for j in List:
                RobotArray.append((self.distance(coo,j),i,tuple(j)))
        RobotArray.sort(key= lambda x: x[0])
        for dis,x,y in RobotArray:
            if x in a or y in b: continue
            a.add(x)
            b.add(y)
            self.robots[x].GOAL = y


    def is_Range(self, A: tuple, B: tuple):
        dy = B[1] - A[1]
        dx = B[0] - A[0]
        return (dy * dy + dx * dx) <= self.RANGE

    def is_Wait(self, A: tuple, B: tuple):
        dy = B[1] - A[1]
        dx = B[0] - A[0]
        return (dy * dy + dx * dx) <= self.RANGE+10

    def draw_path(self,idx):
        MAP = copy.deepcopy(self.MAP)
        for i in self.robots[idx].path:
            x,y = i[1]
            MAP[y][x] = '●'
        for y in range(100):
            print(MAP[y])
    @staticmethod
    def Arrow(a,b)-> str:
        if a in [0,3]:
            return ("R90,3","R-90,3")[0 if b==1 else 1]
        else:
            return ("R90,3","R-90,3")[0 if b==2 else 1]

    def explore_cross(self , T):
        sx, sy = T.COORDINATE
        dir = T.DIRECTION
        Q = deque()
        Q.append((0, sx, sy + 1))
        Q.append((3, sx, sy - 1))

        while Q:
            d, x, y = Q.popleft()
            if self.MAP[y][x] == -1:
                break
            if d == 1:
                if x - 1 < 0: continue
                Q.append((1, x - 1, y))
            elif d == 2:
                if x + 1 > self.SIZE-1: continue
                Q.append((2, x + 1, y))
            elif d == 3:
                if y - 1 < 0: continue
                Q.append((3, x, y - 1))
            else:
                if y + 1 > self.SIZE-1: continue
                Q.append((0, x, y + 1))
        if dir ^ d == 3:
            return d,'R180,0'
        return d,('R90,3', 'R-90,3')[(dir ^ d == 1) if dir in [0, 3] else (dir ^ d == 2)]


    def ToCommand(self,idx):
        tmp = 12
        command_list = []
        path = self.robots[idx].Direction_path
        realpath = self.robots[idx].path
        cnt, stopcnt = 0,0
        cur = path[0] # 첫번째 방향
        cur_path = realpath[0][1] # 첫번째 좌표
        fleg = True # 반전 있는지
        CONST = self.CONST_TRANSFER
        for i in range(1,len(path)):
            if cur_path== realpath[i][1]:
                if cnt > 0:
                    command_list.append('F'+str((CONST,-CONST)[0 if fleg else 1]*cnt)+','+str((CONST,-CONST)[0 if fleg else 1]*cnt*tmp))
                    stopcnt = 0
                    cnt = 0
                stopcnt += 1
            else:
                if stopcnt > 0:
                    command_list.append('S' + str(stopcnt))
                    cnt = 0
                    stopcnt = 0
                if cur == path[i]:
                    pass
                elif cur ^ path[i] == 3: # 반대 방향
                    if cnt > 0: command_list.append('F' + str((CONST, -CONST)[0 if fleg else 1] * cnt)+','+str((CONST, -CONST)[0 if fleg else 1] * cnt*tmp))
                    cnt = 1
                    fleg ^= True
                else: # cur ^ path[i] == 1 or == 2
                    if cnt > 0: command_list.append('F' + str((CONST, -CONST)[0 if fleg else 1] * cnt)+','+str((CONST, -CONST)[0 if fleg else 1] * cnt*tmp))
                    command_list.append(self.Arrow(cur,cur ^ path[i]))
                    cnt = 1
                cnt+=1
            cur,cur_path = path[i] , realpath[i][1]

        if cnt > 1:
            command_list.append('F' + str((-CONST, CONST)[1 if fleg else 0] * (cnt-1))+','+str((-CONST, CONST)[1 if fleg else 0] * (cnt-1)*tmp))
        command_list.append(self.robots[idx].last[1])
        return str(self.robots[idx].IDX)+":"+'/'.join(command_list)
    def path_tracking(self, idx: int, T_Node: Node) -> None:
        List = []
        Direction_List = []
        d,last = self.explore_cross(T_Node)
        self.robots[idx].last= (d,last)
        while T_Node.PARENT is not None:
            List.append((T_Node.COST//self.COST_RATIO, T_Node.COORDINATE))
            Direction_List.append(T_Node.DIRECTION)
            T_Node = T_Node.PARENT
        List.append((T_Node.COST // self.COST_RATIO, T_Node.COORDINATE))
        Direction_List.append(T_Node.DIRECTION)
        Direction_List.reverse()
        List.reverse()
        self.robots[idx].put_path(List)
        self.robots[idx].put_direction_path(Direction_List)

        j = 1
        for L in List:
            while j <= L[0]:
                self.AgentTable[idx].append(L[1])
                j+=1
        # self.draw_path(idx)

    def Search(self, idx: int) -> None:  # Robot Path finding
        # Heuristic = Distance  // F = G(현재까지 온 거리) + H(맨하튼 거리)

        ROBOT : Robot = self.robots[idx]
        GOAL : tuple = ROBOT.GOAL
        SPEED : int = ROBOT.STRAIGHT * self.COST_RATIO
        ROTATE : int = ROBOT.ROTATE * self.COST_RATIO
        STOP : int = ROBOT.STOP * self.COST_RATIO
        Q = [Node(parent=None, coordinate=ROBOT.coordinate, cost=0, heuristic=self.distance(ROBOT.coordinate, GOAL), dir=ROBOT.direction)]
        visited = set()
        cnt = 0
        while Q:
            Top : Node = Pop(Q)
            # print(Top.COORDINATE,"Cost: ",Top.COST,"Heurisitc: ",Top.HEURISTIC)
            visited.add(Top.COORDINATE)

            for dir, MOV in enumerate([[0,1],[1,0],[-1,0],[0,-1],[0, 0]]):
                x,y = MOV[0] + Top.COORDINATE[0] , MOV[1] + Top.COORDINATE[1]

                if dir == 4:
                    fleg = True
                    for i in range(len(self.robots)):  # 로봇의 개수만큼
                        if len(self.AgentTable[i]) <= st: continue
                        if self.is_Wait(self.AgentTable[i][st], (x, y)):
                            fleg = False
                    if not fleg:
                        Push(Q,Node(parent=Top, coordinate=Top.COORDINATE, cost=Top.COST + STOP, heuristic=Top.HEURISTIC,dir=Top.DIRECTION))

                else:
                    if x < 0 or y < 0 or x > self.SIZE - 1 or y > self.SIZE - 1 or self.MAP[y][x] == -1 or (x, y) in visited: continue
                    st = Top.COST + ROTATE + SPEED
                    if dir ^ Top.DIRECTION in (0, 3): # 같은 방향을 바라보거나 , 뒤로 가는 방향이라면,
                        st -= ROTATE

                    fleg= True

                    for i in range(len(self.robots)): # 로봇의 개수만큼
                        if i==idx: continue
                        if self.is_Range(self.AgentTable[i][min(st, len(self.AgentTable[i])-1)],(x,y)):
                            fleg = False
                    if fleg:

                        Heuristic: int = self.distance((x, y), GOAL)

                        if Heuristic == 2:  # success path find!

                            self.path_tracking(idx, Node(parent=Top,coordinate= (x, y), cost=st, heuristic=Heuristic, dir=dir))
                            Q.clear()
                            break
                        else:
                            Push(Q, Node(parent=Top, coordinate=(x, y), cost=st, heuristic=Heuristic, dir=dir))

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    n = 100
    obstacles = []
    robots = [ Robot((33, 12), 0, 1, 2, 1, 'G'), Robot((47, 12), 0, 1, 2, 1, 'R'), Robot((7, 11),0, 1, 2, 1, 'B'), Robot((20, 12), 0, 1, 2, 1, 'P')]
    astar = TimeAstar( SIZE=n,Radius=7 ,robots=robots, goal= np.array(((13,80),(34,79),(34,59),(13,59))), obstacles=obstacles)

    ob = []
    for y in range(self.SIZE):
        for x in range(self.SIZE):
            if self.MAP[y][x] == -1:
                ob.append((x,y))
    robot1,robot2,robot3,robot4 = [],[],[],[]
    if astar.robots[0].path is not None: robot1 = [t for _,t in astar.robots[0].path]
    if astar.robots[1].path is not None: robot2 = [t for _,t in astar.robots[1].path]
    if astar.robots[2].path is not None: robot3 = [t for _,t in astar.robots[2].path]
    if astar.robots[3].path is not None: robot4 = [t for _,t in astar.robots[3].path]
    x = [x for x,y in ob]
    y = [y for x,y in ob]
    plt.scatter(x,y,c='gray',edgecolors='none',s=3)
    if len(robot1):
        x = [x for x,y in robot1]
        y = [y for x,y in robot1]
        plt.scatter(x,y,c='blue',edgecolors='none',s=3)
    if len(robot2):
        x = [x for x, y in robot2]
        y = [y for x, y in robot2]
        plt.scatter(x,y,c='purple',edgecolors='none',s=3)
    if len(robot3):
        x = [x for x, y in robot3]
        y = [y for x, y in robot3]
        plt.scatter(x,y,c='orange',edgecolors='none',s=3)
    if len(robot4):
        x = [x for x, y in robot4]
        y = [y for x, y in robot4]
        plt.scatter(x,y,c='red',edgecolors='none',s=3)
    plt.show()

# robots = [ Robot((41, 12), 0, 1, 2, 1, 'G'), Robot((56, 11), 0, 1, 2, 1, 'R'), Robot((26, 12),0, 1, 2, 1, 'B'), Robot((13, 11), 0, 1, 2, 1, 'P')]
# astar = TimeAstar( SIZE=n,Radius=7 ,robots=robots, goal= [(10,77),(29,76),(29,56),(10,57)], obstacles=obstacles)
# astar.Robot_sort()
# print(np.matrix(astar.MAP))
    start=  time.time()
    for i in range(4):
        astar.Search(i)
        print(astar.ToCommand(i))

    print(time.time()-start)

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