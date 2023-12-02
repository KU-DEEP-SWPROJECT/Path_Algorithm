from heapq import heappush as Push
from heapq import heappop as Pop
import numpy as np
from math import *
from robot_class import robot as Robot

class Node:
    def __init__(self,parent :object, coordinate:tuple, cost, heuristic,dir):
        self.PARENT = parent
        self.COORDINATE = coordinate
        self.COST = cost
        self.HEURISTIC = heuristic
        self.DIRCTION = dir  # 0b00 : front  0b11 : back 0b01 : left 0b10: right
        
    def __lt__(self, other):
        return (self.COST + self.HEURISTIC) < (other.COST + other.HEURISTIC)
    def __eq__(self, other) -> bool:
        if isinstance(other, Node):
            if self.COORDINATE == other.COORDINATE and self.HEURISTIC == other.HEURISTIC and self.COST == other.COST:
                return True
        return False
    def __hash__(self) -> int:
        return self.COST+self.HEURISTIC


class TimeAstar:

    def __init__(self,SIZE:int,robots:list, goal:tuple) -> None:
        self.SIZE = SIZE
        self.robots = robots.copy()
        self.MAP = [[0]*SIZE for _ in range(SIZE)]
        self.TimeTable = [[[[0,0]  for _ in range(len(robots))] for _ in range(SIZE)] for _ in range(SIZE)]
        self.COST_RATIO = 5
        self.set_goal(goal)

    def set_goal(self,goal:tuple):
        for robot in self.robots:
            robot.goal = goal

    def distance(self,A:list , B:list) -> int: # manathn
        return abs(A[0]-B[0]) + abs(A[1]-B[1])
    
    def put_robots(self,robots:list):
        self.robots = robots.copy()

    def Set_line(self,Ob,p1idx,p2idx):
        p1 = Ob[p1idx]
        p2 = Ob[p2idx]
        if p1[0] >= p2[0] and p1[1] >= p2[1]:
            min_x,max_x = p2[0],p1[0]
            min_y,max_y = p2[1],p2[1]
        elif p1[0] <= p2[0]  and p1[1] >= p2[1]:
            pass
        elif p1[0] >= p2[0] and p1[1] <= p2[1]:
            pass
        else:
            pass
    def Set_obstacle(self,obstacles) -> None:
        for obstacle in obstacles:
            self.Set_line(obstacle,0,1)
            self.Set_line(obstacle,1,2)
            self.Set_line(obstacle,2,3)
            self.Set_line(obstacle,3,0)
            
        

    def CCW(self,A, B):
    # A와 B의 외적 계산
        cross_product = np.cross(A, B)

        if cross_product > 0 : return 0
        elif cross_product < 0 : return 1
        return 2

    def ToCommand(self,idx :int):
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

    def path_tracking(self,idx:int,T_Node : Node) -> None:
        List = []
        Table = self.TimeTable
        while(T_Node.PARENT.COORDINATE is not None):
            y,x = T_Node.COORDINATE
            Table[y][x][idx] = [T_Node.COST - T_Node.PARENT.COST,T_Node.COST]
            print(T_Node.COORDINATE,T_Node.COST)
            List.append((x-T_Node.PARENT.COORDINATE[0],y-T_Node.PARENT.COORDINATE[1]))
            T_Node = T_Node.PARENT
        List.reverse()
        print(List)
        # self.robots[idx].put_path(self.ToCommand(List))
        
    def Robot_sort(self):
        self.robots.sort(key=lambda x : self.distance(x.coordinate,x.goal))

    def Search(self,idx:int) -> None: # Robot Path finding 
        # Heuristic = Distance  // F = G(현재까지 온 거리) + H(맨하튼 거리)
        
        ROBOT = self.robots[idx]
        GOAL = ROBOT.GOAL
        SPEED = ROBOT.STRAIGHT*self.COST_RATIO
        ROTATE = ROBOT.ROTATE*self.COST_RATIO
        STOP = ROBOT.STOP*self.COST_RATIO

        Q = [ Node(None, ROBOT.coordinate , 0, self.distance(ROBOT.coordinate , GOAL), ROBOT.direction)]
        visited = set()
        while Q:
            Top = Pop(Q)
            visited.add(Top.COORDINATE)
            
            for dir,MOV in enumerate([[0,1],[0,-1],[1,0],[-1,0],[0,0]]):
                y = MOV[1] + Top.COORDINATE[1]
                x = MOV[0] + Top.COORDINATE[0]
                if dir == 4:
                    Push(Q, Node(Top,Top.COORDINATE,Top.COST+STOP,Top.HEURISTIC,Top.DIRCTION))
                else:
                    if x < 0 or y < 0 or x > self.SIZE-1 or y > self.SIZE-1 or self.MAP[y][x]== -1 or (x,y) in visited: continue
                    st = Top.COST+ROTATE+SPEED
                    if dir ^ Top.dir in (0,3):
                        st = Top.COST+SPEED
                    if all(map(lambda r: (st < r[0]  or st > r[1]),self.TimeTable[y][x])):
                        if((x,y) == GOAL): #success path find!
                            self.path_tracking(idx,Node(Top,(x,y),st,self.distance((x,y),GOAL),dir))
                        Push(Q, Node(Top,(x,y),st,self.distance((x,y),GOAL),dir))
                    
    

n = int(input())

robots = [Robot((1,0),1,1,2,1,'G'),Robot((0,0),1,1,2,1,'R'),Robot((0,4),0,1,2,1,'B')]
astar = TimeAstar(n, robots,(4,4))
astar.Robot_sort()
astar.MAP[3][2] = astar.MAP[3][3]=astar.MAP[3][1]= astar.MAP[3][4] = -1

for i in range(len(astar.robots)):
    astar.Search(i)
    # print(astar.robots[i].path)
    
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