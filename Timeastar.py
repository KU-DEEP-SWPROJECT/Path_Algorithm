from heapq import heappush as Push
from heapq import heappop as Pop
import numpy as np
from math import *
from robot_class import robot as Robot

class Node:
    def __init__(self,parent, coordinate, cost, heuristic,dir):
        self.parent = parent
        self.coordinate = coordinate
        self.cost = cost
        self.heuristic = heuristic
        self.dir = dir
        
    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)
    def __eq__(self, other) -> bool:
        if isinstance(other, Node):
            if self.coordinate == other.coordinate and self.heuristic == other.heuristic and self.cost == other.cost:
                return True
        return False
    def __hash__(self) -> int:
        return self.cost+self.heuristic


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
        # print(A,B,"= ",cross_product)

        # 외적의 부호에 따라 시계방향 또는 반시계방향 여부 판단
        if cross_product > 0 : return 0
        elif cross_product < 0 : return 1
        return 2

    def lightweight(self,vector_list:list):
        i = 0
        modified_list = []
        command_list = []
        dir = { 
            (0,0) : 'S',
            (0,1) : 'F', 
            (1,0) : 'F',
            (0,-1) : 'B',
            (-1,0) : 'B',
            }

        while i < len(vector_list) - 1:
            j = i + 1
            while j < len(vector_list) and vector_list[i] == vector_list[j]:
                j += 1

            ccw = self.CCW(modified_list[-1], vector_list[j - 1]) if modified_list else None
            R = ("R90", "R-90", "")[ccw] if ccw is not None else ""
            if R:
                command_list.append(R)

            if vector_list[i] != (0, 0):
                modified_list.append(vector_list[i])
            command_list.append(dir[vector_list[i]] + str(j - i))
            i = j

        else:
            if i < len(vector_list):
                command_list.append(dir[tuple(vector_list[i])] + "1")

        return '/'.join(command_list)

    def path_tracking(self,idx:int,T_Node : Node) -> None:
        List = []
        Table = self.TimeTable
        while(T_Node.parent is not None):
            y,x = T_Node.coordinate
            Table[y][x][idx] = [T_Node.cost - T_Node.parent.cost,T_Node.cost]
            print(T_Node.coordinate,T_Node.cost)
            List.append((x-T_Node.parent.coordinate[1],y-T_Node.parent.coordinate[0]))
            T_Node = T_Node.parent
        List.reverse()
        print(List)
        # self.robots[idx].put_path(self.lightweight(List))
        
    def Robot_sort(self):
        self.robots.sort(key=lambda x : self.distance(x.coordinate,x.goal))

    def Search(self,idx:int) -> None: # Robot Path finding 
        # Heuristic = Distance  // F = G(현재까지 온 거리) + H(맨하튼 거리)
        
        robot = self.robots[idx]
        GOAL = robot.GOAL
        SPEED = robot.straight*self.COST_RATIO
        ROTATE = robot.rotate*self.COST_RATIO
        STOP = robot.stop*self.COST_RATIO

        Q = [ Node(None, robot.coordinate , 0, self.distance(robot.coordinate , GOAL), robot.direction)]
        visited = set()
        while Q:
            Top = Pop(Q)
            visited.add(Top.coordinate)
            
            for dir,ku in enumerate([[0,1],[0,-1],[1,0],[-1,0],[0,0]]):
                y = ku[1] + Top.coordinate[1]
                x = ku[0] + Top.coordinate[0]
                if dir == 4:
                    Push(Q, Node(Top,Top.coordinate,Top.cost+STOP,Top.heuristic,Top.dir) )
                else:
                    if x < 0 or y < 0 or x > self.SIZE-1 or y > self.SIZE-1 or self.MAP[y][x]== -1 or (x,y) in visited: continue
                    st = Top.cost+ROTATE+SPEED
                    if dir ^ Top.dir in (0,3):
                        st = Top.cost+SPEED
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