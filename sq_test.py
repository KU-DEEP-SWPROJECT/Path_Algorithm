import math
from math import pow, atan2, asin, sqrt, pi
import gunwoo

input_map=[]

input_map = input()
print("input map ")
print(input_map)

class map_parser:
    def __init__(self,map):
        self.map = map
        self.pixel_num = map[0]
        self.robot_array = map[1]
        self.jim_array = map[2]
        
        self.robot_num = len(self.robot_array)
        self.robot = []
        

    def parsing(self):
        pass
    
    def robot_parsing(self):
        for n in self.robot_array:
            self.robot.append(n)
        return robot

    def print_data(self):
        
        print("pixel :%d * %d", self.pixel_num, self.pixel_num)
        
        for bot in self.robot:
            print("robot : ",bot)
        print("jim :",jim_array)

    def check_jim(self):
        line=[]
        for i in range(4):
            a=self.jim_array[i]
            b=self.jim_array[(i+1)%4]
            line.append(sqrt(pow(a[0]-b[0]),pow[a[1]-b[1]))
            print("line length %d",line[i])

        

