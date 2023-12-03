class robot:
    def __init__(self,coordinate:tuple,direction:int,straight_speed:int,rotate_speed:int,stop:int,color:str):
        self.coordinate : tuple= coordinate   # initial location
        self.direction : int = direction     # initial direction [00 : front], [11 : back] , [01 : left], [10 : right]
        self.path = None               # robot's path [] 
        self.STRAIGHT = straight_speed
        self.ROTATE = rotate_speed
        self.STOP = stop
        self.GOAL = None
        if color == 'R':
            self.IDX = 1
        elif color == 'G':
            self.IDX = 2
        elif color == 'B':
            self.IDX = 3
        else: 
            self.IDX = 4

    def put_path(self,Path)->None:
        self.path = Path

    def put_speed(self,straight_speed,rotate_speed):
        self.STRAIGHT = straight_speed
        self.ROTATE = rotate_speed

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