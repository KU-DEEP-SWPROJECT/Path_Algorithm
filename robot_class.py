class robot:
    def __init__(self,coordinate:tuple,direction:int,straight_speed:int,rotate_speed:int,stop:int,color:str):
        self.coordinate = coordinate   # initial location
        self.direction = direction     # initial direction [00 : front], [11 : back] , [01 : left], [10 : right]
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
        self.path = str(self.IDX)+":"+Path

    def put_speed(self,straight_speed,rotate_speed):
        self.STRAIGHT = straight_speed
        self.ROTATE = rotate_speed
