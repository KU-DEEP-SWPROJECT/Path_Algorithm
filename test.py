
class Node:
    def __init__(self,parent, coordinate, cost, heuristic,dir):
        self.parent = parent
        self.coordinate = coordinate
        self.cost = cost
        self.heuristic = heuristic
        self.dir = dir
    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)
    def __str__(self) -> str:
        return str(self.cost+self.heuristic)
a=  Node(None,[1,2],10,0,0)
b = Node(a,[1,3],1,2,3)
print(b)
print(b.parent)
