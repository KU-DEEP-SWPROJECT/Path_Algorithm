from collections import deque
def explore_cross(start):
    sx,sy = start
    Q = deque()
    Q.append((0,sx,sy+1))
    Q.append((1, sx - 1, sy))
    Q.append((2, sx + 1, sy))
    Q.append((3, sx, sy - 1))
    while Q:
        d,x,y = Q.popleft()
        if input_matrix[y][x] == -1:
            break
        if d==1:
            if x - 1 < 0: continue
            Q.append((1, x-1, y ))
        elif d==2:
            if x + 1 > 4: continue
            Q.append((2, x+1, y))
        elif d==3:
            if y-1 < 0: continue
            Q.append((3,x,y-1))
        else:
            if y+1 > 4: continue
            Q.append((0,x,y+1))
    print(d)

# 테스트용 입력 행렬
input_matrix = [
    [0, 0, -1, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 1, -1, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
]

explore_cross((2,2))
for i in range(5):
    print(input_matrix[i])