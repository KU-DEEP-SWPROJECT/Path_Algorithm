def Robot_sort() -> None:
    RobotArray = [
        [0, 7, 7, 7],
        [1, 2, 7, 4],
        [3, 4, 5, 6],
        [4, 4, 4, 4]
    ]
    Check = [0] * 4

    for i in range(4):
        Max = 0
        for j in range(1, 4):
            if RobotArray[j][i] < RobotArray[Max][i]:
                Max = j
        Check[i] = Max

    print(Check)

Robot_sort()
