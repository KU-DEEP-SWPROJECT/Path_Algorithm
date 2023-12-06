from Timeastar import TimeAstar
from robot_class import robot
import matplotlib.pyplot as plt
BOT_DIR = 0b00
BOT_LINENAR_TIME = 1
BOT_ROTATE_TIME = 25
BOT_STOP_TIME = 1
BOT_COLOR = "RGBP"

def test_algorithm(
    board_size,
    bot_radius,
    bot_start_point,
    bot_target_point
):
    robots = []
    for i in range(len(bot_start_point)):
        robots.append(robot(
            bot_start_point[i],
            BOT_DIR,BOT_LINENAR_TIME,
            BOT_ROTATE_TIME,
            BOT_STOP_TIME,
            BOT_COLOR[i]
        ))
    
    astar = TimeAstar(SIZE=board_size, Radius=bot_radius, robots=robots, goal=bot_target_point, obstacles=[])
    # for i in range(len(robots)):
    #     astar.Search(i)
    #     print(i, 'Complete')
    # for i in range(len(astar.robots)):
    #     print("GOAL : ",astar.robots[i].GOAL,"START: ",astar.robots[i].coordinate)
    # for i in range(len(astar.robots)):
    #     print(astar.ToCommand(i))
    # print(astar.MAP)
    # for i in range(4):
    #     x,y = astar.robots[i].GOAL
    #     astar.MAP[y][x] = '◆'
    # for y in range(board_size):
    #     print(astar.MAP[y])
    for i in range(4):
        astar.Search(i)
        if(astar.robots[i].path is not None):
            print(astar.ToCommand(i))
        else:
            print(i,"길을 못찾앗습니다.")

    ob = []
    for y in range(board_size):
        for x in range(board_size):
            if astar.MAP[y][x] == -1:
                ob.append((x, y))
    robot1, robot2, robot3, robot4 = [], [], [], []
    if astar.robots[0].path is not None: robot1 = [t for _, t in astar.robots[0].path]
    if astar.robots[1].path is not None: robot2 = [t for _, t in astar.robots[1].path]
    if astar.robots[2].path is not None: robot3 = [t for _, t in astar.robots[2].path]
    if astar.robots[3].path is not None: robot4 = [t for _, t in astar.robots[3].path]
    x = [x for x, y in ob]
    y = [y for x, y in ob]
    plt.scatter(x, y, c='gray', edgecolors='none', s=10)
    if len(robot1):
        x = [x for x, y in robot1]
        y = [y for x, y in robot1]
        plt.scatter(x, y, c='blue', edgecolors='none', s=10)
    if len(robot2):
        x = [x for x, y in robot2]
        y = [y for x, y in robot2]
        plt.scatter(x, y, c='purple', edgecolors='none', s=10)
    if len(robot3):
        x = [x for x, y in robot3]
        y = [y for x, y in robot3]
        plt.scatter(x, y, c='orange', edgecolors='none', s=10)
    if len(robot4):
        x = [x for x, y in robot4]
        y = [y for x, y in robot4]
        plt.scatter(x, y, c='red', edgecolors='none', s=10)
    plt.show()
