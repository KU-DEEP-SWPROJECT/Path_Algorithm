def group_coordinates(direction_list, path_list):
    result = []
    current_direction = None
    current_position = None
    count = 0

    for direction, position in zip(direction_list, path_list):
        if current_direction is None or current_position is None:
            current_direction = direction
            current_position = position
            count = 1
        elif current_direction == direction:
            if current_position == position:
                count += 1
            else:
                if count == 1:
                    result.append(f'S{current_direction}')
                else:
                    result.append(f'F{count}')
                if direction == 1:
                    result.append(f'R{90 if position[0] > current_position[0] else -90}')
                elif direction == 2:
                    result.append(f'R{90 if position[0] < current_position[0] else -90}')
                elif direction == 3:
                    result.append(f'R90' if position[1] < current_position[1] else 'R-90')
                current_direction = direction
                current_position = position
                count = 1

    if count == 1:
        result.append(f'S{current_direction}')
    else:
        result.append(f'F{count}')
    if current_direction != direction:
        result.append(f'R{90 if direction == 2 else -90}')

    return result

# Example usage:
direction_list = [1, 1, 1, 1, 2]
path_list = [(0, 0), (0, 0), (0, 1), (0, 2), (1, 2)]
result = group_coordinates(direction_list, path_list)
print(result)
