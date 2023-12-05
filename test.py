def find_unique_min_values(matrix):
    unique_min_values = []

    # Find minimum values for each row
    for row in matrix:
        min_value = min(row)
        row_index = matrix.index(row)
        col_index = row.index(min_value)
        unique_min_values.append((row_index, col_index, min_value))
        # Exclude the chosen minimum value from other rows
        for r in matrix:
            r[col_index] = float('inf')  # Exclude from future min calculations

    # Find minimum values for each column
    for col in range(len(matrix[0])):
        column = [row[col] for row in matrix]
        min_value = min(column)
        row_index = column.index(min_value)
        unique_min_values.append((row_index, col, min_value))
        # Exclude the chosen minimum value from other columns
        for r in matrix:
            r[col] = float('inf')  # Exclude from future min calculations

    return unique_min_values


# Example usage with a 4x4 matrix
RobotArray = [
    [3, 7, 2, 9],
    [1, 8, 6, 4],
    [5, 3, 7, 2],
    [8, 7, 1, 6]
]

result = find_unique_min_values(RobotArray)
print(result)
