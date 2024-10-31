import numpy as np


def transform_to_moves(move_array: np.ndarray):
    # Check if the array has the correct size
    if len(move_array) != 30:
        raise ValueError("Array must be of size 30.")

    # Create the list of moves
    moves = []

    # Iterate through the array in steps of 3
    for i in range(0, len(move_array), 3):
        # Extract left PWM, right PWM, and duration from the array
        left_pwm = move_array[i]
        right_pwm = move_array[i + 1]
        duration = move_array[i + 2]

        # Create a move dictionary and append to the moves list
        move = {
            'left_pwm': left_pwm,
            'right_pwm': right_pwm,
            'duration': duration
        }
        moves.append(move)

    return moves
