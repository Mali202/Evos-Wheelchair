import ast
from controller import Robot
import math


class WheelchairRobot:
    def __init__(self):
        # Create the robot instance
        self.robot = Robot()

        # Get the basic time step of the simulation.
        self.time_step = int(self.robot.getBasicTimeStep())

        # Initialize motors
        self.left_motor = self.robot.getMotor('left motor')
        self.right_motor = self.robot.getMotor('right motor')

        # Set the motors to velocity control mode
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        # Initialize the emitter and receiver to communicate with the supervisor
        self.emitter = self.robot.getEmitter('emitter')

        self.receiver = self.robot.getReceiver('receiver')
        self.receiver.enable(self.time_step)

        # Initialize pairs of distance sensors
        self.distance_sensors = []
        self.distance_sensors.append({'x': self.robot.getDistanceSensor('l1'), 'y': self.robot.getDistanceSensor('l4')})
        self.distance_sensors.append({'x': self.robot.getDistanceSensor('l2'), 'y': self.robot.getDistanceSensor('l3')})
        self.distance_sensors.append({'x': self.robot.getDistanceSensor('r1'), 'y': self.robot.getDistanceSensor('r4')})
        self.distance_sensors.append({'x': self.robot.getDistanceSensor('r2'), 'y': self.robot.getDistanceSensor('r4')})

    def execute_moves(self, moves):
        """
        Execute a series of moves provided by the supervisor.
        Each move is a dictionary with 'left_pwm', 'right_pwm', and 'duration'.
        """
        for move in moves:
            left_pwm = move['left_pwm']
            right_pwm = move['right_pwm']
            duration = move['duration']

            # Convert PWM to a velocity value suitable for Webots (may require calibration)
            left_velocity = left_pwm / 3.3  # Scale the pwm to a velocity value.
            right_velocity = right_pwm / 3.3  # Scale the pwm to a velocity value.

            # Set motor velocities
            self.left_motor.setVelocity(left_velocity)
            self.right_motor.setVelocity(right_velocity)

            # Calculate the duration in time steps for Webots
            steps = int(duration * 1000 / self.time_step)

            # Run the robot for the specified duration
            for _ in range(steps):
                if self.robot.step(self.time_step) == -1:
                    break

        # Stop the motors after finishing the moves
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        # Notify the supervisor that the movements are complete
        self.notify_supervisor()

    def wait_for_moves(self):
        """
        Waits for a message from the supervisor containing the list of moves.
        """
        print("Robot: Waiting for moves from the supervisor...")
        while self.robot.step(self.time_step) != -1:
            # Check if there's a message available from the supervisor
            if self.receiver.getQueueLength() > 0:
                message = self.receiver.getString()
                self.receiver.nextPacket()  # Clear the message from the queue
                print(f"Robot: Received moves: {message}")

                # Convert the message string back into a list of moves
                try:
                    _moves = ast.literal_eval(message)
                    return _moves
                except (ValueError, SyntaxError) as e:
                    print(f"Robot: Error parsing moves: {e}")

        return None

    def notify_supervisor(self):
        """
        Calculates the fitness based on the distance from start to end position.
        Sends the fitness score to the supervisor.
        """
        # Send fitness score to supervisor
        fitness_score = self.fitness_function()
        message = f"fitness:{fitness_score}"
        self.emitter.send(message.encode('utf-8'))
        print(f"Robot: Sent fitness score to supervisor: {fitness_score}")

    def fitness_function(self):
        return 0


# Main function
if __name__ == "__main__":
    # Create the robot instance
    wheelchair_robot = WheelchairRobot()

    # Wait for the list of moves from the supervisor
    _moves = wheelchair_robot.wait_for_moves()

    # If moves are successfully received, execute them
    if _moves:
        wheelchair_robot.execute_moves(_moves)


def calculate_altitude(a, b):
    # Calculate hypotenuse c using Pythagorean theorem
    c = math.sqrt(a * 2 + b * 2)

    # Calculate area of the triangle
    area = (a * b) / 2

    # Calculate altitude h with area = 1/2 * base * height
    h = (2 * area) / c
    return h
