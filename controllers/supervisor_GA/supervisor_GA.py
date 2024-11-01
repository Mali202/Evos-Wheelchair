import numpy as np
from GA import Model
from controller import Supervisor
from utils import transform_to_moves


class WheelchairSupervisor:
    def __init__(self):
        # Create the supervisor instance
        self.supervisor = Supervisor()

        # Get the basic time step of the simulation.
        self.time_step = int(self.supervisor.getBasicTimeStep())

        # Initialize the emitter to send data to the robot
        self.emitter = self.supervisor.getEmitter('emitter')

        # Initialize the receiver to receive data from the robot
        self.receiver = self.supervisor.getReceiver('receiver')
        self.receiver.enable(self.time_step)

        # Get the robot node for resetting its position
        self.robot_node = self.supervisor.getFromDef("Wheelchair_Robot")

        # Store the initial position of the robot for resetting purposes
        if self.robot_node:
            self.initial_translation = self.robot_node.getField("translation").getSFVec3f()
            self.initial_rotation = self.robot_node.getField("rotation").getSFRotation()
        else:
            raise ValueError(
                "Supervisor: Could not find robot node. Ensure the robot has a DEF name 'Wheelchair_Robot'.")

        # Initialize the DE model
        self.model = Model(num_params=30, pop_size=10, crossover_rate=0.8, alpha=0.9, mutation_rate=0.1,
                           mutation_magnitude=0.1, tournament_size=3, elitism_rate=0.2)

    def send_moves(self, moves):
        """
        Sends a list of moves to the robot.
        Each move is a dictionary with 'left_pwm', 'right_pwm', and 'duration'.
        """
        # Convert the list of moves to a string for simplicity in communication
        message = str(moves)
        self.emitter.send(message.encode('utf-8'))
        print(f"Supervisor: Sent moves to robot: {message}")

    def wait_for_completion(self):
        """
        Waits for a message from the robot indicating completion of the moves.
        """
        print("Supervisor: Waiting for completion message from robot...")
        while self.supervisor.step(self.time_step) != -1:
            # Check if there's a message available from the robot
            if self.receiver.getQueueLength() > 0:
                message = self.receiver.getString()
                if message == "done":
                    print("Supervisor: Received completion message from robot.")
                    self.receiver.nextPacket()  # Clear the message from the queue
                    break
                self.receiver.nextPacket()  # Clear unrecognized messages from the queue

    def reset_robot(self):
        """
        Resets the robot to its initial position and orientation.
        """
        if self.robot_node:
            print("Supervisor: Resetting robot to initial position.")
            # Reset position
            self.robot_node.getField("translation").setSFVec3f(self.initial_translation)
            # Reset rotation
            self.robot_node.getField("rotation").setSFRotation(self.initial_rotation)
            # Reset the robot's velocity if needed (optional)
            self.robot_node.resetPhysics()
        else:
            print("Supervisor: Robot node not found, cannot reset.")

    def wait_for_fitness(self):
        """
        Waits for a fitness message from the robot.
        Returns the fitness score once received.
        """
        print("Supervisor: Waiting for fitness score from robot...")
        while self.supervisor.step(self.time_step) != -1:
            if self.receiver.getQueueLength() > 0:
                message = self.receiver.getString()
                self.receiver.nextPacket()
                if message.startswith("fitness:"):
                    # Extract fitness score
                    fitness_score = float(message.split(":")[1])
                    print(f"Supervisor: Received fitness score: {fitness_score}")
                    return fitness_score

    def evaluate_population(self):
        """
        Sends each genome in the population to the robot and collects fitness scores.
        """
        fitness_scores = []
        for i in range(self.model.population.shape[0]):
            # Send the individual's moves to the robot
            self.send_moves(transform_to_moves(self.model.population[i]))

            # Wait for the fitness score from the robot
            fitness_score = self.wait_for_fitness()
            fitness_scores.append(fitness_score)

            # Reset robot after each individual evaluation
            self.reset_robot()

        # Assign fitness scores to the population
        self.model.fitness = np.array(fitness_scores)

    def run(self):
        gens = 5
        fitness_results = []
        for gen in range(gens):
            print(f"Supervisor: Evaluating generation {gen}")

            # Evaluate the fitness of the current population
            self.evaluate_population()

            fitness_results.append(np.min(self.model.fitness))
            # Evolve the population
            self.model.evolve_gen()

            print(f"Supervisor: Completed generation {gen}, Best Fitness = {np.min(self.model.fitness)}")

        print(f"Supervisor: Best Fitness Results: {fitness_results}")
        print("Supervisor: Best Individual: ", self.model.population[np.argmin(self.model.fitness)])


# Main function
if __name__ == "__main__":
    # Create the supervisor instance
    wheelchair_supervisor = WheelchairSupervisor()
    # Run the supervisor
    wheelchair_supervisor.run()
