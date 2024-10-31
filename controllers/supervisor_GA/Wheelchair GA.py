import random

POPULATION_SIZE = 50
GENERATIONS = 100
MUTATION_RATE = 0.08


class Individual:
    def __init__(self):
        self.PWM1 = 0
        self.PWM2 = 0
        self.Duration = 0
        self.Fitness = 0.0


def initialize_population():
    population = []
    for _ in range(POPULATION_SIZE):
        individual = Individual()
        individual.PWM1 = random.randint(0, 255)
        individual.PWM2 = random.randint(0, 255)
        individual.Duration = random.randint(1, 100)
        population.append(individual)
    return population


def evaluate_fitness(population):
    for individual in population:
        individual.Fitness = simulate_move(individual)


def simulate_move(individual):
    target_distance = 100.0
    max_pwm = 255.0

    average_pwm = (individual.PWM1 + individual.PWM2) / 2
    speed = average_pwm / max_pwm
    distance_traveled = speed * individual.Duration

    remaining_distance_to_door = max(0, target_distance - distance_traveled)
    return remaining_distance_to_door


def select_next_generation(population):
    next_generation = []
    population.sort(key=lambda x: x.Fitness, reverse=True)
    half_population = len(population) // 2

    next_generation.extend(population[:half_population])

    while len(next_generation) < POPULATION_SIZE:
        parent1 = random.choice(next_generation[:half_population])
        parent2 = random.choice(next_generation[:half_population])
        next_generation.append(crossover(parent1, parent2))

    return next_generation


def crossover(parent1, parent2):
    offspring = Individual()
    offspring.PWM1 = (parent1.PWM1 + parent2.PWM1) // 2
    offspring.PWM2 = (parent1.PWM2 + parent2.PWM2) // 2
    offspring.Duration = (parent1.Duration + parent2.Duration) // 2
    return offspring


def mutate(population):
    for individual in population:
        if random.random() < MUTATION_RATE:
            individual.PWM1 = max(0, min(255, individual.PWM1 + random.randint(-10, 10)))
            individual.PWM2 = max(0, min(255, individual.PWM2 + random.randint(-10, 10)))
            individual.Duration = max(1, min(100, individual.Duration + random.randint(-5, 5)))


def get_best_individual(population):
    return max(population, key=lambda x: x.Fitness)


def main():
    population = initialize_population()

    for generation in range(GENERATIONS):
        evaluate_fitness(population)

        best = get_best_individual(population)
        print(
            f"Generation {generation + 1}: Best Move - PWM1: {best.PWM1}, PWM2: {best.PWM2}, Duration: {best.Duration}, Fitness: {best.Fitness}")

        population = select_next_generation(population)
        mutate(population)

    final_best = get_best_individual(population)
    print(f"Best Move: PWM1: {final_best.PWM1}, PWM2: {final_best.PWM2}, Duration: {final_best.Duration}")


if __name__ == "__main__":
    main()

