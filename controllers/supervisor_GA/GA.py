import numpy
import numpy as np


class Model:

    def __init__(self, num_params: int, pop_size: int, **params):
        self.population = initPopulation(pop_size, num_params)
        self.crossover_rate = params['crossover_rate']
        self.alpha = params['alpha']
        self.mutation_rate = params['mutation_rate']
        self.mutation_magnitude = params['mutation_magnitude']
        self.tournament_size = params['tournament_size']
        self.elitism_rate = params['elitism_rate']

        self.fitness = np.zeros(pop_size, dtype=np.float64)

    def evolve_gen(self):
        new_population = np.empty((0, self.population.shape[1]))
        num_elites = int(self.elitism_rate * self.population.shape[0])
        elites = self.population[np.argsort(self.fitness)[:num_elites]]
        new_population = numpy.vstack((new_population, elites))

        while new_population.shape[0] < self.population.shape[0]:
            parent1 = self.tournament_selection(self.tournament_size)
            parent2 = self.tournament_selection(self.tournament_size)

            # Crossover
            if np.random.rand() < self.crossover_rate:
                child1, child2 = self.blend_crossover(parent1, parent2)
            else:
                child1, child2 = parent1, parent2

            # Mutation
            if np.random.rand() < self.mutation_rate:
                child1 = self.gaussian_mutation(child1)
            if np.random.rand() < self.mutation_rate:
                child2 = self.gaussian_mutation(child2)

            # Add children to new population
            new_population = np.vstack((new_population, child1))
            new_population = np.vstack((new_population, child2))
        self.population = new_population

    def tournament_selection(self, tournament_size: int):
        tournament = np.random.choice(self.fitness, tournament_size, replace=False)
        winner_index = np.argmin(tournament)
        winner = self.population[winner_index]
        return winner

    def gaussian_mutation(self, individual):
        for i in range(individual.shape[0]):
            mutation_value = np.random.normal(0, self.mutation_magnitude)
            if np.random.rand() < 0.5:
                individual[i] += mutation_value
            else:
                individual[i] -= mutation_value

        return individual

    def blend_crossover(self, parent1, parent2):
        child1 = np.zeros(parent1.shape)
        child2 = np.zeros(parent2.shape)

        for i in range(parent1.shape[0]):
            lower_bound = min(parent1[i], parent2[i])
            upper_bound = max(parent1[i], parent2[i])

            range_val = upper_bound - lower_bound
            low = lower_bound - self.alpha * range_val
            high = upper_bound + self.alpha * range_val

            child1[i] = np.random.uniform(low, high)
            child2[i] = np.random.uniform(low, high)

        return child1, child2


def initPopulation(size, num_params):
    guess = np.loadtxt('guess.txt')
    init = np.zeros((size, num_params))

    # Initialize PWM and duration values within specified ranges
    for individual in init:
        for i in range(0, num_params, 3):  # Left PWM in triplet
            individual[i] = np.random.uniform(-10, 10)
        for i in range(1, num_params, 3):  # Right PWM in triplet
            individual[i] = np.random.uniform(-10, 10)
        for i in range(2, num_params, 3):  # Duration in triplet
            individual[i] = np.random.uniform(0, 5)

    init[0] = guess
    return init
