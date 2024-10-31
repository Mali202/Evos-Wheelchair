import numpy as np


# Differential Evolution Algorithm to solve diet problem based on best/1/bin strategy
class Model:
    def __init__(self, num_params: int, pop_size: int, init_set, **params):
        self.population = initPopulationWithSet(pop_size, num_params, init_set)
        self.scaling_factor = params['scaling_factor']
        self.recombination_probability = params['recombination_probability']
        self.fitness = np.zeros(pop_size, dtype=np.float64)

    def eval_fitness(self, fitness_func):
        for i in range(self.population.shape[0]):
            self.fitness[i] = fitness_func(self.population[i])

    def evolve_gen(self, fitness_func):
        new_population = np.empty((0, self.population.shape[1]))
        best_solution_index = np.argmin(self.fitness)
        best_solution = self.population[best_solution_index]
        for i in range(self.population.shape[0]):
            mutant_vector = self.construct_mutant_vector(best_solution)
            trial_vector = self.construct_trial_vector(self.population[i], mutant_vector)
            trial_fitness = fitness_func(trial_vector)
            if trial_fitness < self.fitness[i]:
                new_population = np.vstack((new_population, trial_vector))
                if trial_fitness < self.fitness[best_solution_index]:
                    best_solution = trial_vector
            else:
                new_population = np.vstack((new_population, self.population[i]))

        self.population = new_population

    # Construct the mutant vector for the target
    def construct_mutant_vector(self, best):
        x0 = self.population[np.random.choice(self.population.shape[0])]
        x1 = self.population[np.random.choice(self.population.shape[0])]
        mutant_vector = best + self.scaling_factor * (x0 - x1)

        # ensure all parameters are within bounds
        for i in range(mutant_vector.shape[0]):
            if mutant_vector[i] < 0:
                mutant_vector[i] = 0
        return mutant_vector

    # Construct the trial vector for the target
    def construct_trial_vector(self, target, mutant):
        trial_vector = np.zeros(target.shape)
        for i in range(target.shape[0]):
            if np.random.rand() < self.recombination_probability:
                trial_vector[i] = mutant[i]
            else:
                trial_vector[i] = target[i]

        return trial_vector


def initPopulationWithSet(size, num_params, init_set):
    init_population = np.zeros((size, num_params))
    for i in range(size):
        init_population[i] = [np.random.choice(init_set) for _ in range(num_params)]

    return init_population
