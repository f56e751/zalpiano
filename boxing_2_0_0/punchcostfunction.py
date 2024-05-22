import numpy as np

class CostFunction:
    def calculate(self, distance):
        raise NotImplementedError("Each cost function must implement the 'calculate' method.")


class ExponentialCostFunction(CostFunction):
    def __init__(self, sigma):
        self.sigma = sigma

    def calculate(self, distance):
        return np.exp(-distance**2 / (2 * self.sigma**2))

class LinearCostFunction(CostFunction):
    def calculate(self, distance):
        return 1 - distance


class puncostfunction():
