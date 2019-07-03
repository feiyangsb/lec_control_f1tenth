import numpy as np

class InputPreprocessor():
    def __call__(self, state):
        state_normalized = map(lambda x: x/100.0, state)
        s = np.hstack(state_normalized)
        return s