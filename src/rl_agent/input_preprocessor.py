import numpy as np

class InputPreprocessor():
    def __call__(self, state):
        #state_normalized = map(lambda x: x/10.0, state)
        s = np.hstack(state)
        s = np.true_divide(s, 10.0)
        s = np.clip(s, 0.01, 1.0)
        temp = np.isinf(s)
        a = np.argwhere(temp==True)
        if a.size != 0:
            print "a:{}, state:{}".format(a, state[a[0][0]])
        return s