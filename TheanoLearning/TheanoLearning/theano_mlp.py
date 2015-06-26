import numpy as np
#import matplotlib.pyplot as plt
import theano
# By convention, the tensor submodule is loaded as T
import theano.tensor as T

class layer(object):
    def __init__(self, W_init, b_init, activation):

        [n_output, n_input] = W_init.shape
        assert b_init.shape == (n_output,1) or b_init.shape == (n_output,)
        self.W = theano.shared(value = W_init.astype(theano.config.floatX),
                               name = 'W',
                               borrow = True)
        self.b = theano.shared(value = b_init.reshape(n_output,1).astype(theano.config.floatX),
                               name = 'b',
                               borrow = True,
                               broadcastable=(False, True))
        self.activation = activation
        self.params = [self.W, self.b]
        #return super(layer, self).__init__(*args, **kwargs)
    def output(self, x):
        lin_output = T.dot(self.W, x) + self.b
        if self.activation is not None:
            non_lin_output = self.activation(lin_output)
        return ( lin_output if self.activation is None else non_lin_output )


W_init = np.ones([3,3])
b_init = np.array([1,3,2.5]).transpose()
activation = None #T.nnet.sigmoid
L = layer(W_init,b_init,activation)
x = T.vector('x')
print L.output(x).eval({x:np.array([1.0,2,3]).astype(theano.config.floatX)})

