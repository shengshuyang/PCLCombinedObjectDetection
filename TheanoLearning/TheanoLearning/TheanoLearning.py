import numpy as np
#import matplotlib.pyplot as plt
import theano
# By convention, the tensor submodule is loaded as T
import theano.tensor as T

def square(a):
    return a**2

###################################################

A = T.matrix('A')
x = T.vector('x')
b = T.vector('b')
y = T.dot(A, x) + b
## Note that squaring a matrix is element-wise
#z = T.sum(A**2)
## theano.function can compute multiple things at a time
## You can also set default parameter values
## We'll cover theano.config.floatX later
#b_default = np.array([0, 0], dtype=theano.config.floatX)
#linear_mix = theano.function([A, x, theano.Param(b, default=b_default)], [y, z])
## Supplying values for A, x, and b
#print linear_mix(np.array([[1, 2, 3],
#                           [4, 5, 6]], dtype=theano.config.floatX), #A
#                 np.array([1, 2, 3], dtype=theano.config.floatX), #x
#                 np.array([4, 5], dtype=theano.config.floatX)) #b
## Using the default value for b
#print linear_mix(np.array([[1, 2, 3],
#                           [4, 5, 6]]), #A
#                 np.array([1, 2, 3])) #x

#y_J = theano.gradient.jacobian(y,x)
##y_J.eval({x:np.array([1,1]), A : np.array([[10,2],[7,11]]), b : np.array([3,4])})
#linear_mix_J = theano.function([A, x, b], y_J)
## Because it's a linear mix, we expect the output to always be A
#print linear_mix_J(np.array([[9, 8, 7], [4, 5, 6]]), #A
#                   np.array([1, 2, 3]), #x
#                   np.array([4, 5])) #b
###################################################
a = T.scalar('a')
b = square(a)
c = square(b)
grad = T.grad(b,a)

f = theano.function([a],a**2)
f_grad = theano.function([a],grad)
print f_grad(10)
print grad.eval({a:10})
print c.eval({a:10})

#shared_var = theano.shared(np.array([[1, 2], [3, 4]], dtype=theano.config.floatX))
#bias = T.matrix('bias')
#shared_squared = shared_var**2 + bias

#f1 = theano.function([bias],shared_squared)
#print f1(np.array([[1,1],[1,1]]))

#diff = T.matrix('diff')
#f2 = theano.function([diff],shared_squared, updates = {shared_var: shared_var + diff})

#ret = f2(np.array([[2,2],[1,1]]))
#print shared_squared
#print shared_var.get_value()


