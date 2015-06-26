import numpy as np
import time
t_start = time.time()
import theano

A = np.random.rand(1000,10000).astype(theano.config.floatX)
B = np.random.rand(10000,1000).astype(theano.config.floatX)

X,Y = theano.tensor.matrices('XY')
mf = theano.function([X,Y],X.dot(Y))
tAB = mf(A,B)
t_end = time.time()
print "Theano time: %f[s]" %(t_end-t_start)