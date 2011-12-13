"""
This module implements the Sparse Spectrum Gaussian Process for Regression. A 
detailed description of the algorithm can be found in:

  "Sparse Spectrum Gaussian Process Regression." 
  Miguel Lazaro-Gredilla, Joaquin Quinonero-Candela, Carl Edward Rasmussen, 
  and Anibal R. Figueiras-Vidal. 
  In Journal of Machine Learning Research (JMLR) 2010.


 Copyright (C) 2011 Arjan Gijsberts <arjan@liralab.it>

"THE BEER-WARE LICENSE" (Revision 43):
<arjan@liralab.it> wrote this file. As long as you retain this notice you
can do whatever you want with this stuff. If we meet some day, and you think
this stuff is worth it, you can buy me a beer in return. Arjan Gijsberts
"""

import sys
import numpy
import numpy.core.umath_tests
import scipy.linalg
import scipy.optimize
import time

inner1d = numpy.core.umath_tests.inner1d


# keep this here till NumPy >=1.4.0 is shipped by default on most distros
def diag_indices(n,ndim=2):
    idx = numpy.arange(n) 
    return (idx,)*ndim 


class NoPrior(object):
    """Empty hyperprior."""
    def __init__(self, *args, **kwargs): pass

    def logpdf(self, x):
        """Return the log probability for the value x."""
        return 0.

    def pdf(self, x):
        """Return the probability for the value x."""
        return 1.

    def logpdfgrad(self, x):
        """Return the gradient of the log probability for the value x."""
        return 0.


class NormalDistribution(object):
    """Normally distributed hyperprior."""
    def __init__(self, mu = 1., sigma = 1.):
        """Construct a normally distributed hyperprior with mean mu and standard 
        deviation sigma."""
        assert sigma > 0.
        self.mu = mu
        self.sigma = sigma
        self.c = 1. / (numpy.sqrt(2. * numpy.pi) * self.sigma)
        self.logc = numpy.log(self.c)

    def logpdf(self, x):
        """Return the log probability for the value x."""
        xmu = (x - self.mu)
        return self.logc + (xmu * xmu) / (-2. * (self.sigma * self.sigma))

    def pdf(self, x):
        """Return the probability for the value x."""
        xmu = (x - self.mu)
        return self.c * numpy.exp((xmu * xmu) / (-2. * (self.sigma * self.sigma)))

    def logpdfgrad(self, x):
        """Return the gradient of the log probability for the value x."""
        return (self.mu - x) / (self.sigma * self.sigma)


class LogNormalDistribution(object):
    def __init__(self, mu = 1., sigma = 1.):
        """Construct a log-normally distributed hyperprior with mean mu and standard 
        deviation sigma."""
        assert sigma > 0.
        self.mu = mu
        self.sigma = sigma
        self.c = 1. / (numpy.sqrt(2. * numpy.pi) * self.sigma)
        self.logc = numpy.log(self.c)

    def logpdf(self, x):
        """Return the log probability for the value x."""
        logx = numpy.log(x)
        if logx is numpy.nan:
            logx = numpy.log(1e-300)
        return -logx + self.logc - ((logx - self.mu)**2 / (2. * (self.sigma * self.sigma)))

    def pdf(self, x):
        """Return the probability for the value x."""
        if numpy.abs(x - 0.) < 1e-12: x = 1e-12
        logx = numpy.log(x)
        if logx is numpy.nan:
            logx = numpy.log(1e-300)
        return 1./x * self.c * numpy.exp((logx - self.mu)**2 / (-2. * self.sigma2))

    def logpdfgrad(self, x):
        """Return the gradient of the log probability for the value x."""
        if numpy.abs(x - 0.) < 1e-12: x = 1e-12
        logx = numpy.log(x)
        if logx is numpy.nan:
            logx = numpy.log(1e-300)
        return (self.mu - (self.sigma * self.sigma) - logx) / (x * (self.sigma * self.sigma))



class SparseSpectrumFeatures(object):
    """Sparse Spectrum features approximate a Gaussian kernel in a finite 
    dimensionality by sampling from its normalized Fourier transform."""
    def __init__(self, n, nproj = 50, sigma_o = 0.1, sigma_o_prior = NoPrior(), l = [], l_prior = None, fixed = False):
        """Construct a Sparse Spectrum feature mapping. The parameter fixed 
        determines whether the sparse spectrum frequencies are kept fixed or  
        will be optimized as well."""
        assert sigma_o >= 0
        assert len(l) == n
        self.n = n
        self.sigma_o = sigma_o
        self.sigma_o_prior = sigma_o_prior
        self.l = numpy.array(l)
        self.l_prior = l_prior or [NoPrior()] * len(self.l)
        self.nproj = nproj
        self.fixed = fixed

        self.reset()

    def reset(self):
        """Recomputes the initialization of the feature mapping."""
        self.Wf = numpy.random.randn(self.nproj, self.n)
        self.rescale()

    def rescale(self):
        """Scales the randomly samples frequencies with respect to the 
        characteristic length scales."""
        self.W = self.Wf / self.l

    def getparams(self):
        """Returns a concatenation of the parameter sigma, the characteric 
        length scales, and (if not fixed) the sparse spectrum frequencies in a 
        flat array."""
        if self.fixed:
            return numpy.concatenate(([self.sigma_o], self.l))
        else:
            return numpy.concatenate(([self.sigma_o], self.l, self.Wf.T.flatten()))

    def setparams(self, params):
        """Obtains the parameter sigma, the characteristic length scales, and 
        (if not fixed) the sparse spectrum frequencies in that order from a 
        flat array.""" 
        params = numpy.array(params)
        assert params.shape[0] == 1 + self.n or params.shape[0] == 1 + self.n + (self.nproj * self.n)

        self.sigma_o = params[0]
        assert self.sigma_o >= 0.

        self.l = params[1:self.n+1]
        assert (self.l >= 0.).all()

        if not self.fixed and params.shape[0] == 1 + self.n + (self.nproj * self.n):
            self.Wf = params[self.n+1:].reshape((self.n, self.nproj)).T

        self.rescale()

    def guessparams(self, X, Y):
        """Perform a rudimentary guess of the parameters based on the data."""
        # initial estimate of hyperparameters similar to SPGP code [Snelson et al.]
        self.sigma_o = Y.std(axis=0).max() / 1.
        self.l = X.ptp(axis=0) / 0.5
        self.rescale()

    def outputdim(self):
        """Return the effective output dimension of the feature mapping."""
        return self.nproj * 2
    
    def getpriors(self):
        """Return the hyperpriors."""
        return [self.sigma_o_prior] + self.l_prior
    
    def setpriors(self, priors):
        """Set the hyperpriors."""
        self.sigma_o_prior = priors[0]
        self.l_prior = priors[1:]

    def priorlogprob(self):
        """Return the log probability of the parameters given the hyperpriors."""
        return numpy.array([p.logpdf(x) for p, x in zip(self.getpriors(), self.getparams())])

    def priorloggrad(self):
        """Return the gradient of the log probability of the parameters with 
        respect to the hyperpriors."""
        return numpy.array([p.logpdfgrad(x) for p, x in zip(self.getpriors(), self.getparams())])

    def evaluate(self, X):
        """Returns the evaluation of the feature mapping on input data."""
        Phi = numpy.dot(X, self.W.T)
        f = self.sigma_o / numpy.sqrt(self.nproj)
        return numpy.hstack((f * numpy.cos(Phi), f * numpy.sin(Phi)))

    def gradient(self, Xp, X, A1, A2, sigma_n):
        """Computes the gradient of the log marginal likelihood with respect to 
        the parameters and precomputed settings from the machine."""
        # This implementation is quite fast though incomprehensible. I've
        # heavily used numpy's broadcasting in combination with array reshaping. 
        # Additionally, I used the undocumented inner1d generalized ufunc.
        # I am open to any improvements to the readability of the code if it 
        # does not harm performance.
        grad = numpy.zeros((A1.shape[1], self.getparams().shape[0]))

        sigma_o2 = self.sigma_o * self.sigma_o
        sigma_n2 = sigma_n * sigma_n

        A1Xp = numpy.dot(A1.T, Xp)
        A2Xp = numpy.dot(A2.T, Xp)
        
        m = Xp.shape[0]

        # sigma_o gradient
        grad[:,0] = (((sigma_o2) / self.nproj) * (m * self.nproj) / (sigma_n2) - 
                   (inner1d(A1Xp, A1Xp) + inner1d(A2Xp, A2Xp).sum())) / self.sigma_o
        
        B1 = (A1.T[:,numpy.newaxis,:] * A1Xp[:,:self.nproj,numpy.newaxis]) * Xp[:,self.nproj:].T - \
             (A1.T[:,numpy.newaxis,:] * A1Xp[:,self.nproj:,numpy.newaxis]) * Xp[:,:self.nproj].T
        B2 = numpy.dot(A2Xp[:,:self.nproj].T, A2.T) * Xp[:,self.nproj:].T - \
             numpy.dot(A2Xp[:,self.nproj:].T, A2.T) * Xp[:,:self.nproj].T
        
        XB = (numpy.dot(B1, X) + numpy.dot(B2, X)) / self.l

        # length scale gradient        
        grad[:,1:1+self.n] = inner1d(XB.swapaxes(2,1), -self.W.T)
        #grad[:,1:1+self.n] = -1. * (XB * self.W).sum(axis=1)
        
        if not self.fixed:
            # sparse spectrum frequency gradient
            shape = (A1.shape[1], XB.shape[1]*XB.shape[2])
            grad[:,1+self.n:] = XB.swapaxes(2,1).reshape(shape)

        return -grad


#
# Linear Gaussian Process Regression (with optional mapping)
#
class LinearGPR(object):
    """Implementation of standard Bayesian regression, or a linear Gaussian
    Process for Regression. For more information, see:

    Gaussian Processes for Machine Learning.
      Carl Edward Rasmussen and Christopher K. I. Williams.
      The MIT Press, 2005.

    Pattern Recognition and Machine Learning.
      Christopher M. Bishop.
      Springer-Verlag, 2006.
    """
    def __init__(self, n, p, mapping, sigma_n = 0.1, sigma_n_prior = NoPrior()):
        """Construct a Linear Gaussian Process for Regression with the specified 
        feature mapping."""
        assert sigma_n >= 0.
        assert n > 0
        assert p > 0

        self.n = n
        self.p = p

        self.mapping = mapping
        self.sigma_n = sigma_n
        self.sigma_n_prior = sigma_n_prior

        self.reset()


    def setparams(self, params):
        """Obtains the parameter sigma and the feature mapping parameters in 
        that order from a flat array. The machine is reset afterwards.""" 
        assert len(params) == len(self.getparams())
        params = numpy.array(params)

        self.sigma_n = params[0]
        assert self.sigma_n > 0.

        self.mapping.setparams(params[1:])

        self.reset()
        
    def getparams(self):
        """Returns a concatenation of the sigma parameter and the feature mapping 
        parameters in a flat array."""
        return numpy.concatenate(([self.sigma_n], self.mapping.getparams()))

    def guessparams(self, X, Y):
        """Perform a rudimentary guess of the machine and mapping parameters 
        based on the data."""
        self.sigma_n = Y.std(axis=0).max() / 4.
        self.mapping.guessparams(X, Y)

    def reset(self):
        """Clears the machine to an initial state."""
        # take the output dimension after preprocessing
        n = self.mapping.outputdim()
        self.L = self.sigma_n * numpy.identity(n)
        self.B = numpy.zeros((n, self.p))
        self.W = numpy.zeros((n, self.p))
        try:
            del self.LPhiY, self.X, self.Phi, self.Y, self.lml
        except AttributeError:
            pass
        

    def predict(self, Xstar):
        """Predicts the output for the supplied input data."""
        assert len(Xstar.shape) == 1 or len(Xstar.shape) == 2
        assert Xstar.shape[-1] == self.n 
        if Xstar.ndim == 2:
            Phistar = self.mapping.evaluate(Xstar)
            
            # see Bishop, p. 174, 3.58
            mean = numpy.dot(Phistar, self.W)

            # see Bishop, p. 174, 3.59
            # see Rasmussen, p. 30, 2.11
            sigma_n2 = self.sigma_n * self.sigma_n
        
            vT, info = scipy.linalg.flapack.dgetrs(self.L, numpy.arange(self.L.shape[0]), 
                                                   Phistar.T, trans=1, overwrite_b=1)
            assert info == 0, 'scipy.linalg.flapack.dgetrs(L, Phi*)'

            var = numpy.empty((Xstar.shape[0], self.W.shape[1]))
            var = sigma_n2 * (1. + inner1d(vT.T, vT.T))[:,numpy.newaxis]

            return mean, var
        else:
            mean, var = self.predict(numpy.array([sx]))
            return mean[0], var[0]


    def train(self, X, Y):
        """Trains the machine on the supplied set of inputs and outputs."""
        assert X.shape[0] == Y.shape[0]
        assert len(X.shape) == len(Y.shape) == 2
        assert X.shape[1] == self.n
        assert Y.shape[1] == self.p

        self.X = X
        self.Y = Y

        self.Phi = self.mapping.evaluate(self.X)
        m, n = self.Phi.shape

        sigma_n2 = self.sigma_n * self.sigma_n
        self.L = numpy.dot(self.Phi.T, self.Phi)
        idx = diag_indices(self.Phi.shape[1])
        self.L[idx] += sigma_n2
        self.up = 0
        self.L, info = scipy.linalg.flapack.dpotrf(self.L,lower=self.up,clean=1,overwrite_a=1)

        self.B = numpy.dot(self.Phi.T, self.Y)
        self.LPhiY, info = scipy.linalg.flapack.dgetrs(self.L,numpy.arange(self.L.shape[0]),
                                                        self.B, trans=1, overwrite_b=1)
        assert info == 0, 'scipy.linalg.flapack.dgetrs(L, B)'

        # former should be faster, but isn't on small datasets
        #self.w, info = scipy.linalg.flapack.dgetrs(self.L,numpy.arange(self.L.shape[0]), 
        #                                           LPhiy, trans=0, overwrite_b=0)
        self.W = scipy.linalg.cho_solve((self.L, self.up), self.B)

        self.lml = -0.5 * ((1. / (sigma_n2)) * (inner1d(self.Y.T, self.Y.T) - inner1d(self.LPhiY.T, self.LPhiY.T)) + 
                           (m - n) * numpy.log(sigma_n2) + m * numpy.log(2. * numpy.pi)) - \
                    numpy.log(self.L.diagonal()).sum()

    def lmlfunc(self):
        """Computes the log marginal likelihood given the trained state and the 
        hyperpriors."""
        return self.lml.sum() + self.sigma_n_prior.logpdf(self.sigma_n) + \
               self.mapping.priorlogprob().sum()

    def lmlgradient(self):
        """Computes the gradient of the log marginal likelihood given the 
        trained state and the hyperpriors.""" 
        LPhi, info = scipy.linalg.flapack.dgetrs(self.L,numpy.arange(self.L.shape[0]),
                                                 self.Phi.T, trans=1, overwrite_b=0)
        assert info == 0, 'scipy.linalg.flapack.dgetrs(L, Phi.T)'
        A2 = (1. / self.sigma_n) * LPhi.T
        LPhiY = numpy.dot(LPhi, self.Y)
        sigma_n2 = self.sigma_n * self.sigma_n
        A1 = (1. / sigma_n2) * (self.Y - numpy.dot(LPhi.T, LPhiY))
        mapgrad = self.mapping.gradient(self.Phi, self.X, A1, A2, self.sigma_n)

        grad = self.sigma_n * (inner1d(A1.T, A1.T) + inner1d(A2.T, A2.T).sum()) - \
               self.Phi.shape[0] * (1. / self.sigma_n)
        grad = grad.sum() + self.sigma_n_prior.logpdfgrad(self.sigma_n) / self.sigma_n
        return numpy.concatenate(([grad], mapgrad.sum(axis=0)))


    def optimize(self, X, Y, solver = 'ralg', ftol = 1e-4, gtol= 1e-4, contol = 1e-6, 
                       maxIter = 2500, maxFunEvals = 2500, maxtime = 3600., checkgrad = False, 
                       verbose = False):
        """Minimizes the negative log marginal likelihood. The positivity 
        constraints of several parameters is enforced by treating these in 
        exponential scale.

        Requires: openopt"""
        import openopt

        def checkretrain(params):
            try:
                self.machine.lmlfunc()
                if not (numpy.abs(self.getparams() - params) < 1e-12).all():
                    self.setparams(params)
                    self.train(X, Y)
            except AttributeError:
                self.setparams(params)
                self.train(X, Y)

        def convert(params):
            pconv = params.copy()
            pconv[:2+self.n] = numpy.exp(pconv[:2+self.n])
            return pconv

        def convertgrad(grad, params):
            grad[:2+self.n] = grad[:2+self.n] * convert(params)[:2+self.n]
            return grad

        def invert(params):
            pinv = params.copy()
            pinv[:2+self.n] = numpy.log(pinv[:2+self.n])
            return pinv

        def f(params):
            checkretrain(convert(params))
            return -self.lmlfunc()

        def df(params):
            checkretrain(convert(params))
            return convertgrad(-self.lmlgradient(), params)

        x0 = invert(self.getparams())

        if verbose:
            iprint = 10
            itercb = []
        else:
            iprint = -1

            def itercb(p):
                lib = ['-','\\','|','/']
                symbol = lib[p.iter % len(lib)]
                sys.stdout.write('%-40s\r' % ('%s iteration: %4d lml: %g' % (symbol, p.iter, p.fk)))
                sys.stdout.flush()
                return False

        p = openopt.NLP(f, x0, df = df, iprint = iprint, callback = itercb, 
                        gtol = gtol, contol = contol, ftol = ftol, 
                        maxIter = maxIter, maxFunEvals = maxFunEvals, maxTime = maxtime)

        if checkgrad:
            p.checkdf()

        r = p.solve(solver)

        self.setparams(convert(r.xf))

        if not verbose:
            sys.stdout.write('%40s\r' % '')
            sys.stdout.flush()

        return r 

