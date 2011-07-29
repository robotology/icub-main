#!/usr/bin/env python
"""
This script facilitates training, hyperparameter optimization, and testing 
of the Sparse Spectrum Gaussian Process for Regression. In addition, models of 
the algorithm can be exported to a format that can be imported directly in the 
iCub learningMachine framework (see example below).

Notes:
 - Please supply the --help parameter to see an overview of the available 
   options.
 - All outputs dimensions share the same hyperparameter configuration. If this 
   is problematic in your application domain, consider learning different 
   (sets of) outputs individually.
 - By design, the method uses a randomized initialization for the feature 
   mapping. The results may therefore vary depending on the seed of the PRNG.
   However, variance in results will decrease as the number of projections 
   increases.
 - Hyperparameter optimization is performed by minimizing the negative log 
   marginal likelihood. This is a non-convex, non-linear optimization problem 
   and may therefore result in local optima, depending on the initial 
   parameter configuration.


Dependencies: numpy, scipy, openopt


Example of using models in the learningMachine framework:
./lmtrain --load ssgpr.model
./lmtransform --trainport /lm/train/train:i --predictport /lm/train/predict:io --load ssf.model
./lmtest --trainport /lm/transform/train:i --predictport /lm/transform/predict:io --inputs '(1 2 ... n)' --outputs '(1 2 ... p)' --datafile dataset.dat


 Copyright (C) 2011 Arjan Gijsberts <arjan@liralab.it>

"THE BEER-WARE LICENSE" (Revision 43):
<arjan@liralab.it> wrote this file. As long as you retain this notice you
can do whatever you want with this stuff. If we meet some day, and you think
this stuff is worth it, you can buy me a beer in return. Arjan Gijsberts
"""

import numpy
import optparse
import time
import scipy

from ssgpr import SparseSpectrumFeatures, LinearGPR, NoPrior, \
                  NormalDistribution, LogNormalDistribution

import tools


def main():
    print 'SSGPR Tuning'

    usage = 'usage: %prog [options]'
    parser = optparse.OptionParser(usage)
    generalgroup = optparse.OptionGroup(parser, 'General Options')
    datasetgroup = optparse.OptionGroup(parser, 'Dataset Options')
    optimizationgroup = optparse.OptionGroup(parser, 'Optimization Options')


    generalgroup.add_option('--projections', dest='projections', type='int', default=100, metavar='D', 
                      help='number of spectral projections (default: 100)')
    generalgroup.add_option('--nofixed', dest='fixed', action='store_false', default=True, 
                      help='tune sparse spectrum frequencies')
    generalgroup.add_option('--params', dest='params', default=None, metavar='[\sigma_n, \sigma_f, \ell_1,  ..., \ell_n]', 
                      help='set hyperparameters (takes precedence over --guess)')
    generalgroup.add_option('--seed', dest="seed", default=None, type='int', 
                      help='seed for the PRNG (default: None)')
    generalgroup.add_option('--yarp', dest="yarp", default=False, action='store_true', 
                      help='serialize to YARP learningMachine compatible files')

    # datasets for 4 different functions
    datasetgroup.add_option('-i', '--inputs', dest='inputs', default=None, metavar='IDX[,IDX]*', 
                      help='input column indices')
    datasetgroup.add_option('-o', '--outputs', dest='outputs', default=None, metavar='IDX[,IDX]*', 
                      help='output column indices')
    datasetgroup.add_option('--guess', dest='guess', default=None, metavar='DATASET', 
                      help='guess hyperparameters using specified dataset')
    datasetgroup.add_option('--optimize', dest='optimize', default=None, metavar='DATASET', 
                      help='optimize hyperparameters using specified dataset')
    datasetgroup.add_option('--train', dest='train', default=None, metavar='DATASET', 
                      help='train machine using specified dataset')
    datasetgroup.add_option('--test', dest='test', default=None, metavar='DATASET', 
                      help='test machine on specified dataset')

    # optimization options
    optimizationgroup.add_option('--solver', dest='solver', default='ralg', 
                      help='solver (default: ralg)')
    optimizationgroup.add_option('--verboseopt', dest='verboseopt', default=False, action='store_true', 
                      help='enable verbose optimization output')
    optimizationgroup.add_option('--ftol', dest='ftol', type='float', default=1e-4, metavar='TOL', 
                      help='function tolerance for stop condition (default: 1e-4)')
    optimizationgroup.add_option('--gtol', dest='gtol', type='float', default=1e-4, metavar='TOL', 
                      help='gradient tolerance for stop condition (default: 1e-4)')
    optimizationgroup.add_option('--maxtime', dest='maxtime', type='float', default=3600., metavar='SECONDS', 
                      help='maximum time (default: 3600)')
    optimizationgroup.add_option('--maxiters', dest='maxiters', type='int', default=2000, metavar='ITERS', 
                      help='maximum iterations (default: 2000)')
    optimizationgroup.add_option('--maxfevals', dest='maxfevals', type='int', default=5000, metavar='EVALS', 
                      help='maximum function evaluations (default: 5000)')

    parser.add_option_group(generalgroup)
    parser.add_option_group(datasetgroup)
    parser.add_option_group(optimizationgroup)

    (options, args) = parser.parse_args()

    numpy.random.seed(options.seed)

    input_cols = tools.strtoidx(options.inputs) or [0]
    output_cols = tools.strtoidx(options.outputs) or [-1]

    n = len(input_cols)
    p = len(output_cols)

    # some arbitrary default parameters and no hyperpriors
    sigma_o, sigma_o_prior = 2., NoPrior()
    l, l_prior = [10.] * n, [NoPrior()] * n
    sigma_n, sigma_n_prior = 0.2, NoPrior()

    # construct machine and feature mapping
    ssf = SparseSpectrumFeatures(n, nproj = options.projections, 
                                 sigma_o = sigma_o, sigma_o_prior = sigma_o_prior, 
                                 l = l, l_prior = l_prior, fixed = options.fixed)
    ssgpr = LinearGPR(n, p, ssf, sigma_n = sigma_n, sigma_n_prior = sigma_n_prior)

    print 'General Info'
    print '%12s: %s -> %s' % ('columns', input_cols, output_cols)
    print '%12s: %d' % ('#proj', options.projections)
    print '%12s: %s' % ('fixed', options.fixed)
    print '%12s: (%d -> %d) -> %d' % ('dimensions', n, ssf.outputdim(), p)
    print '%12s: %s' % ('seed', options.seed)


    # rudimentary guess of hyperparameters based on data
    if options.guess:
        print '\nHyperparameter Guess: %s' % (options.guess)

        guessx, guessy = tools.load_data(options.guess, input_cols, output_cols)
        print '%12s: [%d x %d] -> [%d x %d]' % (('data', ) + guessx.shape + guessy.shape)

        start = time.time()
        ssgpr.guessparams(guessx, guessy)
        end = time.time()
        print '%12s: %d seconds' % ('timing', end - start)

    # set hyperparameters if given
    if options.params is not None:
        ssgpr.setparams(list(eval(options.params)))

    # optimize hyperparameters
    if options.optimize:
        print '\nHyperparameter Optimization: %s' % (options.optimize)

        hyperx, hypery = tools.load_data(options.optimize, input_cols, output_cols)
        print '%12s: [%d x %d] -> [%d x %d]' % (('data', ) + hyperx.shape + hypery.shape)
        print '%12s: %s' % ('solver', options.solver)
        print '%12s: % g' % ('ftol', options.ftol)
        print '%12s: % g' % ('gtol', options.gtol)
        print '%12s: % g seconds' % ('max time', options.maxtime)
        print '%12s: % d' % ('max fevals', options.maxfevals)
        print '%12s: % d' % ('max iters', options.maxiters)

        start = time.time()
        res = ssgpr.optimize(hyperx, hypery, solver = options.solver, verbose = options.verboseopt, 
                       ftol = options.ftol, gtol = options.gtol, 
                       maxtime = options.maxtime, maxIter = options.maxiters, maxFunEvals = options.maxfevals, 
                       checkgrad = False)
        end = time.time()
        print '%12s: % g seconds' % ('timing', end - start)
        print '%12s: % g' % ('opt lml', res.ff)
        print '%12s: %s' % ('stop cond', res.msg)
        print '%12s: % d' % ('fevals', res.evals['f'])
        print '%12s: % d' % ('dfevals', res.evals['df'])
        print '%12s: % d' % ('iters', res.evals['iter'])

    # train ssgpr using dataset
    if options.train:
        print '\nTraining: %s' % (options.train)

        trainx, trainy = tools.load_data(options.train, input_cols, output_cols)
        print '%12s: [%d x %d] -> [%d x %d]' % (('data', ) + trainx.shape + trainy.shape)

        start = time.time()
        ssgpr.train(trainx, trainy)
        end = time.time()
        print '%12s: % g seconds' % ('timing', end - start)


        lml = ssgpr.lmlfunc()
        print '%12s: % g' % ('lml', lml)

    # test ssgpr on dataset
    if options.test:
        print '\nTesting: %s' % (options.test)

        testx, testy = tools.load_data(options.test, input_cols, output_cols)
        print '%12s: [%d x %d] -> [%d x %d]' % (('data', ) + testx.shape + testy.shape)

        start = time.time()
        testy_p, testy_pv = ssgpr.predict(testx)
        end = time.time()
        print '%12s: % g seconds' % ('timing', end - start)

        se = (testy - testy_p)**2
        nse = se / testy.var(axis=0)
        lp = (se / testy_pv) + numpy.log(2. * numpy.pi) + numpy.log(testy_pv)

        print '%12s: %s' % ('mse', se.mean(axis=0))
        print '%12s: %s' % ('nmse', nse.mean(axis=0))
        print '%12s: %s' % ('nmlp', 0.5 * lp.mean(axis=0))

    if options.yarp:
        print '\nYarp LearningMachine Serialization'

        machine_fn = 'ssgpr.model'
        tools.serialize_machine(machine_fn, ssgpr)
        print '%12s: %s' % ('machine', machine_fn)

        preprocessor_fn = 'ssf.model'
        tools.serialize_preprocessor(preprocessor_fn, ssf)
        print '%12s: %s' % ('preproc', preprocessor_fn)


if __name__ == "__main__":
    main()

