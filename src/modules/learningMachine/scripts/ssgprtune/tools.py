"""
Helper functions for the Sparse Spectrum Gaussian Process for Regression script.

 Copyright (C) 2011 Arjan Gijsberts <arjan@liralab.it>

"THE BEER-WARE LICENSE" (Revision 43):
<arjan@liralab.it> wrote this file. As long as you retain this notice you
can do whatever you want with this stuff. If we meet some day, and you think
this stuff is worth it, you can buy me a beer in return. Arjan Gijsberts
"""
import types
import sys
import numpy


def strtoidx(s):
    """Converts a string representation of indices to a list."""
    if s is None: 
        return None

    cols = []
    col_str = s.split(',')
    for col_elem in col_str:
        spl = col_elem.split('-', 2)
        if len(spl) == 1: cols.append(int(spl[0]) - 1)
        if len(spl) == 2: cols.extend(range(int(spl[0]) - 1, int(spl[1])))
    return cols

def load_array(fname):
    """Loads an array from a either a plain text or native numpy file."""
    import os
    if os.path.splitext(fname)[1] == '.npy':
        return numpy.load(fname)
    else:
        return numpy.loadtxt(fname)

def load_data(filename, input_cols, output_cols):
    """Loads a dataset of inputs and outputs given lists of indices for inputs 
    and outputs from a file."""
    data = load_array(filename)

    inputs = data[:,input_cols]
    outputs = data[:,output_cols]

    return inputs, outputs

def serialize(fp, *args):
    """Serializes strings, floats, integers, and arrays into a format 
    understood by the learningMachine framework."""
    for item in args:
        if type(item) == str:      
            fp.write('%s' % item)
        elif type(item) in [float, numpy.float32, numpy.float64]:  
            fp.write('%g ' % item)
        elif type(item) in [int, numpy.int16, numpy.int32, numpy.int64]:    
            fp.write('%d ' % item)
        elif type(item) == numpy.ndarray:
            numpy.savetxt(fp, item.flatten()[numpy.newaxis,:], fmt='%.10g', newline=' ')
            serialize(fp, *list(item.shape))

def serialize_preprocessor(filename, preprocessor):
    """Serializes the feature mapping, or preprocess, to a format understood by 
    the learningMachine framework."""
    with open(filename, 'w') as ppfp:
        serialize(ppfp, 'SparseSpectrumFeature\n', 
                        preprocessor.sigma_o, preprocessor.l, preprocessor.W,
                        preprocessor.n, preprocessor.outputdim())

def serialize_machine(filename, machine):
    """Serializes the machine to a format understood by the learningMachine 
    framework."""

    # copy upper triangular to lower triangular
    def reflect(arr):
        arr2 = arr.copy()
        arr2[numpy.tril_indices(arr.shape[0], -1)] = arr2.T[numpy.tril_indices(arr.shape[0], -1)]
        return arr2

    with open(filename, 'w') as lfp:
        try:
            m = machine.X.shape[0]
        except AttributeError:
            m = 0


        serialize(lfp, 'LinearGPR\n', 
                       reflect(machine.L), machine.B.T, machine.W.T, machine.sigma_n, m, 
                       machine.mapping.outputdim(), machine.p)

