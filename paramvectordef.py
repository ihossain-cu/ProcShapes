import random
import numpy as np
from enum import Enum
from typing import List
from itertools import product


class ParamType(Enum):
    SCALAR = 1
    BINARY = 2
    SET = 3


class Parameter:
    def __init__(self, paramtype: ParamType, data: List):
        self.paramtype = paramtype
        self.data = data


class ParamVectorDef:
    def __init__(self):
        self.params = []

    def add_scalars(self, nums_steps: List):
        data = [np.linspace(0.0, 1.0, x).tolist() for x in nums_steps]
        self.params.append(Parameter(ParamType.SCALAR, data))

    def add_binary(self):
        self.params.append(Parameter(ParamType.BINARY, [False, True]))

    def add_set(self, setitems: List):
        self.params.append(Parameter(ParamType.SET, setitems))

    def get_random_vectors(self, max_len=3000, use_seed=False):
        if use_seed:
            random.seed(42)
        param_domain = []
        for param in self.params:
            if param.paramtype == ParamType.SCALAR:
                for elem in param.data:
                    param_domain.append(elem)
            else:
                param_domain.append(param.data)
        if not param_domain:
            param_vectors = []
        else:
            param_vectors = [p for p in product(*param_domain)]
        random.shuffle(param_vectors)
        if max_len is not None:
            if len(param_vectors) > max_len:
                param_vectors = random.choices(param_vectors, k=max_len)
        return param_vectors

    def encode(self, vectors):
        encoded_vectors = []
        index = 0
        for param in self.params:
            paramtype = param.paramtype
            if paramtype == ParamType.SCALAR:
                col_length = len(param.data)
            else:
                col_length = 1
            if paramtype == ParamType.SCALAR or paramtype == ParamType.BINARY:
                data = np.array([x[index:index+col_length] for x in vectors], dtype=float)
                encoded_vectors.append(data)
            else:
                ohe_coded = np.array(
                    [[0 if x[index] != y else 1 for y in param.data] for x in vectors], dtype=float
                )
                encoded_vectors.append(ohe_coded)
            index = index + col_length
        return encoded_vectors

    def decode(self, vectors):
        collector = []
        for i, param in enumerate(self.params):
            paramtype = param.paramtype
            data = vectors[i]
            if paramtype == ParamType.BINARY:
                data = np.isclose(data, 1.0)
            elif paramtype == ParamType.SET:
                data = np.expand_dims(np.array(param.data)[np.argmax(data, axis=1)], axis=1)
            collector.append(data.tolist())
        decoded_vectors = []
        for i in range(len(collector[0])):
            vector = []
            for j in range(len(self.params)):
                vector.extend(collector[j][i])
            decoded_vectors.append(vector)
        return decoded_vectors

def get_param_vec_def(category: str) -> ParamVectorDef:
    paramvecdef = ParamVectorDef()
    if category == 'bed':
        paramvecdef.add_scalars([6, 4, 4, 4, 3])
        paramvecdef.add_set(['basic', 'split', 'box'])
    elif category == 'chair':
        paramvecdef.add_scalars([6, 3, 3])
        paramvecdef.add_set(['basic', 'support', 'pedestal', 'split', 'rocker'])
        paramvecdef.add_set(['none', 'basic', 'solid', 'office'])
        paramvecdef.add_set(['basic', 'hbar', 'vbar'])
    elif category == 'storage':
        paramvecdef.add_scalars([4, 2, 2])
        paramvecdef.add_set([1, 2, 3, 4, 5])
        paramvecdef.add_set([1, 2, 3, 4, 5])
        paramvecdef.add_binary()
        paramvecdef.add_binary()
        paramvecdef.add_binary()
    elif category == 'table':
        paramvecdef.add_scalars([8, 4, 3, 3])
        paramvecdef.add_binary()
        paramvecdef.add_set(['basic', 'bracket', 'solid', 'split', 'pedestal', 'support'])
    return paramvecdef

def unit_test(category, num_samples):
    paramvecdef = get_param_vec_def(category)
    '''
    Parameter vectors can be manually defined, such as
    vectors = [
        [0.6, 0.4, 0.05, 0.05, False, 'basic'],
        [0.6, 0.4, 0.05, 0.05, True, 'basic'],
        [0.6, 0.4, 0.05, 0.05, True, 'support'],
        [0.6, 0.4, 0.05, 0.05, True, 'round'],
        [0.6, 0.4, 0.05, 0.05, False, 'split']
    ]
    or we can randomly sample them.
    '''
    vectors = paramvecdef.get_random_vectors(num_samples)
    enc_vectors = paramvecdef.encode(vectors)
    dec_vectors = paramvecdef.decode(enc_vectors)
    print(vectors)
    print(dec_vectors)

if __name__ == '__main__':
    unit_test('chair', num_samples=5)

