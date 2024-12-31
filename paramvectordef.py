import random
import numpy as np
from enum import Enum
from itertools import product


class ParamType(Enum):
    SCALAR = 1
    BINARY = 2
    INTEGER = 3
    TYPE = 4


class Parameter:
    def __init__(self, paramtype, data):
        self.paramtype = paramtype
        self.data = data


class ParamVectorDef:
    def __init__(self, shape):
        self.shape = shape
        self.params = []

        if shape == 'bed':
            self.params.append(Parameter(ParamType.SCALAR, [
                np.linspace(0.0, 1.0, 5).tolist(),
                np.linspace(0.0, 1.0, 3).tolist(),
                np.linspace(0.0, 1.0, 5).tolist(),
                np.linspace(0.0, 1.0, 5).tolist(),
                np.linspace(0.0, 1.0, 3).tolist()
            ]))
            self.params.append(Parameter(ParamType.TYPE, ['basic', 'split', 'box']))
            self.params.append(Parameter(ParamType.INTEGER, [0, 2]))
        elif shape == 'chair':
            self.params.append(Parameter(ParamType.SCALAR, [
                np.linspace(0.0, 1.0, 6).tolist(),
                np.linspace(0.0, 1.0, 3).tolist(),
                np.linspace(0.0, 1.0, 5).tolist()
            ]))
            self.params.append(Parameter(ParamType.TYPE, ['basic', 'round', 'support', 'office']))
            self.params.append(Parameter(ParamType.TYPE, ['none', 'basic', 'solid', 'office']))
            self.params.append(Parameter(ParamType.TYPE, ['basic', 'hbar', 'vbar', 'office']))
        elif shape == 'shelf':
            self.params.append(Parameter(ParamType.SCALAR, [
                np.linspace(0.0, 1.0, 8).tolist(),
                np.linspace(0.0, 1.0, 3).tolist(),
                np.linspace(0.0, 1.0, 3).tolist()
            ]))
            self.params.append(Parameter(ParamType.INTEGER, [1, 5]))
            self.params.append(Parameter(ParamType.INTEGER, [1, 5]))
            self.params.append(Parameter(ParamType.BINARY, None))
            self.params.append(Parameter(ParamType.BINARY, None))
            self.params.append(Parameter(ParamType.BINARY, None))
        elif shape == 'table':
            self.params.append(Parameter(ParamType.SCALAR, [
                np.linspace(0.0, 1.0, 8).tolist(),
                np.linspace(0.0, 1.0, 4).tolist(),
                np.linspace(0.0, 1.0, 3).tolist(),
                np.linspace(0.0, 1.0, 3).tolist()
            ]))
            self.params.append(Parameter(ParamType.BINARY, None))
            self.params.append(Parameter(ParamType.TYPE, ['basic', 'support', 'round', 'split', 'square', 'solid']))

    def get_random_vectors(self, max_len=3000):
        param_domain = []
        for param in self.params:
            if param.paramtype == ParamType.SCALAR:
                for elem in param.data:
                    param_domain.append(elem)
            elif param.paramtype == ParamType.BINARY:
                param_domain.append([False, True])
            elif param.paramtype == ParamType.INTEGER:
                param_domain.append(list(range(param.data[0], param.data[1]+1)))
            elif param.paramtype == ParamType.TYPE:
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
                if paramtype == ParamType.SCALAR:
                    min_vals = np.array([x[0] for x in param.data], ndmin=2)
                    max_vals = np.array([x[-1] for x in param.data], ndmin=2)
                    data = (data - min_vals) / (max_vals - min_vals)
                encoded_vectors.append(data)
            elif paramtype == ParamType.INTEGER:
                ohe_coded = np.array(
                    [[0 if i != (x[index] - param.data[0]) else 1 for i in range(param.data[1] - param.data[0] + 1)] for
                     x in vectors]
                )
                encoded_vectors.append(ohe_coded)
            elif paramtype == ParamType.TYPE:
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
            if paramtype == ParamType.SCALAR:
                min_vals = np.array([x[0] for x in param.data], ndmin=2)
                max_vals = np.array([x[-1] for x in param.data], ndmin=2)
                data = data * (max_vals - min_vals) + min_vals
            elif paramtype == ParamType.BINARY:
                data = np.isclose(data, 1.0)
            elif paramtype == ParamType.INTEGER:
                data = np.expand_dims(param.data[0]+np.argmax(data, axis=1), axis=1)
            elif paramtype == ParamType.TYPE:
                data = np.expand_dims(np.array(param.data)[np.argmax(data, axis=1)], axis=1)
            collector.append(data.tolist())
        decoded_vectors = []
        for i in range(len(collector[0])):
            vector = []
            for j in range(len(self.params)):
                vector.extend(collector[j][i])
            decoded_vectors.append(vector)
        return decoded_vectors


def unit_test(shape, num_samples):
    param_def = ParamVectorDef(shape)
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
    vectors = param_def.get_random_vectors(num_samples)
    enc_vectors = param_def.encode(vectors)
    dec_vectors = param_def.decode(enc_vectors)
    print(vectors)
    print(dec_vectors)


if __name__ == '__main__':
    unit_test('chair', num_samples=5)

