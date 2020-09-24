#! /usr/bin/env python
import numpy as np
import pandas as pd
import os


class BrakingAlgorithm:
    def __init__(self):
        path = os.path.dirname(os.path.abspath(__file__))
        self.weightLayer1 = pd.read_excel(path + '/layer1Weights.xlsx', header=None)
        self.weightLayer2 = pd.read_excel(path +'/layer2Weights.xlsx', header=None)
        self.weightLayer3 = pd.read_excel(path + '/layer3Weights.xlsx', header=None)
        self.biasLayer1 = pd.read_excel(path + '/layer1Bias.xlsx', header=None)
        self.biasLayer2 = pd.read_excel(path + '/layer2Bias.xlsx', header=None)
        self.biasLayer3 = 0.0954225063323975

    def algo(self, input):
        input = [[input[0]],[input[1]]]
        firstLayer = np.dot(np.asarray(input).transpose(), np.asarray(self.weightLayer1).transpose()) + np.asarray(self.biasLayer1)
        relu_firstLayer = np.asarray(np.maximum(firstLayer,0.0))
        relu_firstLayer = relu_firstLayer.astype(float)
        secondLayer = np.dot(relu_firstLayer, np.asarray(self.weightLayer2).transpose()) + np.asarray(self.biasLayer2)
        relu_secondLayer = np.asarray(np.maximum(secondLayer, 0.0))
        relu_secondLayer = relu_secondLayer.astype(float)
        thirdLayer = np.dot(relu_secondLayer, np.asarray(self.weightLayer3).transpose()) + self.biasLayer3
        return self.hard_sigmoid(thirdLayer)

    def hard_sigmoid(self, x):
        if x[0][0] < -2.5:
            return np.zeros([1, 2])
        if x[0][0] > 2.5:
            return np.ones([1, 2])
        else:
            return 0.2 * x + 0.5