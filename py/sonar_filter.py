import numpy as np
import scipy.stats as ss
import time


class SonarFilter():

    def __init__(self):
        self.iir_last_value = 0.0
        self.iir_a = 0.4  # a1 coefficient of the iir (recursive) filter
        self.iir_b = 1.0 - self.iir_a  # b0 coefficient of the iir filter
        self.ma_order = 2  # order of the MA filter
        self.ma_M = self.ma_order + 1  # size of the MA filter (order +1)
        self.ma_memory = np.zeros(self.ma_M)  # memory for MA filter
        self.ma_cnt = 0  # count the number of value in MA memory
        self.median_size = 5
        self.median_memory = np.zeros(self.median_size)
        self.median_cnt = 0
        self.Bias = 0
        self.etat_transitoire = True

    # reset filter when sonar detects nothing
    def reset_ma(self):
        self.ma_memory = np.zeros(self.ma_M)
        self.ma_cnt = 0

    def reset_iir(self):
        self.iir_last_value = 0.0

    # set filter parameters
    def set_ma_order(self, N):
        self.ma_order = N
        self.ma_M = self.ma_order + 1
        self.ma_memory = np.zeros(self.ma_M)
        self.ma_cnt = 0

    def set_iir_a(self, a):
        self.iir_a = a
        self.iir_b = 1.0 - self.iir_a
        self.iir_last_value = 0.0

    # implement at least one of the two filters
    def ma_filter(self, v):
        t = np.zeros(self.ma_M)
        t[0] = v
        for i in range(self.ma_M - 1):
            t[i + 1] = self.ma_memory[i]
        vf = np.mean(t)
        self.ma_memory = t
        return vf

    def iir_filter(self, v):
        self.iir_last_value = self.iir_a * self.iir_last_value + v * self.iir_b  # no filter !!!
        return self.iir_last_value

    # median only for the advanced challenge
    def reset_median(self):
        self.ma_memory = np.zeros(self.ma_M)
        self.ma_cnt = 0

    def set_median_size(self, size):
        self.median_size = size
        self.median_memory = np.zeros(self.median_size)
        self.median_cnt = 0

    def median_filter(self, v):
        l = list(self.median_memory)
        l.pop(0)
        l.append(v)
        self.median_memory = np.array(l)
        l.sort()
        vf = l[len(l) // 2]
        if vf == 0:
            return v
        else:
            return vf
