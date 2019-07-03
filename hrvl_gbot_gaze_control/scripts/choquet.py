#!/usr/bin/python

import roslib
import rospy
import numpy as np
import math
import sys
import matplotlib.pyplot as plt

from operator import itemgetter

# class fuzzy_measure:
#     def __init__(self, f_m):

    
# use dictionary f_measure with 'bin', 'a', 'fm'
class fuzzy_integral:
    def __init__(self, num_criteria, f_measure):
        self.num_c = num_criteria # number of criteria
        self.num_pset = len(f_measure['bin']) # powerset number
        self.f_m = f_measure # fuzzy measure values

    def check_print_fm(self):
        for b in self.f_m['bin']: # index binary of powerset a
            print(b)
        for a in self.f_m['a']: # powerset number
            print(a)
        for f in self.f_m['fm']: # fuzzy measure values
            print(f)

    def update_fm(self, f_measure):
        self.f_m = f_measure

    def choquet(self, p_evals):
        output = 0
        # self.check_print_fm()
        # print "[Original p_evals]"
        # print p_evals

        # p_evals_sorted = p_evals;
        # p_evals_sorted_index = sorted(range(len(p_evals)), key=p_evals.__getitem__)
        # p_evals_sorted.sort()
        
        ind_sh, sorted_h = zip(*sorted(enumerate(p_evals), key=itemgetter(1))) # sorting with index key
        
        mask = np.ones_like(p_evals)
        b_zero = np.zeros_like(p_evals)

        ind_sh_ar = np.array(ind_sh)
        orig_ind = np.zeros_like(p_evals)

        for i in range(0,self.num_c):
            # masking-omitted
            orig_ind = ind_sh_ar[mask>0]
            # print(orig_ind)
            b_orig_ind = np.zeros_like(mask)
            b_orig_ind[orig_ind] = 1
            # print 'OrigInd'
            # print orig_ind
            # print 'b_OrigInd'
            # print b_orig_ind
            

            t_ind = 0
            o_ind = 0
            m_num = self.num_c-i
            while True:
                if t_ind > self.num_pset:
                    print "Exceeded Max number of fuzzy measure in choquet.m"
                    break
                print(np.sum(np.multiply(self.f_m['bin'][t_ind],b_orig_ind)))
                if np.sum(np.multiply(self.f_m['bin'][t_ind],b_orig_ind)) == m_num :
                    o_ind = t_ind
                    break
                else:
                    t_ind = t_ind+1
            mu = self.f_m['fm'][o_ind]
            print(o_ind)
            if i==0:
                output = output+(p_evals[i] - 0)*mu # p_evals??? not sorted_h???
            else:
                output = output+(p_evals[i] - p_evals[i-1])*mu

            mask[i] = 0
        
        return output   
        

def main(args):
    # use dictionary for fuzzy measure saving
    fm_value = {}
    fm_bin = [[0,0,0,0],[1,0,0,0],[0,1,0,0],[1,1,0,0],[0,0,1,0],[1,0,1,0],[0,1,1,0],[1,1,1,0],\
        [0,0,0,1],[1,0,0,1],[0,1,0,1],[1,1,0,1],[0,0,1,1],[1,0,1,1],[0,1,1,1],[1,1,1,1]]
    fm_value['bin'] = fm_bin
    fm_a = [[],[1],[2],[1,2],[3],[1,3],[2,3],[1,2,3],[4],[1,4],[2,4],[1,2,4],[3,4],[1,3,4],[2,3,4],[1,2,3,4]]
    fm_value['a'] = fm_a
    fm = [0, 0.2130, 0.3196, 0.5326, 0.0551, 0.2681, 0.3747, 0.5877, 0.4123, 0.6253, 0.7319, 0.9449, 0.4674, 0.6804, 0.7870, 1] 
    fm_value['fm'] = fm
    
    # Temporary partial evaluation function
    h_tmp = [0.2, 0.6, 0.23, 0.9]

    ic = fuzzy_integral(4,fm_value) # Initialization of fuzzy integral
    output = ic.choquet(h_tmp)
    print "Output"
    print(output)


if __name__ == '__main__':
    main(sys.argv)


