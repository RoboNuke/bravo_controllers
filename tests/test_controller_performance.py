"""
#!/usr/bin/env python
PKG = 'bravo_controllers'
#import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import sys
import unittest

## A sample python unit test
class TestBareBones(unittest.TestCase):
    ## test 1 == 1
    def test_one_equals_one(self): # only functions with 'test_'-prefix will be run!
        self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_bare_bones', TestBareBones)
"""

import sys
sys.path.insert(0, "/home/hunter/catkin_ws/src/bravo_controllers/scripts")
from controller_performance import ControllerPerformance
import pytest
from math import fabs
import numpy as np


np.set_printoptions(precision=3, suppress=True)
class TestControllerPerformance:
    eps = 0.001
    cp = ControllerPerformance()
    rng = np.random.default_rng(69)
    def assert_circle_traj(self, traj, r, dy, n):
        
        maxs = [0.0, 0.0, 0.0]
        mins = [0.0, 0.0, 0.0]
        for pt in traj:
            for i in range(3):
                if pt[i] > maxs[i]:
                    maxs[i] = pt[i]
                if pt[i] < mins[i]:
                    mins[i] = pt[i]
        print(maxs, mins)
        assert len(traj) == n
        assert fabs(mins[0] + r) <= self.eps
        assert fabs(mins[1] + dy/2) <= self.eps
        assert fabs(mins[2] + r) <= self.eps

        assert fabs(maxs[0] - r) <= self.eps
        assert fabs(maxs[1] - dy/2) <= self.eps
        assert fabs(maxs[2] - r) <= self.eps

    def test_getCircleTraj(self):
        r = 1.0
        dy = 1.0
        n = 4
        traj = self.cp.getCircleTraj(r, dy, n)
        self.assert_circle_traj(traj, r, dy, n)
        n2 = 50
        r2 = 0.5
        dy2 = 0.5
        traj2 = self.cp.getCircleTraj(r2, dy2, n2)
        self.assert_circle_traj(traj2, r2, dy2, n2)
    
    def assert_oval_traj(self, traj, dx, dy, dz, n):
        maxs = [0.0, 0.0, 0.0]
        mins = [0.0, 0.0, 0.0]
        for pt in traj:
            for i in range(3):
                if pt[i] > maxs[i]:
                    maxs[i] = pt[i]
                if pt[i] < mins[i]:
                    mins[i] = pt[i]

        assert len(traj) == n
        assert mins[0] + dx/2.0 <= self.eps
        assert mins[1] + dy/2.0 <= self.eps
        assert mins[2] + dz/2.0 <= self.eps

        assert maxs[0] - dx/2.0 <= self.eps
        assert maxs[1] - dy/2.0 <= self.eps
        assert maxs[2] - dz/2.0 <= self.eps

    def test_getOvalTraj_1(self):
        dx = 1.0
        dy = 0.25
        dz = 0.5
        n = 4
        traj = self.cp.getOvalTraj(dx, dy, dz, n)
        self.assert_oval_traj(traj, dx, dy, dz, n)
        n2 = 50
        dx2 = 0.5
        dy2 = 0.5
        dz2 = 0.75
        traj2 = self.cp.getOvalTraj(dx2, dy2, dz2, n2)
        self.assert_oval_traj(traj2, dx2, dy2, dz2, n2)

    def assert_square_traj(self, traj, sx, sy, sz, n):
        assert n == len(traj)
        slen = n // 4
        dx = sx / slen
        dy = sy / slen
        dz = sz / slen
        for i in range(slen):
            rwI = i
            lwI = i + 2 * slen
            twI = i + slen
            bwI = i + 3 * slen
            pt_rw = traj[rwI]
            pt_tw = traj[twI]
            pt_lw = traj[lwI]
            pt_bw = traj[bwI]

            # right and left wall have constant x values 
            assert pt_rw[0] == sx/2.0
            assert pt_lw[0] == -sx/2.0

            assert pt_bw[2] == -sz/2.0
            assert pt_tw[2] == sz/2.0

            assert pt_tw[1] == sy/2.0
            assert pt_bw[1] == -sy/2.0

            if i > 0:
                assert pt_bw[0] - traj[bwI-1][0] - dx <= self.eps
                assert pt_tw[0] - traj[twI-1][0] + dx <= self.eps

                assert pt_lw[1] - traj[lwI-1][1] + dy <= self.eps
                assert pt_rw[1] - traj[rwI-1][1] - dy <= self.eps

                assert pt_lw[2] - traj[lwI-1][2] + dz <= self.eps
                assert pt_rw[2] - traj[rwI-1][2] - dz <= self.eps


    def test_getSquareTraj(self):
        n = 100
        sx = 25
        sy = 50
        sz = 75
        traj = self.cp.getSquareTraj(sx, sy, sz, n)
        self.assert_square_traj(traj, sx, sy, sz, n)

        n2 = 50
        sx2 = 12.5
        sy2 = -25.0
        sz2 = 37.5
        traj2 = self.cp.getSquareTraj(sx2, sy2, sz2, n2)
        self.assert_square_traj(traj2, sx2, sy2, sz2, n2)
    
    def test_getTrajStats(self):
        n = 4
        n_traj = 5
        traj = self.cp.getCircleTraj(1.0, 0.0, n) # unit cicle
        real_traj = np.zeros((n_traj, len(traj), 3))
        mean = np.array([1.0, 2.5, 0.0])
        covar = np.array([[1.0, 0.0, 0.0],
                          [00.0, 10.0, 0.0], 
                          [00.0, 0.0, 0.0]])
        error = self.rng.multivariate_normal(mean, covar, (n_traj,n))
        for i in range(n_traj):
            real_traj[i] = error[i] + traj 

        stats = []
        # Avg per point
        avg_pt = np.average(real_traj, axis=0)
        stats.append(avg_pt)
        # std dev per point
        std_pt = np.std(real_traj, axis=0)
        stats.append(std_pt)
        # l2 error
        l2_error = np.sqrt( error[:,:,0] ** 2 + error[:,:,1]**2 + error[:,:,2] ** 2)
        l2_avg_pt = np.average(l2_error, axis=0)
        stats.append(l2_avg_pt)
        l2_std_pt = np.std(l2_error, axis=0)
        stats.append(l2_std_pt)
        l2_tot_avg = np.average(l2_error) 
        stats.append(l2_tot_avg)
        l2_tot_std = np.std(l2_error) 
        stats.append(l2_tot_std)
        # summed traj error 
        sum_error = np.sum(error, axis=1)
        l2_sum_error = np.sum(l2_error, axis=1)
        # avg summed traj error
        avg_sum_error = np.average(sum_error, axis=0)
        stats.append(avg_sum_error)
        # std dev summed traj error 
        std_sum_error = np.std(sum_error, axis=0)
        stats.append(std_sum_error)
        stats.append(np.average(l2_sum_error))
        stats.append(np.std(l2_sum_error))
        stats.append(np.array([np.average(error[:,:,0]),np.average(error[:,:,1]),np.average(error[:,:,2])]))
        stats.append(np.array([np.std(error[:,:,0]),np.std(error[:,:,1]),np.std(error[:,:,2])]))

        calc_stats = self.cp.getTrajStats(traj, real_traj)
        assert len(calc_stats) == len(stats)
        for i in range(len(calc_stats)):
            assert np.all(np.isclose(calc_stats[i], stats[i]))
        
        #self.cp.plotTrajStats(traj, calc_stats)
        self.cp.tabTrajStats(calc_stats)
        assert 1 == 0

    
    def test_getPt2PtStats(self):
        a = np.array([0.0, 0.0, 0.0])
        b = np.array([1.0, 2.0, 3.0])
        real_a = np.array([[10.0, 5.0,  15.0],
                           [12.0, 7.0,  78.0],
                           [23.0, 63.0, 64.0],
                           [32.0, 45.0, 32.0]])
    
        real_b = np.array([[11.0, 07.0, 18.0],
                           [13.0, 09.0, 81.0],
                           [24.0, 65.0, 67.0],
                           [33.0, 47.0, 35.0]])
        
        calc_stats = self.cp.getPt2PtStats(a, b, real_a, real_b)

        # CHECK XYZ 
        assert calc_stats[0,0] - 19.25 <= self.eps
        assert calc_stats[1,0] - 8.870597 <= self.eps

        assert calc_stats[0,1] - 30.0 <= self.eps
        assert calc_stats[1,1] - 24.839484 <= self.eps

        assert calc_stats[0,2] - 47.25 <= self.eps
        assert calc_stats[1,2] - 24.9937 <= self.eps

        assert calc_stats[2,0] - 19.25 <= self.eps
        assert calc_stats[3,0] - 8.870597 <= self.eps

        assert calc_stats[2,1] - 30.0 <= self.eps
        assert calc_stats[3,1] - 24.839484 <= self.eps

        assert calc_stats[2,2] - 47.25 <= self.eps
        assert calc_stats[3,2] - 24.9937 <= self.eps

        assert calc_stats[4,0] - 19.25 <= self.eps
        assert calc_stats[5,0] - 8.870597 <= self.eps

        assert calc_stats[4,1] - 30.0 <= self.eps
        assert calc_stats[5,1] - 24.839484 <= self.eps

        assert calc_stats[4,2] - 47.25 <= self.eps
        assert calc_stats[5,2] - 24.9937 <= self.eps

        # for the totals
        assert calc_stats[0,3] - 63.615 <= self.eps
        assert calc_stats[1,3] - 27.868514 <= self.eps

        assert calc_stats[2,3] - 63.615 <= self.eps
        assert calc_stats[3,3] - 27.868514 <= self.eps

        assert calc_stats[4,3] - 63.615 <= self.eps
        assert calc_stats[5,3] - 27.868514 <= self.eps

        self.cp.tabPt2PtStats(calc_stats)
    
    def test_tabMinErrorEval(self):
        stats = np.array([0.0, 1.0, 2.0])
        self.cp.tabMinErrorEval(stats)
        #raise NotImplementedError
    """
    def test_runMinErr4MotionEval(self):
        raise NotImplementedError
    
    def test_runFullMinError4MotionEval(self):
        raise NotImplementedError
    
    def test_runPt2PtEval(self):
        raise NotImplementedError
    
    def test_runFullPt2PtEval(self):
        raise NotImplementedError
    
    def test_runTrajEval(self):
        raise NotImplementedError
    
    def test_runFullTrajEvals(self):
        raise NotImplementedError
    """
if __name__ == "__main__":
    tcp = TestControllerPerformance()
    tcp.test_getTrajStats()