import unittest
import Util
import math

class TestUtil(unittest.TestCase):
    def testRotZTheta(self):
        v = [2,2]
        theta = math.pi/2
        r = [-2,2]
        self.assertAlmostEquals(r[0], Util.rotZTheta(v, theta)[0])
        self.assertAlmostEquals(r[1], Util.rotZTheta(v, theta)[1])
        self.assertAlmostEquals(v[0], Util.rotZTheta(r, -theta)[0])
        self.assertAlmostEquals(v[1], Util.rotZTheta(r, -theta)[1])

    
    def testCartFromPolar(self):
        m = 5
        theta = 0.6435011088
        self.assertAlmostEquals(4, Util.cartFromPolar(m, theta)[0])
        self.assertAlmostEquals(3, Util.cartFromPolar(m, theta)[1])


    def testPolarFromCart(self):
        r = [4,3]
        m, theta = Util.polarFromCart(r)
        self.assertAlmostEquals(5, m)
        self.assertAlmostEquals(0.6435011088, theta)


    def testCoulombForce(self):
        RC = -1
        OC = -1
        GC = 1
        r = [3,4]
        f = Util.coulombForce(r, OC, RC)

        self.assertEquals(True, f[0] < 0 and f[1] < 0)
        f = Util.coulombForce(r, GC, RC)
        self.assertEquals(True, f[0] > 0 and f[1] > 0)


    def testUndeadReckoning(self):
        a = [3,4]
        u = 1
        phi = math.pi/6
        xDot, phiDot = Util.undeadReckoning(a, u, phi)



    def testUnicycleTracking(self):
        theta = 0;
        Pd = [2,2]
        v = 0
        omega = 0
        v, omega = Util.unicycleTracking(Pd, v, omega, theta)
        self.assertEquals(True, v>0 and omega>0)
        Pd = [4,2]
        theta += omega * 0.01
        v, omega = Util.unicycleTracking(Pd, v, omega, theta)
        # There is no good way of testing this because I don't know what the
        # values should be
        