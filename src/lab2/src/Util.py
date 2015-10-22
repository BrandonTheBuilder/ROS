import math
import MatrixMath as MM

def rotZTheta(v, theta):
    x = v[0]*math.cos(theta) - v[1]*math.sin(theta)
    y = v[0]*math.sin(theta) + v[1]*math.cos(theta)
    return [x,y]


def cartFromPolar(d, theta):
    x = d*math.cos(theta)
    y = d*math.sin(theta)
    return [x,y]


def polarFromCart(v):
    d = math.pow(math.pow(v[0],2)+math.pow(v[1],2), 0.5)
    theta = math.atan2(v[1], v[0])
    return (d, theta)


def coulombForce(r, q1, q2):
    k = 10 #8.98 * math.pow(10,9)
    m = math.pow(math.pow(r[0],2)+math.pow(r[1],2), 0.5)
    u = [r[0]/m, r[1]/m]
    c = k*q1*q2/math.pow(m,2)
    return [-c*u[0], -c*u[1]] 


def scanAngleMod(theta):
    return math.cos(theta/2)


def undeadReckoning(a, u, phi):
    
    if u == 0:
        ra = rotZTheta(a, phi)
        d, theta = polarFromCart(a)
        return (a[0]/math.cos(phi), 5)
    A = [[math.cos(phi), -u*math.sin(phi)],
        [math.sin(phi), u*math.cos(phi)]]
    print(A)
    print phi
    B = [[a[0]],[a[1]]]
    x = MM.Multiply(MM.Inv(A), B)
    return (x[0][0], x[1][0])