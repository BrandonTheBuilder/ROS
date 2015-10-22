import math

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