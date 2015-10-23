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