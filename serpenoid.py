import matplotlib.pyplot as plt
from math import sin
from math import cos
from math import pi
import time as t

def plot(a,b,c):
    x=[0]
    y=[0]
    for i in range (1,9):
        x.append(x[i-1]+99*((1/8)*cos(a*cos(b*i/8)+c)))
        y.append(y[i-1]+99*((1/8)*sin(a*cos(b*i/8)+c)))
    i=i+1
    plt.plot(x[len(x)-9:len(x)],y[len(y)-9:len(y)],'o--')
    plt.axis([-200,200,-200,200])
    plt.show()
    t.sleep(0.3)
    c=c+0.1
    while True:
        x_round =[]
        y_round =[]
        x.append(x[len(x)-1]+99*((1/8)*cos(a*cos(b*i/8)+c)))
        y.append(y[len(y)-1]+99*((1/8)*sin(a*cos(b*i/8)+c)))
        x=x[1:len(x)]
        y=y[1:len(y)]
        for j in range (len(x)-9,len(x)):
            x_round.append(round(x[j]))
            y_round.append(round(y[j]))
        plt.plot(x,y,'o--')
        plt.plot(x_round,y_round,'ro-')
        plt.axis([-200,200,-200,200])
        plt.show()
        t.sleep(0.3)
        c=c+0.1
        i=i+1

plot(pi/3,2*pi,0)