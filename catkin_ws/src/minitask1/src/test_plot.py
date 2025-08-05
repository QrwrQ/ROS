#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

x = np.linspace(0,2*np.pi,100)
y = np.sin(x)

fig = plt.figure(tight_layout=True)

plt.xlim((-1,11))
plt.ylim((-1,10))

point_ani,=plt.plot(x[0],y[0],"r-")
text_pt = plt.text(3.5,0.8,'',fontsize=16)

def update(num):

    xx=[0.0411975856276,0.0910625765567]
    yy=[2.44045139016e-05,3.30534207269e-05]
    point_ani.set_data(xx,yy)
    #text_pt.set_text("x=%.3f,y=%.3f"%(x[num],y[num]))
    return point_ani,text_pt,

ani=animation.FuncAnimation(fig=fig,func=update,frames=np.arange(0,200),
                            interval=800, blit=True)

plt.show()
