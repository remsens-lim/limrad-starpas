import numpy as np

import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.patches import Circle
from matplotlib.transforms import Affine2D
import matplotlib.animation as animation

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def do_3d_projection(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)# renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)
        return 0
    
# def conv
def draw_axis(ax,lim=(-12,12)):
    ax.plot(xs=[lim[0],0],ys=[0,0],zs=[0,0],color='r',lw=1,ls='--')
    x_axis = Arrow3D([0, lim[1]], [0, 0], [0,0], mutation_scale=5, 
                    lw=1, arrowstyle="-|>", color="r")
    ax.add_artist(x_axis)
    ax.text(10,0,0.5,'x-Axis')
    
    ax.plot(xs=[0,0],ys=[lim[0],0],zs=[0,0],color='g',lw=1,ls='--')
    y_axis = Arrow3D([0, 0], [0, lim[1]], [0,0], mutation_scale=5, 
                     lw=1, arrowstyle="-|>", color="g")
    ax.add_artist(y_axis)
    ax.text(0.5,10,0,'y-Axis')
    
    ax.plot(xs=[0,0],ys=[0,0],zs=[lim[0],0],color='b',lw=1,ls='--')
    z_axis = Arrow3D([0, 0], [0, 0], [0,lim[1]], mutation_scale=5, 
                lw=1, arrowstyle="-|>", color="b")
    ax.add_artist(z_axis)
    ax.text(0.5,0,10,'z-Axis')
    
    ax.set_xlim(-12,12)
    ax.set_ylim(-12,12)
    ax.set_zlim(-12,12)
    return ax
    
def draw_vector(ax,point,spoint=(0,0,0),
                color='k',hscale=20,arrowstyle="-|>",lw=1):
    vector = Arrow3D([spoint[0], point[0]],
                     [spoint[1], point[1]],
                     [spoint[2], point[2]],
                     mutation_scale=hscale, 
                lw=lw, arrowstyle=arrowstyle, color=color)
    ax.add_artist(vector)
    return ax


def make_animation(fname,roll1, pitch1, roll2, pitch2, roll3 = None, pitch3=None,radius=30,interval=100):
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_aspect('equal')
    ax.set_xlim(-5,5)
    ax.set_ylim(-5,5)
    ax.axis('off')
    
    lx0 = ax.axline((0,0),(1,0),color='grey')
    ly0 = ax.axline((0,0),(0,1),color='grey')
    pitchlabel = ax.annotate("pitch",(-5,0.2),xytext=(-4.5,0.2),arrowprops=dict(width=1,headwidth=6,headlength=3,facecolor='black'),ha='left',va='center')
    rolllabel = ax.annotate("roll",(-0.2,-5),xytext=(-0.2,-4.5),arrowprops=dict(width=1,headwidth=6,headlength=3,facecolor='black'),ha='center',va='bottom',rotation=90)
    
    
    shiplabel = ax.annotate("Ship",(2,2),va='center',ha='center',rotation=-45)
    plane_a = Circle((0,0),radius=3)
    plane_a.set_facecolor('#8da0cb')
    plane_a.set_edgecolor('k')
    line_a, = ax.plot([0,0],[0,0],marker='.',color='#7570b3')
    
    radarlabel = ax.annotate("Radar",(.5,.5),va='center',ha='center',rotation=-45)
    plane_b = Circle((0,0),radius=1)
    plane_b.set_facecolor('#fc8d62')
    plane_b.set_edgecolor('k')
    line_b, = ax.plot([0,0],[0,0],marker='.',color='#d95f02')
    
    ax.add_patch(plane_a)
    ax.add_patch(plane_b)

    if roll3 is not None:
        Plabel = ax.annotate("P",(.2,.2),va='center',ha='center',rotation=-45)
        plane_c = Circle((0,0),radius=0.5)
        plane_c.set_facecolor('#66c2a5')
        plane_c.set_edgecolor('k')
        ax.add_patch(plane_c)
        line_c, = ax.plot([0,0],[0,0],marker='.',color='#1b9e77')
        

    for angle in range(1,6):
        pa = Circle((0,0),radius=radius*np.sin(np.deg2rad(float(angle))))
        pa.set_edgecolor((0.5,0.5,0.5,0.5))
        pa.set_facecolor((0,0,0,0))
        ax.add_patch(pa)
        ax.annotate(f"{angle}°",(radius*np.sin(np.deg2rad(float(angle))),.1))
    
    # ---------- 4. animation update ----------
    def update(frame):
        r1, p1 = np.radians([roll1[frame], pitch1[frame]])
        r2, p2 = np.radians([roll2[frame], pitch2[frame]])
        Xa,Ya = radius*np.sin(p1),-radius*np.sin(r1)
        Xb,Yb = radius*np.sin(p2),-radius*np.sin(r2)
        
        # update circle position
        trans_a = (Affine2D().translate(Xa,Ya))
        trans_b = (Affine2D().translate(Xb,Yb))
        plane_a.set_transform(trans_a + ax.transData)
        plane_b.set_transform(trans_b + ax.transData)
    
        shiplabel.set_position((1.5+Xa,1.5+Ya))
        radarlabel.set_position((0.5+Xb,0.5+Yb))
    
        # update line data
        line_a.set_xdata((0,Xa))
        line_a.set_ydata((0,Ya))
        line_b.set_xdata((0,Xb))
        line_b.set_ydata((0,Yb))

        if roll3 is not None:
            r3, p3 = np.radians([roll3[frame], pitch3[frame]])
            Xc,Yc = radius*np.sin(p3),-radius*np.sin(r3)
            trans_c = (Affine2D().translate(Xc,Yc))
            plane_c.set_transform(trans_c + ax.transData)
            Plabel.set_position((0.2+Xc,0.2+Yc))
            line_c.set_xdata((0,Xc))
            line_c.set_ydata((0,Yc))
            return plane_a,plane_b,plane_c
        return plane_a,plane_b
    
    ani = animation.FuncAnimation(fig, update, frames=len(roll1)-1, blit=True, interval=interval)
    
    # ---------- 5. save ----------
    ani.save(fname, writer='pillow', fps=20)   # GIF

