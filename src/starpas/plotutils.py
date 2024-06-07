import matplotlib.pyplot as plt
import matplotlib.dates as mdates

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

