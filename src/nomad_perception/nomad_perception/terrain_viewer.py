"""Three synchronized layers, same XY grid; click a cell for exact values."""
import json
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid,Odometry
from .live_terrain import NAMES,PALETTE
from .semantic_input import stamp_ns

def main():
    import matplotlib
    matplotlib.use("Qt5Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.colors import ListedColormap,BoundaryNorm
    from matplotlib.ticker import MultipleLocator
    from matplotlib.patches import Patch
    from matplotlib.widgets import Button
    rclpy.init();node=rclpy.create_node('terrain_viewer');packets={};latest=[None];subscriptions=[]
    snapshot=node.declare_parameter('snapshot_file','').value
    vehicle=[None]
    subscriptions.append(node.create_subscription(Odometry,'/odom',lambda m:vehicle.__setitem__(0,m.pose.pose.position),10))
    q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def receive(k,m):
        key=stamp_ns(m);packet=packets.setdefault(key,{});packet[k]=m
        if len(packet)==4:latest[0]=packet
        for old in sorted(packets)[:-4]:packets.pop(old,None)
    for name in ['grid','height','slope','classes']:
        subscriptions.append(node.create_subscription(OccupancyGrid if name=='grid' else Image,'/nomad/terrain/'+name,lambda m,k=name:receive(k,m),q))
    fig,axes=plt.subplots(1,3,figsize=(18,8),sharex=True,sharey=True);fig.canvas.manager.set_window_title('NOMAD 2.5D WIDE: height | slope | class')
    cmaps=[plt.get_cmap('terrain').copy(),plt.get_cmap('turbo').copy(),ListedColormap(np.asarray(PALETTE)/255.)]
    for c in cmaps:c.set_bad('#e4e4e4')
    artists=[];vehicle_artists=[]
    for ax,title,cmap in zip(axes,['Ground height (m)','Ground slope (degrees)','Semantic class'],cmaps):
        artists.append(ax.imshow(np.ma.masked_all((2,2)),origin='lower',extent=(-1,1,-1,1),cmap=cmap,vmin=0,vmax=30 if title.startswith('Ground slope') else (9 if title=='Semantic class' else 1),interpolation='nearest'))
        vehicle_artists.append(ax.plot([],[],'co',markersize=5)[0])
        ax.set_title(title);ax.set_xlabel('odom X (m)');ax.set_aspect('equal')
    axes[0].set_ylabel('odom Y (m)');fig.colorbar(artists[0],ax=axes[0],shrink=.6);fig.colorbar(artists[1],ax=axes[1],shrink=.6)
    axes[2].legend(handles=[Patch(color=np.asarray(c)/255.,label='%d %s'%(i,n)) for i,(n,c) in enumerate(zip(NAMES,PALETTE))],loc='upper left',bbox_to_anchor=(1.01,1),fontsize=8)
    fig.subplots_adjust(left=.05,right=.86,bottom=.18,top=.85,wspace=.12)
    fig.suptitle('Waiting for measured semantic RGB-D map; grey = unknown')
    detail=fig.text(.05,.06,'Click a cell for height, slope and class. Toolbar: zoom / pan / home.',fontsize=10)
    state={'stamp':None,'data':None,'grid':None,'initial':True,'follow':False}
    def tick(_):
        if not rclpy.ok():
            plt.close(fig);return
        for _ in range(12):rclpy.spin_once(node,timeout_sec=0.)
        p=latest[0]
        if p is None or stamp_ns(p['grid'])==state['stamp']:return
        g=p['grid'];shape=(g.info.height,g.info.width)
        values=[]
        for k in ['height','slope','classes']:
            m=p[k];dtype=('<f4' if k!='classes' else '<i2')
            if m.is_bigendian:dtype=dtype.replace('<','>')
            values.append(np.frombuffer(bytes(m.data),dtype=dtype).reshape(shape))
        state.update(stamp=stamp_ns(g),data=values,grid=g)
        x,y=g.info.origin.position.x,g.info.origin.position.y;r=g.info.resolution
        extent=(x,x+shape[1]*r,y,y+shape[0]*r)
        for i,a in enumerate(artists):
            a.set_extent(extent);a.set_data(np.ma.masked_invalid(values[i]) if i<2 else np.ma.masked_less(values[i],0))
        valid=values[0][np.isfinite(values[0])]
        if len(valid):artists[0].set_clim(float(valid.min())-.05,float(valid.max())+.05)
        seen=np.argwhere(values[2]>=0)
        if state['initial']:
            axes[0].set_xlim(extent[:2]);axes[0].set_ylim(extent[2:]);state['initial']=False
        if state['follow'] and vehicle[0] is not None:
            axes[0].set_xlim(vehicle[0].x-15,vehicle[0].x+15)
            axes[0].set_ylim(vehicle[0].y-15,vehicle[0].y+15)
        for a in vehicle_artists:
            if vehicle[0] is not None:a.set_data([vehicle[0].x],[vehicle[0].y])
        for ax in axes:
            fine=abs(ax.get_xlim()[1]-ax.get_xlim()[0])/r<=100
            ax.xaxis.set_minor_locator(MultipleLocator(r if fine else 5.));ax.yaxis.set_minor_locator(MultipleLocator(r if fine else 5.))
            if fine:ax.grid(True,which='minor',color='black',alpha=.12,linewidth=.4)
            else:ax.grid(False,which='minor')
        fig.suptitle('Measured 2.5D grid | cell %.0f cm | height %d / slope %d / class %d cells'%(r*100,np.isfinite(values[0]).sum(),np.isfinite(values[1]).sum(),(values[2]>=0).sum()))
        if snapshot:
            from pathlib import Path
            Path(snapshot).parent.mkdir(parents=True,exist_ok=True)
            fig.savefig(snapshot,dpi=120)
    def full(_):
        state['follow']=False
        if state['grid'] is None:return
        g=state['grid'];x=g.info.origin.position.x;y=g.info.origin.position.y;r=g.info.resolution
        axes[0].set_xlim(x,x+g.info.width*r);axes[0].set_ylim(y,y+g.info.height*r);fig.canvas.draw_idle()
    def follow(_):
        state['follow']=not state['follow']
    full_button=Button(fig.add_axes([.05,.10,.10,.04]),'Full map')
    follow_button=Button(fig.add_axes([.17,.10,.12,.04]),'Follow 30m')
    full_button.on_clicked(full);follow_button.on_clicked(follow)
    def click(e):
        if e.inaxes not in axes or state['data'] is None or e.xdata is None:return
        g=state['grid'];x=int(np.floor((e.xdata-g.info.origin.position.x)/g.info.resolution));y=int(np.floor((e.ydata-g.info.origin.position.y)/g.info.resolution))
        if not 0<=x<g.info.width or not 0<=y<g.info.height:return
        h,s,c=[v[y,x] for v in state['data']]
        detail.set_text('Cell (%d,%d) XY=(%.2f, %.2f) | height %s m | slope %s deg | class %s'%(x,y,e.xdata,e.ydata,'unknown' if not np.isfinite(h) else '%.3f'%h,'unknown' if not np.isfinite(s) else '%.2f'%s,'unknown' if c<0 else NAMES[int(c)]));fig.canvas.draw_idle()
    fig.canvas.mpl_connect('button_press_event',click)
    animation=FuncAnimation(fig,tick,interval=200,cache_frame_data=False)
    try:plt.show()
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown() if rclpy.ok() else None
