"""Endpoint-aligned SE(2) lattice with cached cubic-Hermite motion primitives.

The lattice is rooted at the initial rear-axle pose; translations repeat at a fixed
spacing and orientations at 16 bins. Curvature is checked before admitting an edge.
"""
import math
from .hybrid_astar import HybridAStar,wrap


class StateLattice(HybridAStar):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.spacing=.4
        self.library={}

    def primitive(self, heading, turn, length, sample_step):
        key=(heading,turn,length,sample_step)
        if key in self.library:
            return self.library[key]
        a=heading*math.pi/8; b=a+turn*math.pi/8
        mid=(a+b)/2
        dx=round(length*math.cos(mid)/self.spacing)*self.spacing
        dy=round(length*math.sin(mid)/self.spacing)*self.spacing
        distance=math.hypot(dx,dy)
        if not distance:
            return []
        # Cubic Bezier with endpoint tangents fixed to lattice orientations.
        controls=((0.,0.),(distance*math.cos(a)/3,distance*math.sin(a)/3),
                  (dx-distance*math.cos(b)/3,dy-distance*math.sin(b)/3),(dx,dy))
        output=[]
        count=max(24,math.ceil(distance/sample_step)*2)
        for i in range(count+1):
            t=i/count; u=1-t
            x,y=[u**3*controls[0][d]+3*u*u*t*controls[1][d]+3*u*t*t*controls[2][d]+t**3*controls[3][d] for d in (0,1)]
            vx,vy=[3*u*u*(controls[1][d]-controls[0][d])+6*u*t*(controls[2][d]-controls[1][d])+3*t*t*(controls[3][d]-controls[2][d]) for d in (0,1)]
            ax,ay=[6*u*(controls[2][d]-2*controls[1][d]+controls[0][d])+6*t*(controls[3][d]-2*controls[2][d]+controls[1][d]) for d in (0,1)]
            speed=math.hypot(vx,vy)
            if speed<1e-9 or abs(vx*ay-vy*ax)/speed**3 > self.curvature*.98:
                output=[]; break
            output.append((x,y,math.atan2(vy,vx)))
        self.library[key]=output
        return output

    def plan(self, request, corridor=None):
        self.heading_origin=request.start[2]
        return super().plan(request,corridor)

    def successors(self, pose, grid):
        root=self.heading_origin
        h=round(wrap(pose[2]-root)*8/math.pi)%16
        rear=(pose[0]-self.offset*math.cos(pose[2]),pose[1]-self.offset*math.sin(pose[2]))
        c,s=math.cos(root),math.sin(root)
        for length in (.8,1.2,1.6):
            for turn in (-1,0,1):
                primitive=self.primitive(h,turn,length,min(.06,grid.r*.3))
                if primitive:
                    yield [(rear[0]+c*x-s*y+self.offset*math.cos(yaw+root),
                            rear[1]+s*x+c*y+self.offset*math.sin(yaw+root),wrap(yaw+root))
                           for x,y,yaw in primitive[1:]]
