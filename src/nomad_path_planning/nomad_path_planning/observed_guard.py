"""Observed-space execution guard; no world geometry or reference-route input.

Occupancy >=80 already includes obstacle inflation: do not inflate it again.
Unknown newly entered cells are checked over the swept vehicle rectangle.
The grid's bootstrap assumption remains an assumption, not a measured observation.
"""
import math
from .history_recovery import free


def known_prefix(start, path, grid):
    """Connected centre-line prefix from current pose to nearest route projection.

    Preserve interpolated vehicle yaw, and sample every segment (not just vertices).
    The final mission goal and published global route remain unchanged.
    """
    if len(path) < 2 or not free(grid, *start[:2]):
        return []
    best = None
    for i, (a, b) in enumerate(zip(path, path[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        den = dx*dx+dy*dy
        t = max(0., min(1., ((start[0]-a[0])*dx+(start[1]-a[1])*dy)/den)) if den else 0.
        q = (a[0]+t*dx, a[1]+t*dy)
        error = math.hypot(q[0]-start[0], q[1]-start[1])
        if best is None or error < best[0]:
            yaw_a = a[2] if len(a)>2 else math.atan2(dy,dx)
            yaw_b = b[2] if len(b)>2 else yaw_a
            yaw = yaw_a+t*math.atan2(math.sin(yaw_b-yaw_a),math.cos(yaw_b-yaw_a))
            best = (error, i, (*q,yaw))
    _, i, q = best
    output = [start]
    for endpoint in [q]+list(path[i+1:]):
        a = output[-1]
        d = math.hypot(endpoint[0]-a[0], endpoint[1]-a[1])
        if d < 1e-9:
            continue
        n = max(1, math.ceil(d/(grid['resolution']*.25)))
        yaw_b = endpoint[2] if len(endpoint)>2 else math.atan2(endpoint[1]-a[1],endpoint[0]-a[0])
        delta = math.atan2(math.sin(yaw_b-a[2]), math.cos(yaw_b-a[2]))
        for j in range(1,n+1):
            t=j/n
            p=(a[0]+t*(endpoint[0]-a[0]),a[1]+t*(endpoint[1]-a[1]),a[2]+t*delta)
            if not free(grid,*p[:2]):
                return output
            output.append(p)
    return output


def footprint_known(grid, x, y, radius):
    if not free(grid,x,y):
        return False
    r=grid['resolution']; ox=grid['origin_x'];oy=grid['origin_y']
    # Include every cell that may intersect the disc, including corner contact.
    margin=radius+r/math.sqrt(2)
    for cy in range(math.floor((y-radius-oy)/r), math.floor((y+radius-oy)/r)+1):
        for cx in range(math.floor((x-radius-ox)/r), math.floor((x+radius-ox)/r)+1):
            if math.hypot(ox+(cx+.5)*r-x,oy+(cy+.5)*r-y)>margin:
                continue
            if not (0<=cx<grid['width'] and 0<=cy<grid['height']):
                return False
            if grid['data'][cy*grid['width']+cx]<0:
                return False
    return True


def footprint_cells(grid, pose, half_length=.52, half_width=.40):
    """Grid cells intersecting an oriented bounding rectangle (SAT test).

    Includes body/wheels at planning steering limit .4rad. This is an execution
    envelope, not a new inflated occupancy map.
    """
    x,y,yaw=pose; c=math.cos(yaw);s=math.sin(yaw);r=grid['resolution']
    ox=grid['origin_x'];oy=grid['origin_y']
    ex=abs(c)*half_length+abs(s)*half_width
    ey=abs(s)*half_length+abs(c)*half_width
    cell_extent=.5*r*(abs(c)+abs(s))
    cells=set()
    for cy in range(math.floor((y-ey-oy)/r), math.floor((y+ey-oy)/r)+1):
        for cx in range(math.floor((x-ex-ox)/r), math.floor((x+ex-ox)/r)+1):
            dx=ox+(cx+.5)*r-x;dy=oy+(cy+.5)*r-y
            if (abs(c*dx+s*dy)<=half_length+cell_extent and
                    abs(-s*dx+c*dy)<=half_width+cell_extent):
                cells.add((cx,cy))
    return cells


def observed_distance(grid, pose, steer, direction, wheelbase=.72,
                      offset=.36, half_length=.52, half_width=.40, horizon=2.):
    """Forward: check newly entered cells; reverse: require known whole footprint.

    The initial exemption is fixed for this entire predicted arc, never advanced
    along it, and applies only to unknown cells. Nothing in the map is changed.
    Known blocked centres and out-of-map cells are never exempted.
    """
    width=grid['width'];height=grid['height'];data=grid['data']
    initial=footprint_cells(grid,pose,half_length,half_width) if direction>0 else set()
    exempt={(cx,cy) for cx,cy in initial if 0<=cx<width and 0<=cy<height and data[cy*width+cx]<0}
    def permitted(p):
        x,y,_=p;r=grid['resolution']
        cx=math.floor((x-grid['origin_x'])/r);cy=math.floor((y-grid['origin_y'])/r)
        if not (0<=cx<width and 0<=cy<height) or data[cy*width+cx]>=80:
            return False
        for cell in footprint_cells(grid,p,half_length,half_width):
            i,j=cell
            if not (0<=i<width and 0<=j<height):return False
            if data[j*width+i]<0 and cell not in exempt:return False
        return True
    x,y,yaw=pose
    if not permitted(pose):return 0.
    x-=offset*math.cos(yaw);y-=offset*math.sin(yaw)
    step=min(.02,grid['resolution']*.1);walked=0.;k=math.tan(steer)/wheelbase
    while walked<horizon-1e-9:
        ds=min(step,horizon-walked);signed=direction*ds;next_yaw=yaw+signed*k
        if abs(k)>1e-9:
            x+=(math.sin(next_yaw)-math.sin(yaw))/k
            y+=(-math.cos(next_yaw)+math.cos(yaw))/k
        else:
            x+=signed*math.cos(yaw);y+=signed*math.sin(yaw)
        yaw=next_yaw
        if not permitted((x+offset*math.cos(yaw),y+offset*math.sin(yaw),yaw)):
            return max(0.,walked-step)
        walked+=ds
    return max(0.,walked-step)


def speed_limit(distance, latency, deceleration, margin):
    """Solve v*latency + v²/(2*a) + margin <= known distance."""
    if not all(math.isfinite(v) for v in (distance,latency,deceleration,margin)) or deceleration<=0 or min(latency,margin)<0:
        return 0.
    usable=max(0.,distance-margin)
    return max(0.,math.sqrt((deceleration*latency)**2+2*deceleration*usable)-deceleration*latency)
