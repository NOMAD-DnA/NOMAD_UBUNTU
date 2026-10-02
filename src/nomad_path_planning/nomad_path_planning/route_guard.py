"""Shared local route direction checks, independent of waypoint orientation."""
import math


def route_is_behind(pose, points, lookahead=1.0, margin=.2):
    """Is the next ~1m of route in the vehicle's rear half-plane?

    Project onto the route, then follow its ordering. The final goal and path yaw
    do not determine travel direction. A lateral turn or a route that first goes
    forward before bending back is not rejected as a rearward route.
    """
    if not points:
        return False
    if not all(math.isfinite(v) for v in pose) or not all(math.isfinite(v) for p in points for v in p[:2]):
        return True
    closest = None
    for i,(a,b) in enumerate(zip(points,points[1:])):
        dx,dy = b[0]-a[0],b[1]-a[1]
        length2 = dx*dx+dy*dy
        if length2 < 1e-12:
            continue
        t = max(0.,min(1.,((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/length2))
        projected = (a[0]+t*dx,a[1]+t*dy)
        error = math.hypot(pose[0]-projected[0],pose[1]-projected[1])
        if closest is None or error < closest[0]:
            closest = error,i,projected
    target = points[-1]
    if closest is not None:
        _,i,current = closest
        remaining = lookahead
        for endpoint in points[i+1:]:
            length = math.hypot(endpoint[0]-current[0],endpoint[1]-current[1])
            if length >= remaining:
                ratio = remaining/length
                target = (current[0]+ratio*(endpoint[0]-current[0]),
                          current[1]+ratio*(endpoint[1]-current[1]))
                break
            remaining -= length
            current = endpoint
    along = ((target[0]-pose[0])*math.cos(pose[2])
             +(target[1]-pose[1])*math.sin(pose[2]))
    return along < -margin


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def path_is_behind(pose, path):
    if pose is None or path is None:
        return False
    p = pose.pose
    return route_is_behind((p.position.x,p.position.y,yaw(p.orientation)),
                           [(p.pose.position.x,p.pose.position.y) for p in path.poses])


def path_moves_forward(path):
    if path is None or len(path.poses) < 2:
        return False
    a,b = path.poses[0].pose,path.poses[1].pose
    heading = yaw(a.orientation)
    return ((b.position.x-a.position.x)*math.cos(heading)
            +(b.position.y-a.position.y)*math.sin(heading)) > 1e-6
