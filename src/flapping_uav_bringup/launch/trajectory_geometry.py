"""Pure geometry for horizontal trajectories inside the calibrated anchor polygon."""
import math

def convex_hull(points):
    points=sorted(set(points))
    def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower=[]
    for p in points:
        while len(lower)>=2 and cross(lower[-2],lower[-1],p)<=0:lower.pop()
        lower.append(p)
    upper=[]
    for p in reversed(points):
        while len(upper)>=2 and cross(upper[-2],upper[-1],p)<=0:upper.pop()
        upper.append(p)
    return lower[:-1]+upper[:-1]

def inside_margin(point,polygon,margin):
    for i,a in enumerate(polygon):
        b=polygon[(i+1)%len(polygon)];dx,dy=b[0]-a[0],b[1]-a[1]
        if (dx*(point[1]-a[1])-dy*(point[0]-a[0]))/math.hypot(dx,dy)<margin-1e-9:return False
    return True

def geometry(scene,uwb,margin,scale,line_length,figure8_axis='x',lobe_aspect_ratio=None):
    if not scene.get('anchor_coordinates_configured',False):raise ValueError('Configure anchor coordinates before using trajectories')
    anchors=scene.get('anchor_positions_m',[])
    if len(anchors)!=12 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in anchors):
        raise ValueError('Expected four finite anchor XYZ coordinates in metres')
    if uwb.get('position_scale',1.)!=1.:raise ValueError('Trajectory geometry requires UWB positions in metres, position_scale=1')
    if not math.isfinite(margin) or margin<=0 or not math.isfinite(scale) or not 0<scale<=1 or not math.isfinite(line_length) or line_length<=0:
        raise ValueError('Invalid trajectory margin, fit scale or line length')
    if figure8_axis not in ('auto','x','y'):
        raise ValueError('figure8_axis must be auto, x or y')
    if lobe_aspect_ratio is not None and (not math.isfinite(lobe_aspect_ratio) or lobe_aspect_ratio<=0):
        raise ValueError('figure8_lobe_aspect_ratio must be positive')
    yaw=uwb['uwb_to_map_yaw_rad'];shift=uwb['uwb_to_map_translation_m'];sign=uwb.get('uwb_y_sign',1.)
    c,s=math.cos(yaw),math.sin(yaw);points=[]
    for i in range(4):
        x,y=anchors[3*i],anchors[3*i+1]*sign
        points.append((c*x-s*y+shift[0],s*x+c*y+shift[1]))
    polygon=convex_hull(points)
    if len(polygon)<3:raise ValueError('Anchor polygon is degenerate')
    xmin,xmax=min(p[0] for p in polygon),max(p[0] for p in polygon)
    ymin,ymax=min(p[1] for p in polygon),max(p[1] for p in polygon)
    center=((xmin+xmax)/2,(ymin+ymax)/2)
    hx=((xmax-xmin)/2-margin)*scale;hy=((ymax-ymin)/2-margin)*scale
    if hx<=0 or hy<=0 or not inside_margin(center,polygon,margin):raise ValueError('Anchor area is too narrow for the requested margin')
    if figure8_axis=='auto':figure8_axis='y' if hy>hx else 'x'
    if lobe_aspect_ratio is not None:
        # Each lobe spans the major half-extent and the entire minor width.
        # Only shrink the available envelope; never grow toward a boundary.
        if figure8_axis=='x':
            hx=min(hx,2*hy*lobe_aspect_ratio);hy=hx/(2*lobe_aspect_ratio)
        else:
            hy=min(hy,2*hx*lobe_aspect_ratio);hx=hy/(2*lobe_aspect_ratio)
    for unused in range(100):
        if all(inside_margin((center[0]+sx*hx,center[1]+sy*hy),polygon,margin) for sx in [-1,1] for sy in [-1,1]):break
        hx*=.95;hy*=.95
    else:raise ValueError('Unable to fit a horizontal trajectory inside the anchor polygon')
    if not all(inside_margin((center[0]+sx*line_length/2,center[1]),polygon,margin) for sx in [-1,1]):
        raise ValueError('Requested line length does not fit inside the calibrated area')
    return {'center':list(center),'figure8_half_extents':[hx,hy],'figure8_axis':figure8_axis,
            'line_half_extents':[line_length/2,0.], 'polygon':[v for p in polygon for v in p]}
