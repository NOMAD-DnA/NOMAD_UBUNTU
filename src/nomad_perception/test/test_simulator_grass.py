"""Simulation oracle must identify surfaces, never a whole grass bounding box."""
from pathlib import Path
import numpy as np
import pytest
from nomad_perception.simulator_grass import SimulatorGrass


def test_tall_grass_without_colour_ground_or_confirmation_and_no_bbox_clearing():
    oracle = SimulatorGrass([[[0, 0, 0], [1, 0, 0], [0, 0, 3]]])
    points = [[.1, 0, 2], [.2, .004, 1], [.9, 0, 2.8], [.1, .02, 1], [np.nan, 0, 0]]
    assert oracle.matches(points).tolist() == [True, True, False, False, False]
    assert not oracle.matches([]).any()


def test_selected_world_pose_scale_and_non_grass_models(tmp_path):
    (tmp_path/'leaf.obj').write_text('v 0 0 0\nv 1 0 0\nv 0 0 1\nf 1 2 3\n')
    world=tmp_path/'world.sdf'
    world.write_text('''<sdf><world name="test"><model name="undergrowth"><static>true</static>
    <pose>5 6 0 0 0 1.5707963267948966</pose><link name="grass"><visual name="mesh">
    <geometry><mesh><uri>leaf.obj</uri><scale>2 2 2</scale></mesh></geometry>
    </visual></link></model><model name="rock"><static>true</static><link name="rock">
    <visual name="mesh"><geometry><mesh><uri>leaf.obj</uri></mesh></geometry></visual>
    </link></model></world></sdf>''')
    oracle=SimulatorGrass.from_world(world)
    assert oracle.matches([[5,6.2,1.5], [.1,0,.5]]).tolist()==[True,False]
    world.write_text(world.read_text().replace('<link name="grass">','<link name="grass"><collision/>'))
    with pytest.raises(ValueError,match='collidable'):
        SimulatorGrass.from_world(world)


def test_map04_all_authored_grass_faces_including_tall_tips():
    root=Path(__file__).resolve().parents[2]/'nomad_gazebo/maps/04_vio_flat_loop'
    oracle=SimulatorGrass.from_world(root/'worlds/forest.sdf')
    assert len(oracle.triangles)>0
    # Sample every triangle at its centre and one tip, including all 1,800 clumps.
    for points in [oracle.triangles.mean(axis=1),oracle.triangles[:,2]]:
        assert oracle.matches(points).all()
    assert oracle.matches([[26.32895337,-2.86692869,5]]).tolist()==[False]


def test_empty_bare_world_labels_nothing(tmp_path):
    world=tmp_path/'bare.sdf';world.write_text('<sdf><world name="empty"/></sdf>')
    assert not SimulatorGrass.from_world(world).matches([[0,0,0]]).any()


def test_geometry_pipeline_preserves_other_observations_and_unseen_cells(monkeypatch):
    from types import SimpleNamespace as NS
    from builtin_interfaces.msg import Time
    import nomad_perception.camera_terrain as module
    from nomad_perception.observed_grid import ObservedGrid
    camera=module.CameraTerrain.__new__(module.CameraTerrain)
    header=NS(stamp=Time(sec=10),frame_id='optical')
    depth=NS(header=header,width=1,height=1,encoding='32FC1',is_bigendian=0,
             step=4,data=np.array([[2]],dtype='<f4').tobytes())
    info=NS(header=header,width=1,height=1,d=[],k=[1,0,0,0,1,0,0,0,1])
    tf=NS(transform=NS(rotation=NS(x=0,y=0,z=0,w=1),translation=NS(x=0,y=0,z=0)))
    grid=ObservedGrid(size=8)
    camera.node=NS(fault=None,fresh=lambda s:True,frame='odom',
                   buffer=NS(lookup_transform=lambda *a:tf),grid=grid)
    camera.depth,camera.info,camera.last_stamp=[depth],{'depth':info},-np.inf
    camera.mode,camera.stride,camera.max_range='geometry',1,6
    camera.patch_radius,camera.geometry_min_samples=.35,6
    camera.slope_limit,camera.roughness_limit,camera.step_limit=.35,.04,.18
    camera.slope_enabled,camera.objects_only=False,True
    # Three grass cells: pure grass, grass+rock, grass+unknown. A separate rock.
    grass=np.array([[.1,.1,1.5],[.5,.1,1.5],[.9,.1,1.5]])
    solids=np.array([[.51,.11,.3],[.91,.11,.3],[1.3,.1,.3]])
    camera.simulator_grass=SimulatorGrass([[[0,0,1.5],[2,0,1.5],[0,1,1.5]]])
    camera.ground=NS(cells={},history={},object_costs=lambda p,h:(
        np.array([[.5,.1],[.9,.1],[1.3,.1]]),np.array([100,-1,100],dtype=np.int8)))
    camera.obstacle_height=.12
    camera.grass_cost=35
    monkeypatch.setattr(module,'depth_points',lambda *a:np.vstack([grass,solids]))
    monkeypatch.setattr(module,'surface_costs',lambda *a,**k:(np.empty((0,2)),np.array([]),np.empty((0,3))))
    camera.process_geometry()
    assert camera.last_stamp==10
    for x,expected in [(.1,35),(.5,100),(.9,-1),(1.3,100),(2.1,-1)]:
        ix,iy=grid.cell(x,.1)
        assert grid.terrain[iy,ix]==expected
    assert 'SIMULATOR mesh oracle' in camera.status


def test_grass_cost_survives_navigation_maps_expiry_and_reclassification():
    from nomad_perception.observed_grid import ObservedGrid
    grid=ObservedGrid(size=8, inflation=.4)
    grid.terrain.fill(0)
    grid.terrain_seen.fill(10.)
    point=np.array([[.1,.1]])
    x,y=grid.cell(.1,.1)
    grid.integrate_surface_costs(point,np.array([35],dtype=np.int8),10.,passable=True)
    global_map,local_map=grid.navigation_maps_without_lidar_obstacles(10.,30.,2.)
    assert global_map[y,x]==local_map[y,x]==35
    assert local_map[y,x+1]==0  # bare ground retains its cheaper cost
    global_map,local_map=grid.navigation_maps_without_lidar_obstacles(13.,30.,2.)
    assert global_map[y,x]==35 and local_map[y,x]==-1
    assert grid.navigation_maps_without_lidar_obstacles(41.,30.,2.)[0][y,x]==-1
    grid.ensure_bounds(-6,-6)
    x,y=grid.cell(.1,.1)
    assert grid.terrain_passable[y,x]
    # A newer ambiguous observation revokes traversability; an old grass vote cannot restore it.
    grid.integrate_surface_costs(point,np.array([35],dtype=np.int8),14.)
    grid.integrate_surface_costs(point,np.array([35],dtype=np.int8),10.,passable=True)
    assert not grid.terrain_passable[y,x]
    assert grid.navigation_maps_without_lidar_obstacles(14.,30.,2.)[0][y,x]==-1


def test_grass_preference_does_not_reduce_obstacle_clearance_cost():
    from nomad_perception.observed_grid import ObservedGrid
    grid=ObservedGrid(size=8,inflation=.4,soft_inflation_radius=1.2)
    grid.terrain.fill(0);grid.terrain_seen.fill(10.)
    grid.integrate_surface_costs(np.array([[.1,.1]]),np.array([35],dtype=np.int8),10.,passable=True)
    grid.integrate_surface_costs(np.array([[.9,.1]]),np.array([100],dtype=np.int8),10.)
    x,y=grid.cell(.1,.1)
    for layer in grid.navigation_maps_without_lidar_obstacles(10.,30.,2.):
        assert 35<layer[y,x]<80
        assert layer[y,x+4]==100


def test_fresh_simulator_grass_keeps_cost_at_unknown_boundary_but_not_obstacles():
    from nomad_perception.observed_grid import ObservedGrid
    grid=ObservedGrid(size=8,inflation=.6)
    point=np.array([[.1,.1]])
    x,y=grid.cell(.1,.1)
    grid.integrate_surface_costs(point,np.array([35],dtype=np.int8),10.,passable=True)
    global_map,local_map=grid.navigation_maps_without_lidar_obstacles(10.,30.,2.)
    assert global_map[y,x]==local_map[y,x]==35
    assert local_map[y,x+1]==-1  # no invented free neighbours/space behind grass
    assert grid.navigation_maps_without_lidar_obstacles(12.01,30.,2.)[1][y,x]==-1
    assert grid.navigation_maps_without_lidar_obstacles(9.,30.,2.)[1][y,x]==-1
    # Fresh semantic grass must still obey the actual obstacle clearance radius.
    grid.integrate_surface_costs(np.array([[.5,.1]]),np.array([100],dtype=np.int8),10.)
    assert grid.navigation_maps_without_lidar_obstacles(10.,30.,2.)[1][y,x]==100


def test_ordinary_colour_cost_does_not_bypass_unknown_boundary():
    from nomad_perception.observed_grid import ObservedGrid
    grid=ObservedGrid(size=8,inflation=.6)
    point=np.array([[.1,.1]])
    x,y=grid.cell(.1,.1)
    grid.integrate_surface_costs(point,np.array([35],dtype=np.int8),10.)
    assert grid.navigation_maps_without_lidar_obstacles(10.,30.,2.)[1][y,x]==-1
