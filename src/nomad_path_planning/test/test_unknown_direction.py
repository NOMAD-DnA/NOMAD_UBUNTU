import math
import numpy as np
import pytest
from nomad_path_planning.gpp.unknown_direction import unknown_direction_cost
from nomad_path_planning.gpp.base import Grid,Request


def grid():return dict(width=9,height=9,resolution=1.,origin_x=-4.5,origin_y=-4.5,data=[-1]*81)


def test_front_is_unpenalized_sides_equal_and_far_cells_zero():
    g=grid();c=unknown_direction_cost(g,(0.,0.,0.),12.,3.).reshape(9,9)
    assert c[4,6]==0 and c[6,4]==pytest.approx(4.)
    assert c[6,4]==c[2,4] and c[8,4]==0
    assert c[4,2]>0


def test_heading_rotation_and_known_cells():
    g=grid();c=unknown_direction_cost(g,(0.,0.,math.pi/2),12.,3.).reshape(9,9)
    assert c[6,4]==0 and c[4,6]>0
    g['data'][4*9+6]=5
    assert unknown_direction_cost(g,(0.,0.,math.pi/2),12.,3.)[4*9+6]==0


def test_disabled_and_hard_constraints_preserved():
    g=grid();g['data'][40]=100
    req=Request(g,(0.,0.,0.),(2.,0.));base=Grid(req).weights
    extra=unknown_direction_cost(g,req.start,24.)
    assert math.isinf(base[40]+extra[40])
    assert np.all(unknown_direction_cost(g,req.start,0.)==0)


@pytest.mark.parametrize('weight,radius,angle',[(-1,6,.5),(1,0,.5),(1,6,math.pi/2),(float('nan'),6,.5)])
def test_invalid_options_fail(weight,radius,angle):
    with pytest.raises(ValueError):unknown_direction_cost(grid(),(0.,0.,0.),weight,radius,angle)
