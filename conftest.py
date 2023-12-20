from math import pi
import pytest

from arena.map import Map
from arena.obstacle import Obstacle
from common.enums import Direction
from simulator import Simulator


######################
##  test_movements  ##
######################

@pytest.fixture
def src():
    return 50, 50, pi/2


@pytest.fixture
def mp():
    return Map([
        Obstacle(50, 150, Direction.EAST),
        Obstacle(30, 80, Direction.SOUTH),
        Obstacle(120, 30, Direction.EAST),
        Obstacle(90, 110, Direction.WEST),
        Obstacle(150, 90, Direction.NORTH)
    ])


@pytest.fixture
def collision_sim():
    mp = Map([Obstacle(180, 160, Direction.SOUTH)])
    sim = Simulator(mp)
    return sim

@pytest.fixture
def collision_mp():
    return Map([Obstacle(180, 160, Direction.SOUTH)]) 

@pytest.fixture
def collision_sim_2():
    mp = Map([
        Obstacle(50, 150, Direction.NORTH), 
        Obstacle(90, 110, Direction.NORTH), 
        Obstacle(70, 80, Direction.NORTH)
    ])
    sim = Simulator(mp)
    return sim

@pytest.fixture
def collision_mp_2():
    return Map([
        Obstacle(50, 150, Direction.NORTH), 
        Obstacle(90, 110, Direction.NORTH), 
        Obstacle(70, 80, Direction.NORTH)
    ]) 