import logging
from math import pi

from arena.map import Map
from arena.obstacle import Obstacle
from common.enums import Direction
from common.types import Position
from path_finding.astar import AStar
from path_finding.hamiltonian_path import ExhaustiveSearch
from robot.stm_commands import backtracking_smooth_path, merge_cmds
from simulator import Simulator
import settings


logger = logging.getLogger('BULLSEYE ROTATION')

origin = Position(80,50,pi/2)
x, y = 90, 100
obstacles = [
    Obstacle(x, y, Direction.NORTH),
    Obstacle(x, y, Direction.EAST),
    Obstacle(x, y, Direction.SOUTH),
    Obstacle(x, y, Direction.WEST)
]
mp = Map(obstacles)
astar = AStar(mp)

exh = ExhaustiveSearch(mp, origin, 8)

def rot_cmds():
    segs = exh.search()
    wps = []
    cmds = []
    for seg in segs:
        if not seg:
            continue
        wps.extend(seg)
        cmds.append(backtracking_smooth_path(seg))
        logger.info(cmds[-1])
    cmds_str = merge_cmds(cmds)
    logger.info(cmds_str)
    sim = Simulator(mp)
    sim.path = wps
    sim.run()

    return cmds_str


if __name__ == '__main__':
    rot_cmds()