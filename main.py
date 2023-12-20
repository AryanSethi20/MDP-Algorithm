import logging
from math import pi
import time

from arena.map import Map
from arena.obstacle import Obstacle
from simulator import Simulator
from common.enums import Direction
from common.types import Position
from path_finding.hamiltonian_path import ExhaustiveSearch
from robot.stm_commands import backtracking_smooth_path
import settings


logger = logging.getLogger('MAIN')

origin = Position(0,0,pi/2)

# 2021 s2 run 1
_obstacles = [
    Obstacle(10, 180, Direction.SOUTH),
    Obstacle(60, 120, Direction.NORTH),
    Obstacle(140, 160, Direction.WEST),
    Obstacle(100, 60, Direction.EAST),
    Obstacle(130, 20, Direction.EAST),
    Obstacle(180, 90, Direction.WEST)
]

# 2021 s2 run 2
obstacles = [
    Obstacle(110, 40, Direction.WEST),
    Obstacle(60, 100, Direction.SOUTH),
    Obstacle(10, 160, Direction.SOUTH),
    Obstacle(130, 140, Direction.SOUTH),
    Obstacle(180, 30, Direction.NORTH),
    Obstacle(180, 160, Direction.SOUTH)
]

# 2022 s1 run 1
_obstacles = [
    Obstacle(20, 120, Direction.NORTH),
    Obstacle(100, 60, Direction.EAST),
    Obstacle(110, 180, Direction.SOUTH),
    Obstacle(190, 150, Direction.WEST),
    Obstacle(150, 90, Direction.NORTH),
    Obstacle(180, 20, Direction.WEST)
]

# 2023 s1 run 1
_obstacles = [
    Obstacle(10, 160, Direction.EAST),
    Obstacle(50, 120, Direction.SOUTH),
    Obstacle(80, 50, Direction.NORTH),
    Obstacle(110, 140, Direction.EAST),
    Obstacle(150, 20, Direction.WEST),
    Obstacle(160, 190, Direction.SOUTH),
    Obstacle(190, 90, Direction.WEST)
]

# testing
_obstacles = [
    Obstacle(50, 150, Direction.EAST),
    Obstacle(30, 80, Direction.SOUTH),
    Obstacle(120, 30, Direction.EAST),
    Obstacle(90, 110, Direction.WEST),
    Obstacle(150, 90, Direction.NORTH),
    Obstacle(70, 80, Direction.NORTH),
    Obstacle(180, 180, Direction.SOUTH),
    Obstacle(20, 80, Direction.NORTH)
]

_obstacles = [
    Obstacle(10, 70, Direction.SOUTH),
    Obstacle(10, 150, Direction.SOUTH),
    Obstacle(80, 40, Direction.WEST),
    Obstacle(100, 130, Direction.WEST),
    Obstacle(110, 130, Direction.SOUTH),
    Obstacle(120, 20, Direction.WEST),
    Obstacle(120, 40, Direction.NORTH),
    Obstacle(160, 30, Direction.NORTH)
]

_obstacles = [
    Obstacle(20, 120, Direction.NORTH),
    Obstacle(60, 100, Direction.EAST),
    Obstacle(150, 90, Direction.NORTH),
    Obstacle(180, 20, Direction.WEST),
    Obstacle(190, 150, Direction.WEST),
    Obstacle(110, 180, Direction.SOUTH)
]

_obstacles = [
    Obstacle(100, 60, Direction.NORTH),
    Obstacle(60, 120, Direction.SOUTH),
    Obstacle(110, 180, Direction.SOUTH),
    Obstacle(150, 90, Direction.SOUTH),
    Obstacle(170, 20, Direction.WEST),
    Obstacle(150, 160, Direction.WEST)
]

# Thursday Testing
_obstacles = [
    Obstacle(10, 180, Direction.SOUTH),
    Obstacle(50, 10, Direction.EAST),
    Obstacle(50, 90, Direction.SOUTH),
    Obstacle(90, 50, Direction.WEST),
    Obstacle(160, 10, Direction.WEST),
    Obstacle(180, 190, Direction.SOUTH),
    Obstacle(110, 140, Direction.NORTH),
    Obstacle(190, 90, Direction.WEST)
]

_obstacles = [
    Obstacle(20, 110, Direction.SOUTH),
    Obstacle(20, 120, Direction.NORTH),
    Obstacle(60, 100, Direction.EAST),
    Obstacle(80, 0, Direction.NORTH),
    Obstacle(150, 90, Direction.SOUTH),
    Obstacle(180, 20, Direction.NORTH),
    Obstacle(190, 150, Direction.WEST),
    Obstacle(110, 180, Direction.SOUTH)
]


mp = Map(obstacles)

processes = 9

if __name__ == '__main__':
    st = time.time()
    e = ExhaustiveSearch(mp, origin, processes)
    perm, segs = e.search()
    perm.pop(0)
    wps = []
    stm_cmds = ''
    f = 0

    for i, seg in enumerate(segs):
        if seg:
            wps.extend(seg)
            stm_cmds += f"{perm[i]}:{','.join(backtracking_smooth_path(seg))}{',BEGINNNN' if i > 0 else ''}-"
            f += seg[-1].f

    stm_cmds = stm_cmds.strip('-')
    logger.info(f'Finished in ({(time.time() - st):.2f}) s')
    logger.info(f'Total cost: ({f})')
    logger.info(f'STM commands: {stm_cmds}')

    sim = Simulator(mp)
    sim.path = wps
    sim.run()
