import logging
from math import pi, atan
import numpy as np

from common.consts import (
    ROBOT_WIDTH,
    ROBOT_VERT_OFFSET,
    FL_A,
    FL_B,
    FR_A,
    FR_B,
    BL_A,
    BL_B,
    BR_A,
    BR_B
)
from common.enums import Movement
from common.utils import calc_vector
import settings


logger = logging.getLogger('WAYPOINTS')


def _discretise(cx, cy, mv, step=0.2, mn_step=3):
    fx = lambda x:(cy*(1-((x-cx)/cx)**2)**.5, cy*(1-((x-cx)/cx)**2)**.5)
    dydx = lambda x,y:(cy**2/cx**2*(cx-x)/y)
    sign = -1 if cx < 0 else 1
    step *= sign
    x = step
    wps = [(0,0,0)]
    wps_filt = [(0,0,0)]
    prev = 0, 0

    while x*sign < cx*sign:
        y1, y2 = fx(x)
        y = y1 if mv.value in (3,4) else y2
        m = dydx(x, y)
        
        phi = abs(atan(1/m)) if mv.value in (3,6) else -abs(atan(1/m))
        vv = calc_vector(pi/2 + phi, ROBOT_VERT_OFFSET) # UP 
        vh = calc_vector(phi, ROBOT_WIDTH/2) # RIGHT
        xx, yy = np.array([x+ROBOT_WIDTH/2, y+ROBOT_VERT_OFFSET]) - vh - vv
        wps.append((xx, yy, phi))

        if abs(prev[0] - xx) + abs(prev[1] - yy) > mn_step:
            wps_filt.append(wps[-1])
            prev = xx, yy
        x += step
    
    phi = pi/2 if mv.value in (3,6) else -pi/2
    vv = calc_vector(pi/2 + phi, ROBOT_VERT_OFFSET) # UP 
    vh = calc_vector(phi, ROBOT_WIDTH/2) # RIGHT
    y = fx(cx)[mv.value in (5,6)]
    xx, yy = np.array([x+ROBOT_WIDTH/2, y+ROBOT_VERT_OFFSET]) - vh - vv
    
    wps.append((xx, yy, phi))
    wps_filt.append(wps[-1])

    return wps, wps_filt
        

def gen_wps():
    _, wps_filt = _discretise(-FL_A, FL_B, Movement.FWD_LEFT, 0.2, .35)
    logger.info(f'FL {wps_filt} {len(wps_filt)}')

    _, wps_filt = _discretise(FR_A, FR_B, Movement.FWD_RIGHT, 0.4, 2.7)
    logger.info(f'FR {wps_filt} {len(wps_filt)}')

    _, wps_filt = _discretise(BR_A, -BR_B, Movement.BWD_RIGHT, 0.4, 3)
    logger.info(f'BR {wps_filt} {len(wps_filt)}')

    _, wps_filt = _discretise(-BL_A, -BL_B, Movement.BWD_LEFT, 0.2, .4)
    logger.info(f'BL {wps_filt} {len(wps_filt)}')
    

if __name__ == '__main__':
    # only run gen_wps() if this script is being run, not when this module is imported
    gen_wps()