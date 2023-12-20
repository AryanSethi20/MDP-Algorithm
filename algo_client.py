import logging
from math import pi
import socket
import time
import threading
from typing import List

from arena.map import Map
from arena.obstacle import Obstacle
from common.consts import PENALTY_STOP
from common.utils import parse_map_str
from common.types import Position
from robot.stm_commands import backtracking_smooth_path
from path_finding.hamiltonian_path import ExhaustiveSearch
import settings


HOST = '192.168.1.1'  # The server's hostname or IP address
PORT = 5005 # The port used by the server
LISTEN_TIMEOUT = 60*3 # 3 mins

s = socket.socket()
s.connect((HOST, PORT))
done = False

logger = logging.getLogger('CLIENT')


def obs_to_cmds(ids: List[int], obs: List["Obstacle"]):
    mp = Map(obs)
    origin = Position(0,0,pi/2)

    st = time.time()
    e = ExhaustiveSearch(mp, origin, 8)
    perm, segs = e.search()
    perm.pop(0)
    wps = []
    stm_cmds = ''
    f = 0
    penalties = 0
    for i, seg in enumerate(segs):
        if seg:
            wps.extend(seg)
            stm_cmds += f"{ids[perm[i]-1]}:{','.join(backtracking_smooth_path(seg))}{',BEGINNNN' if i > 0 else ''}-"
            f += seg[-1].f
            penalties += len(seg) * PENALTY_STOP

    stm_cmds = stm_cmds.strip('-')
    logger.info(f'Finished in ({(time.time() - st):.2f}) s')
    logger.info(f'Total cost: ({f}), Penalties: {penalties}, Distance cost: {f-penalties}')
    logger.info(f'STM commands: {stm_cmds}')

    return stm_cmds


def receive():
    global done

    logger.info('Listening on socket ...')
    
    while True:
        text = s.recv(1024)

        if text:
            text = text.decode()
            logger.info(text)
            ids, obs = parse_map_str(text.split(';')[0])
            cmds = obs_to_cmds(ids, obs)
            done = True
            logger.info('Sending cmds to RPI')
            send(cmds)
            break


def send(text):
    logger.info(f'Sending to RPI: "{text}"')
    try:
        s.send(text.encode())

        logger.info('Successfully sent to RPI. Closing connection')
        s.close()
    except:
        logger.info('An exception occured when sending to RPI')


def start_timer():
    time.sleep(60*3)
    if not done:
        logger.info('Timed out. Unable to compute path in time')
        send('Timed out. Unable to compute path in time')


threading.Thread(target=receive).start()

# 1,1,7,2|2,0,15,2|3,8,4,4|4,10,13,4|5,11,13,2|6,12,12,4|7,12,4,1|8,16,3,1

'''

1:FW010.00-4:BW005.00,FR090.00,BW005.00,BEGINNNN-FW015.00,BL090.00,FW020.00,FL090.00,FW015.00,BEGINNNN-FW010.00,BL090.00,BW010.00,BL090.00,BR090.00,BEGINNNN-FW010.00,BL090.00,BW040.00,FL090.00,BEGINNNN-FW005.00,BL090.00,FW085.00,BL090.00,BEGINNNN-BL090.00,BL090.00,BR090.00,FW005.00,BEGINNNN-FW010.00,BR090.00,BEGINNNN

'''