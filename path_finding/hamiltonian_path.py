import heapq
import logging
import multiprocessing as mp
import time
from typing import List

from arena.map import Map
from common.types import Position
from common.utils import euclidean
from path_finding.astar import AStar, Node


logger = logging.getLogger('HAMILTONIAN PATH')


def knn(mp: "Map", src: "Position") -> List[List["Node"]]:
    astar = AStar(mp)
    path = [src] + [o.to_pos() for o in mp.obstacles]

    for i in range(1, len(path)):
        mn = float('inf')
        mn_i = i+1

        for j in range(i, len(path)):
            dist = euclidean(path[i-1], path[j])
            if dist < mn:
                mn = dist
                mn_i = j

        path[mn_i], path[i] = path[i], path[mn_i]

    res = []
    prev = src
    for i in range(1, len(path)):
        res.append(astar.search(prev, path[i]))
        prev = res[-1][-1].c_pos
    return res


def _permutate(n: int, start_from_zero: bool) -> List[List[int]]:
    res = []

    def helper(curr: List[int]):
        if len(curr) == n:
            res.append(curr)
            return
        for i in range(n):
            if i not in curr:
                helper([*curr, i])
    helper([])
    if start_from_zero:
        res = list(filter(lambda p:p[0] == 0, res))
    return res


class SearchProcess(mp.Process):
    def __init__(
        self,
        pos: List["Position"],
        astar: AStar,
        todo: mp.Queue,
        done: mp.Queue,
        i:int
    ):
        super().__init__()
        self.astar = astar
        self.pos = pos
        self.todo = todo
        self.done = done
        self.i = i
        logger.info(f'Spawning P{i}')


    def _search(
        self,
        st: int,
        end: int
    ) -> float:
        logger.info(f'P{self.i} start search {st, end}')
        path = self.astar.search(self.pos[st], self.pos[end])
        return path[-1].f if path else 99999
        
    
    def run(self):
        while 1:
            try:
                st, end = self.todo.get()
                self.done.put((st, end, self._search(st, end)))
            except:
                logger.info(f'P{self.i} finished')
                return


class ExhaustiveSearch:

    def __init__(
        self,
        map: "Map", 
        src: "Position",
        n: int = 8
    ):
        self.astar = AStar(map)
        self.src = src
        self.pos = [src] + [o.to_pos() for o in map.obstacles]
        self.n = n

    
    def search(self, top_n: int = 3):
        st = time.time()
        n = len(self.pos)
        m = int(n*n - n) # total paths to calc from pt to pt
        perms = _permutate(n, True)
        edges = [[0 for _ in range(n)] for _ in range(n)]
        todo = mp.Queue()
        done = mp.Queue()

        for r in range(n):
            for c in range(n):
                if r != c:
                    todo.put((r, c)) 

        for i in range(self.n):
            p = SearchProcess(self.pos, self.astar, todo, done, i)
            p.daemon = True
            p.start()

        while m:
            r, c, f = done.get()
            edges[r][c] = f
            logger.info(f'{r} -> {c} ({f})')
            m -= 1
        logger.info(f'Adj list completed in {time.time()-st} s')

        # get shortest path, i.e., lowest cost among all permutations
        h = []
        for i, perm in enumerate(perms):
            cost = sum([edges[perm[i]][perm[i+1]] for i in range(n-1)])
            heapq.heappush(h, (cost, perm))

        loc_mn_path = []
        loc_mn_f = float('inf')
        min_perm = []
        for _ in range(min(top_n, len(h))):

            path = []
            prev = self.pos[0]
            cost, perm = heapq.heappop(h)
            f = 0
            logger.info(f'Calculating path for {perm}')

            for i in range(1, n):
                segment = self.astar.search(prev, self.pos[perm[i]])

                if segment:
                    path.append(segment)
                    prev = segment[-1].c_pos
                    f += segment[-1].f
                else:
                    f += 99999

                if f > loc_mn_f:
                    break

            if f < loc_mn_f:
                loc_mn_f = f
                loc_mn_path = path
                min_perm = perm
            if f < 99999:
                return perm, path
        
        return min_perm, loc_mn_path