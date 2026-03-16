#!/usr/bin/env python3
"""Pathfinding on grids: A*, Dijkstra, BFS."""
import sys,heapq

def neighbors(pos,grid):
    r,c=pos;rows,cols=len(grid),len(grid[0]);result=[]
    for dr,dc in[(-1,0),(1,0),(0,-1),(0,1)]:
        nr,nc=r+dr,c+dc
        if 0<=nr<rows and 0<=nc<cols and grid[nr][nc]!=1:result.append((nr,nc))
    return result

def astar(grid,start,goal):
    def h(a):return abs(a[0]-goal[0])+abs(a[1]-goal[1])
    open_set=[(h(start),0,start)];came_from={};g_score={start:0}
    while open_set:
        _,cost,current=heapq.heappop(open_set)
        if current==goal:
            path=[];n=goal
            while n in came_from:path.append(n);n=came_from[n]
            path.append(start);return list(reversed(path))
        for nb in neighbors(current,grid):
            ng=g_score[current]+1
            if ng<g_score.get(nb,float('inf')):
                g_score[nb]=ng;came_from[nb]=current
                heapq.heappush(open_set,(ng+h(nb),ng,nb))
    return None

def bfs(grid,start,goal):
    queue=[start];visited={start};came_from={}
    while queue:
        current=queue.pop(0)
        if current==goal:
            path=[];n=goal
            while n in came_from:path.append(n);n=came_from[n]
            path.append(start);return list(reversed(path))
        for nb in neighbors(current,grid):
            if nb not in visited:visited.add(nb);came_from[nb]=current;queue.append(nb)
    return None

def main():
    if len(sys.argv)>1 and sys.argv[1]=="--test":
        grid=[[0,0,0,0,0],[0,1,1,1,0],[0,0,0,1,0],[0,1,0,0,0],[0,0,0,0,0]]
        path=astar(grid,(0,0),(4,4))
        assert path is not None and path[0]==(0,0) and path[-1]==(4,4)
        assert all(grid[r][c]==0 for r,c in path)
        path_bfs=bfs(grid,(0,0),(4,4))
        assert path_bfs is not None and len(path_bfs)==len(path)  # both optimal on unweighted
        # No path
        blocked=[[0,1],[1,0]]
        assert astar(blocked,(0,0),(1,1)) is None
        print("All tests passed!")
    else:
        grid=[[0]*10 for _ in range(10)];grid[3]=[0,0,0,1,1,1,1,1,0,0]
        path=astar(grid,(0,0),(9,9))
        print(f"Path length: {len(path)}")
if __name__=="__main__":main()
