from pyamaze import maze,agent,COLOR,textLabel
from queue import PriorityQueue
def h(cell1, cell2):
    x1, y1 = cell1
    x2, y2 = cell2
    return (abs(x1 - x2) + abs(y1 - y2))
    
def aStar(m,start=None):
    if start is None:
        start=(m.rows,m.cols)
    open = PriorityQueue()
    open.put((h(start, m._goal), h(start, m._goal), start))
    aPath = {}
    g_score = {row: float("inf") for row in m.grid}
    g_score[start] = 0
    f_score = {row: float("inf") for row in m.grid}
    f_score[start] = h(start, m._goal)
    searchPath=[start]
    while not open.empty():
        currCell = open.get()[2]
        searchPath.append(currCell)
        if currCell == m._goal:
            break        
        for d in 'ESNW':
            if m.maze_map[currCell][d]==True:
                if d=='E':
                    childCell=(currCell[0],currCell[1]+1)
                elif d=='W':
                    childCell=(currCell[0],currCell[1]-1)
                elif d=='N':
                    childCell=(currCell[0]-1,currCell[1])
                elif d=='S':
                    childCell=(currCell[0]+1,currCell[1])

                temp_g_score = g_score[currCell] + 1
                temp_f_score = temp_g_score + h(childCell, m._goal)

                if temp_f_score < f_score[childCell]:   
                    aPath[childCell] = currCell
                    g_score[childCell] = temp_g_score
                    f_score[childCell] = temp_g_score + h(childCell, m._goal)
                    open.put((f_score[childCell], h(childCell, m._goal), childCell))


    fwdPath={}
    camino = []
    cell=m._goal
    while cell!=start:
        temporal = aPath[cell]
        fwdPath[temporal]=cell
        if (temporal[0], temporal[1]+1) == cell:
            camino.append('E')
        elif (temporal[0], temporal[1]-1) == cell:
            camino.append('W')
        elif (temporal[0]-1, temporal[1]) == cell:
            camino.append('N')
        elif (temporal[0]+1, temporal[1]) == cell:
            camino.append('S')
        cell=aPath[cell]
    
    camino.reverse()
    return searchPath,aPath,fwdPath,camino

def direccionRobot(caminoMapa):
    robotApunta= 'N'
    caminoRobot= []
    for i in range(len(caminoMapa)):
        if robotApunta == 'N' and caminoMapa[i] =='E':
            caminoRobot.append('d')
            robotApunta= 'E'
        if robotApunta == 'N' and caminoMapa[i] =='W':
            caminoRobot.append('i')
            robotApunta= 'W'
    
        if robotApunta == 'E' and caminoMapa[i] =='N':
            caminoRobot.append('i')
            robotApunta= 'N'
        if robotApunta == 'E' and caminoMapa[i] =='S':
            caminoRobot.append('d')
            robotApunta= 'S'

        if robotApunta == 'W' and caminoMapa[i] =='N':
            caminoRobot.append('d')
            robotApunta= 'N'
        if robotApunta == 'W' and caminoMapa[i] =='S':
            caminoRobot.append('i')
            robotApunta= 'S'

        if robotApunta == 'S' and caminoMapa[i] =='W':
            caminoRobot.append('d')
            robotApunta= 'W'
        if robotApunta == 'S' and caminoMapa[i] =='E':
            caminoRobot.append('i')
            robotApunta= 'E'

        caminoRobot.append('a')
        
    return caminoRobot
            
    

if __name__=='__main__':

    s=[5,8]
    g=[1,4]
    m=maze(5,8)
    m.CreateMaze(g[0],g[1],loadMaze='aStardemo.csv')
    searchPath,aPath,fwdPath,camino=aStar(m,(s[0],s[1]))
    #a=agent(m,footprints=True,color=COLOR.blue,filled=True)
    #b=agent(m,5,5,footprints=True,color=COLOR.yellow,filled=True,goal=(1,4))
    c=agent(m,s[0],s[1],footprints=True,color=COLOR.red,goal=(g[0],g[1]))

    #m.tracePath({a:searchPath},delay=300)
    #m.tracePath({b:aPath},delay=300)
    m.tracePath({c:fwdPath},delay=300)

    l=textLabel(m,'A Star Path Length',len(fwdPath)+1)
    l=textLabel(m,'A Star Search Length',len(searchPath))
    m.run()
    print(fwdPath)
    print(camino)

    arregloCamino= direccionRobot(camino)
    print(arregloCamino)