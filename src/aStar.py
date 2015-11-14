#A* Plan
    # create a node class
    # First create a matix of nodes that corresponts to the map
    # node parameters are H value, G value, Parent, and F value
    # h value is assigned for every node when a goal is chosen
    # an open and closed list are created
    # add the node we are in to a closed list
    # make the node we are in the "selected node"
    # make a list of surrounding nodes not on closed or open list
    # add nodes surrounding "selected node" not on closed or open list to the open list
    # make each surounding node's parent the "selected node"
    # calculate the the g values for the surrounding node list (parent node's g + the cost to move (10 or 14))
    # calculate f values for surrounding nodes list
    # go to the lowest f value node not on the closed list, add it to the closed list
    # make the lowest f value node not on the closed list the "selected node"
    # repeat until the "selected node" is the goal node
    # when "selected node" is goal node, create a path by referencing the parents of nodes until the parent is the start node.


def aStar(mapInfo, start, goal):
	#TODO make the goal and start poses into an X and Y position on the grid
	#TODO make the map info into an array of data values, a width, and a height
	startX
	startY
	startTheta
	goalX
	goalY
	goalTheta
	mapArray
	width
	height

	mapMatrix = makeMap(mapArray, width, height, goalX, goalY) # now we have a map grid with H values

	openList = list() # make open and closed lists for a* search algorithm
	closedList = list()

	selectedCell = mapMatrix[startX][startY] # make the start position the selected cell

	while(selectedCell != mapMatrix[goalX][goalY]): # while we are searching for the goal
		closedList.expand(selectedCell) # add the selectedCell to the closed list
		surroundingCells = makeSurroundingList(selectedCell, openList, closedList) # creates a list of cells around the selected cell that are not in either list and are empty


	pass

def makeMap(map, width, height, goalX, goalY):
	count = 0
	matrixMap = [width][height]
	# iterating through every position in the matrix
	for y in range(0, height):
		for x in range (0, width):
			matrixMap[x][y] = GridCell(x, y, map[count]) # creates all the gridCells
			matrixMap[x][y].setH(goalX, goalY) # adds an H value to every gridCell
			count ++
	return matrixMap

def makeSurroundingList(mapMatrix, selectedCell, openList, closedList):
	# iterating through all the cells adjacent to the selected cell
	surroundingList = list()
	for x in range(selectedCell.getXpos() - 1, selectedCell.getXpos() + 1):
		for y in range(selectedCell.getYpos() - 1, selectedCell.getYpos() + 1):
			if (mapMatrix[x][y].isNotInList(openList) and mapMatrix[x][y].isNotInList(closedList) and mapMarix[x][y].isEmpty()):
				openList.expand(mapMatrix[x][y]) # this can be a copy or a pointer
				surroundingList.expand(mapMatrix[x][y]) # this should be a copy so that we can check 
	return surroundingList
