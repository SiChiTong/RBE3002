class Frontier:
    size = 0
    centroid
    gridCells = []

    def __init__(self, cellList, cellMatrix):
    	self.gridCells = cellList
    	size = len(cellList)
    	centroid = self.getCentroid(cellMatrix)

    def getCentroid(self, cellMatrix):
    	Xsum = 0
    	Ysum = 0
    	for cell in gridCells:
    		Xsum += cell.getXpos()
    		Ysum += cell.getYpos()
    	newX = Xsum // self.size
    	newY = Ysum // self.size
    	explored = []
    	unexplored = []
    	unexplored.append(cellMatrix[newX][newY])
    	while(1):
	    	for cell in unexplored and not in explored:
				explored.append(cell)
				if cell.isEmpty() and not cell.isUnknown():
					return cell
				else:
					unexplored.append(cellMatrix[newX + 1][newY])
					unexplored.append(cellMatrix[newX - 1][newY])
					unexplored.append(cellMatrix[newX][newY + 1])
					unexplored.append(cellMatrix[newX][newY - 1])


