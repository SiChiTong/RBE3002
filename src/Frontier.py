class Frontier:
    size = 0
    centroid = 0
    gridCells = []

    def __init__(self, cell_list, cell_matrix):
        self.gridCells = cell_list
        self.size = len(cell_list)
        self.centroid = self.get_centroid(cell_matrix)

    def get_centroid(self, cell_matrix):
        xsum = 0
        ysum = 0
        for cell in self.gridCells:
            xsum += cell.getXpos()
            ysum += cell.getYpos()
        newx = xsum // self.size
        newy = ysum // self.size
        explored = []
        unexplored = [cell_matrix[newx][newy]]
        while True:
            for cell in unexplored:
                if cell not in explored:
                    explored.append(cell)
                    if cell.isEmpty() and not cell.isUnknown():
                        return cell
                    else:
                        unexplored.append(cell_matrix[newx + 1][newy])
                        unexplored.append(cell_matrix[newx - 1][newy])
                        unexplored.append(cell_matrix[newx][newy + 1])
                        unexplored.append(cell_matrix[newx][newy - 1])
