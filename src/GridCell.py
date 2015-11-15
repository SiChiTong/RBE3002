class GridCell:
    # Hval # manhattan distance to goal
    # Gval # cost for moving to the cell from the parent cell (parent G val + 10 for rook or 14 for bishop)
    # Fval # H + G value
    # Xpos
    # Ypos
    # empty
    # parent
    Gval = 0

    def __init__(self, x, y, occupancyLevel):
        self.Xpos = x
        self.Ypos = y
        self.occupancyLevel = occupancyLevel

        if (occupancyLevel != 0):
            self.empty = False
        else:
            self.empty = True

    # sets the H value to the manhattan distance to the goal
    def setH(self, goalX, goalY):
       self.Hval = abs(goalX - self.Xpos) + abs(goalY - self.Ypos)

    # sets the parent of the cell and also the G value because that is done right after
    # and then the F value because its just H + G and we already have H
    def setParent(self, parentCell):
        self.parent = parentCell
        # if the x or y coordinate of the parent is the same as the child then we know the robot moves like a rook, so we know to add 10
        # otherwize it moves like a bshop so we add 14 because 10 * sqrt(2) = ~14
        # geometry
        if (self.Xpos == parentCell.getXpos() or self.Ypos == parentCell.getYpos()):
            self.Gval = parentCell.getGval() + 10
        else:
            self.Gval = parentCell.getGval() + 14
        self.Fval = self.Hval + self.Gval


    def isNotInList(self, theList):
        for cell in theList:
            if cell.getXpos() == self.Xpos and cell.getYpos() == self.Ypos:
                return False
        return True


    def isEmpty(self):
        return self.empty


    def getXpos(self):
        return self.Xpos


    def getYpos(self):
        return self.Ypos


    def getGval(self):
        return self.Gval


    def getHval(self):
        return self.Hval


    def getFval(self):
        return self.Fval


    def getParent(self):
        return self.parent


    def getOccupancyLevel(self):
        return self.occupancyLevel


    def __str__(self):
        return str(self.getXpos()) + ' ' + str(self.getYpos()) + ' ' + str(self.isEmpty())


    def __repr__(self):
        return self.__str__()


    def __eq__(self, other):
        return self.Xpos == other.Xpos and self.Ypos == other.Ypos
