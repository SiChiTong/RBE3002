class GridCell:
    Gval = 0

    def __init__(self, x, y, occupancyLevel):
        self.Xpos = x
        self.Ypos = y
        self.occupancyLevel = occupancyLevel

        if 30 >= occupancyLevel:
            self.empty = True
        else:
            self.empty = False

    def setH(self, goalX, goalY):
        """
        sets the H value to the manhattan distance to the goal
        :param goalX: The goal X position on the grid.
        :param goalY: The goal Y position on the grid.
        """
        self.Hval = (abs(goalX - self.Xpos) + abs(goalY - self.Ypos)) * 10 + self.getOccupancyLevel()

    def setParent(self, parentCell):
        """
        sets the parent of the cell and also the G value because that is done right after
        and then the F value because its just H + G and we already have H
        :param parentCell: The parent cell to this cell.
        """
        self.parent = parentCell
        # if the x or y coordinate of the parent is the same as the child then we know the robot moves
        # like a rook, so we know to add 10
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
        """
        Determine if the cell is empty.
        :return: True if empty.
        """
        return self.empty

    def getXpos(self):
        """
        Get the X grid position of the cell.
        :return: The grid position.
        """
        return self.Xpos

    def getYpos(self):
        """
        Get the Y grid position of the cell.
        :return: The grid position.
        """
        return self.Ypos

    def getGval(self):
        return self.Gval

    def getHval(self):
        """
        Get the cell's H value.
        :return: The H value.
        """
        return self.Hval

    def getFval(self):
        """
        Get the cell's G value.
        :return: The G value.
        """
        return self.Fval

    def getParent(self):
        """
        Get the cell's parent.
        :return: A grid cell, the parent cell.
        """
        return self.parent

    def getOccupancyLevel(self):
        """
        Get the cell's occupancy level.
        :return: An int from 0 to 100.
        """
        return self.occupancyLevel

    def isUnknown(self):
        return self.occupancyLevel < 0

    def setOccupancyLevel(self, occupancyval):
        """
        Set the square occupancy level.
        :param occupancyval: The new occupancy level.
        """
        self.occupancyLevel = occupancyval
        if 50 >= self.occupancyLevel:
            self.empty = True
        else:
            self.empty = False

    def __str__(self):
        return str(self.getXpos()) + ' ' + str(self.getYpos()) + ' ' + str(self.isEmpty()) + ' ' + str(self.isUnknown())

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        return self.Xpos == other.Xpos and self.Ypos == other.Ypos
