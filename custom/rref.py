from decimal import Decimal, getcontext

import math

import constants


class Matrix:
    """
    Well I was hoping it wouldn't come to this
    but it seems as though the internet is full
    of idiots who don't know how to make a functional
    RREF calculator. Hopefully I'm different. I
    am going to make this super simple and really
    low level. Just to get the job done.

    Ok, so don't use this lol.
    """

    def __init__(self, matrix):
        self.m = []
        self.pivots = len(matrix[0]) - 1  # Because of augmentation.
        self.rows = len(matrix)
        for row in matrix:
            self.m.append(Row(row))

        getcontext().prec = (
            constants.drivetrain.decimalPlaces
        )  # This is the precision of the decimal numbers.

    def rref(self):
        """
        Make sure to call this method! Not solve()!
        """
        self.solve()
        return self.getResults()

    def solve(self):
        self.rearrange()

        if self.rows < self.pivots:
            self.pivots = self.rows

        for pivotSelector in range(self.pivots):
            selectedRow = self.m[pivotSelector]
            a = selectedRow.getValue(pivotSelector)
            for i in range(pivotSelector + 1, self.rows):  # This line might break it.
                rowBelow = self.m[i]
                b = rowBelow.getValue(pivotSelector)
                try:
                    conversion = (Decimal(b) / Decimal(a)) * Decimal(-1)
                    invConversion = Decimal(1) / conversion
                except ZeroDivisionError:
                    continue
                selectedRow.scale(conversion)
                result = rowBelow.combine(selectedRow)
                self.m[i] = result
                selectedRow.scale(invConversion)

        for pivotSelector in range(self.pivots):
            selectedRow = self.m[pivotSelector]
            a = selectedRow.getValue(pivotSelector)
            for i in range(pivotSelector):
                rowAbove = self.m[i]
                b = rowAbove.getValue(pivotSelector)
                try:
                    conversion = (Decimal(b) / Decimal(a)) * Decimal(-1)
                    invConversion = Decimal(1) / conversion
                except (ZeroDivisionError):
                    continue
                selectedRow.scale(conversion)
                result = rowAbove.combine(selectedRow)
                self.m[i] = result
                selectedRow.scale(invConversion)

        for i in range(len(self.m)):
            if not self.m[i].isEmpty():
                selectedRow = self.m[i]
                pivot = selectedRow.getValue(i)
                try:
                    selectedRow.scale(1 / pivot)
                except (ZeroDivisionError):
                    continue

    def display(self):
        print("------")
        for row in self.m:
            print(row.getRow())
        print("------")

    def getResults(self):
        return [row.getRow()[-1] for row in self.m]

    def rearrange(self):
        self.m.sort(key=lambda x: x.numOfZeros(), reverse=False)


class Row:
    """
    Makes doing elementary row operations easier.
    """

    def __init__(self, r):
        self.columns = r

    def combine(self, other):
        return Row(
            [Decimal(i) + Decimal(j) for i, j in zip(self.columns, other.columns)]
        )

    def scale(self, scalar):
        self.columns = [(Decimal(i) * Decimal(scalar)) for i in self.columns]

    def getValue(self, index):
        return self.columns[index]

    def getRow(self):
        return self.columns

    def isEmpty(self):
        return self.numOfZeros() == len(self.columns)

    def numOfZeros(self):
        return self.columns.count(0)
