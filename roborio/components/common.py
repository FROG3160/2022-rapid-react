from collections import deque

class Buffer(deque):
    def __init__(self, size, validLength=1):
        self.validLength = validLength
        super().__init__(maxlen=size)

    def filterList(self):
        # our calculations can't accept None values
        return [x for x in self if not x is None]

    def lengthFiltered(self):
        return len(self.filterList())

    def isValidData(self):
        return self.lengthFiltered() >= self.validLength

    def average(self):
        if self.isValidData():
            filteredList = self.filterList()
            return sum(filteredList) / len(filteredList)
        else:
            return None

    def appendList(self, values):
        for value in values:
            self.append(value)
