import os

class dataLogger:
    def __init__(self):
        self.variables = 0
        self.variableDescriptions = []
        self.currentLog = {}
        self.loggedData = []
        self.initialized = False
        self.fileName = ""
        pass

    def addDataPoint(self, variableName):
        """Adds a data point to the logger object. Datapoints are added sequentially, so add your variables in the same sequence that you want them to show up in on the CSV"""
        if self.initialized == False:
            if str(variableName) in self.currentLog:
                raise IndexError("datapoiont already initialized")
            else:
                self.variables += 1
                self.variableDescriptions.append(variableName)
                self.currentLog[variableName] = None
        else:
            raise IndexError("file already initialized!")

    def recordVariable(self, variableName, data):
        """records a variable to the current log, DOES NOT LOG AUTOMATICALLY"""
        if str(variableName) in self.currentLog:
            # if self.currentLog[str(variableName)] != None:
            #     raise Warning(f'data point {str(variableName)} is being overwritten!')
            self.currentLog[str(variableName)] = data
        else:
            raise IndexError("datapoint not initialized")
    
    def initCSV(self, makeFile, overWrite):
        """Initializes the CSV file and prepares it for writing."""
        self.initialized = True

        os.chdir(os.path.dirname(os.path.abspath(__file__)))

        if os.path.exists(str(self.fileName)):

            f = open(str(self.fileName), "r")

            if not f.read():
                f.close()

                f = open(str(self.fileName), "w")
                outString = ""
                for varName in self.variableDescriptions:
                    outString += varName
                    outString += ","

                f.write(outString[0:-1])

                f.write('\n')
            else:
                if overWrite == True:
                    f.close()

                    f = open(str(self.fileName), "w")
                    outString = ""
                    for varName in self.variableDescriptions:
                        outString += varName
                        outString += ","

                    f.write(outString[0:-1])

                    f.write('\n')
                if overWrite == False:
                    raise OSError("csv file is not empty!")

        else:
            if makeFile == True:
                f = open(str(self.fileName), "w")
                
                f.close()
            else:
                raise OSError("csv file not found!")

    def saveData(self, clearData):
        outString = ""
        for datapoint in self.currentLog:
            currentVar = self.currentLog[str(datapoint)]
            if currentVar == None:
                outString += "0"
            else:
                outString += str(currentVar)
            outString += ","
            if clearData == True:
                self.currentLog[str(datapoint)] = None
        f = open(str(self.fileName), "a")
        f.write(outString[0:-1] + "\n")
        # f.write('\n')

    def getVariable(self, variableName):
        if str(variableName) in self.currentLog:
            return self.currentLog[str(variableName)]
        else:
            raise IndexError("datapoint not initialized")