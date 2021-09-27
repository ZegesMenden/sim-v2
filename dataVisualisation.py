import csv

class dataVisualiser:
    def __init__(self):
        self.allDataDescriptions = []

    def graph_from_csv(self, datapoints):
        descriptionNum = 0
        pointsToLog = []

        for description in self.allDataDescriptions:
            for requestedDatapoint in datapoints:
                if str(description) == str(requestedDatapoint):
                    pointsToLog.append(descriptionNum)
            descriptionNum += 1
        
        with open('data_out.csv', newline='\n') as pathFile:
            reader = csv.reader(pathFile, delimiter=',', quotechar='"')
            logList = []
            dataOut = []
            for index, row in enumerate(reader):    
                for point in pointsToLog:
                    logList.append(row[point])
                if index == 0:
                    for x in logList:
                        dataOut.append([])

                if index > 0:
                    for index, point in enumerate(dataOut):
                        point.append(float(row[pointsToLog[index]]))
                logList = []
        
        return dataOut


