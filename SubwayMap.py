# This file has all the functions required to load the information of a city.
# - Definition of the class Station
# - Definition of the class CityInfo
# - Read functions from files
# - Structure of the information
#
__authors__='TO_BE_FILLED'
__group__='DL01'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Grau en Enginyeria Informatica
# Curs 2016- 2017
# Universitat Autonoma de Barcelona
# _________________________________________________________________________________________


class Station:
    # __init__ Constructor of Station Class.
    def __init__(self, id, name, line, x, y):
        self.id = id                    # station id
        self.destinationDic = {}        # Dictionary where principal keys refers to the set of stations that it is connected.
                                        # The value of this dictionary refers to the time cost between two stations.
        self.name = name                # station Name
        self.line = int(line)           # line name string
        self.x = x                      # coordinate X of the station
        self.y = y                      # coordinate Y of the station

class CityInfo:
    # __init__ Constructor of CityInfo class
    def __init__(self, vel_lines, station_list, connection_time, multipleLines=0):
        self.num_lines=len(vel_lines)          # Number of different lines
        self.velocity_lines=vel_lines       # velocity of each line
        self.max_velocity=max(vel_lines)    # maximum velocity of the subways (faster subway)
        self.min_velocity=min(vel_lines)    # minimum velocity of the subways  (slower subway)
        self.max_transfer=0   # slower transfer time
        self.min_transfer=99999   # faster transfer time
        self.multipleLines=multipleLines
        self.StationList =station_list
        self.setNextStations(connection_time)

    # setNextStations: Given a stationList (- id, name, line, x, y - information), and the set of possible connections between stations,
    # This function set the dictionary of the possible destinations for each station (including the cost )
    def setNextStations( self, connections):
        for i in self.StationList:
            if int(i.id) in connections:
                i.destinationDic.update(connections[int(i.id)])


    def getTransfers(self):
        for i in self.StationList:
            for j in self.StationList[i].destinationDic:
                if i.line != j.line:
                    self.max_transfer=max(self.max_transfer,self.StationList[i].destinationDic[j])
                    self.min_transfer = min(self.min_transfer, self.StationList[i].destinationDic[j])
                    i.destinationDic.update(connections[int(i.id)])



def search_multiple_lines(stationList):
    """
    search_multiple_lines: Searches the set of stations that have different lines.
    :param
        - stationList: LIST of the stations of the current cicty (-id, destinationDic, name, line, x, y -)
    :return:
        - multiplelines: DICTIONARY which relates the different stations with the same name and different id's
                         (stations that have different metro lines)
    """

    multipleLines = {}
    for i in stationList:
        for j in stationList:
            if i.id != j.id:
                if i.x == j.x and i.y == j.y:
                    if multipleLines.has_key(i.id):
                        if j.id not in multipleLines[i.id]:
                            multipleLines[i.id].append(j.id)
                    else:
                        multipleLines[i.id] = []
                        multipleLines[i.id].append(j.id)
                    if multipleLines.has_key(j.id):
                        if j.id not in multipleLines[i.id]:
                            multipleLines[j.id].append(i.id)
                    else:
                        multipleLines[j.id] = []
                        multipleLines[j.id].append(i.id)
    return multipleLines
	
# readStationInformation: Given a filename, it reads the information of this file.
# The file should keep the format:
#	id <\t> name <\t> line <\t> x <\t> y <\n>
def readStationInformation(filename):
    fileMetro = open(filename, 'r')
    stationList = []
    for line in fileMetro:
        information = line.split('\t')
        station_read = Station(int(information[0]), information[1], information[2], int(information[3]),
                               int((information[4].replace('\n', '')).replace(' ', '')))
        stationList.append(station_read)
    fileMetro.close()
    return stationList

def readInformation(filename):
    vector=[]
    fp = open(filename,'r')
    line = fp.readline()
    while line:
       # tmp=fp.readline()
       try:
            value=line.split(" : ")
            value=value[1].split("\n")
            vector.append(int(value[0]))
            line = fp.readline()
       except :
            line = fp.readline()
    del vector[-1] #remove min value
    del vector[-1] #remove max value
    fp.close()
    return (vector)


# readCostTable: Given a filename, it reads the information of this file.
# The file should be an inferior matrix with the cost between two different stations.
def readCostTable(filename):
    fileCorrespondencia = open(filename, 'r')
    connections = {}
    origin = 1
    for i in fileCorrespondencia:
        informations = i.split('\t')
        destination = 1 # because ID of the stations started at '1' instead of '0'
        for j in informations:
            j = j.replace('\n', '')
            if j != '':
                if j != '0':
                    if int(origin) not in connections:
                        connections[int(origin)] = {}
                    if int(destination) not in connections[int(origin)]:
                        connections[int(origin)][int(destination)] = float(j)
                    # as the matrix is an inferior matrix, we should duplicate the information to the superior missing part.
                    if int(destination) not in connections:
                        connections[int(destination)] = {}
                    if int(origin) not in connections[int(destination)]:
                        connections[int(destination)][int(origin)] = float(j)

            destination = destination + 1
        origin = origin + 1
    return connections




# print_stationList: Given a stationList (- id, name, line, x, y - information), it prints the information by terminal
def print_stationList(stationList):
    print("\n")
    print (" ______________ STATION LIST________________")
    print ("\n")
    for i in stationList:
        print (" ID : " + str(i.id) + " - " + str(i.name) + " linea: " + str(i.line) + "   pos: (" + str(i.x) + "," + str(i.y) + ")")
    print ("\n")
    print ("\n")


# print_connections: Given a connections dictionary, it prints the information by terminal
def print_connections(connections):
    print ("\n")
    print (" ______________ CONNECTIONS ________________")
    print ("\n")
    for i in connections.keys():
        print (" ID : " + str(i) + "  ")
        for j in connections[i].keys():
            print ("           " + str(j) + " : " + str(connections[i][j]))
    #print ("\n")
    #print ("\n")

def print_dictionary(stationList):
    print ("\n")
    print (" ______________ DICTIONARY ________________")
    print ("\n")
    for i in stationList:
            print (" ID : "+  str(i.id) + "  -->   " + str(i.destinationDic))
    print ("\n")
    print ("\n")
