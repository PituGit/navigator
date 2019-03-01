# With this exercise we will work on removing cycles from a path
#
__authors__='TO_BE_FILLED'
__group__='DL01'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Grau en Enginyeria Informatica
# Curs 2018- 2019
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________
from SearchAlgorithm import *
from SubwayMap import *
import os

def main():




    #------------------------------------------------------------------#
    city_string="Lyon_smallCity"

    filename = os.path.join(os.path.dirname(__file__), "..", "CityInformation", city_string, "Stations.txt")
    stationList = readStationInformation(filename)


    filename = os.path.join(os.path.dirname(__file__), "..", "CityInformation", city_string, "Time.txt")
    timeStations = readCostTable(filename)


    # Now we want to analyze extra information from the files.
    # Velocity of the lines
    filename = os.path.join(os.path.dirname(__file__), "..", "CityInformation", city_string, "InfoVelocity.txt")
    infoVelocity = readInformation(filename)


    # we save the global information of the city in 'city' variable.
    # Analyze which information contains an element of the class CityInfo.
    city = CityInfo(infoVelocity, stationList, timeStations)

    #------------------------------------------------------------------#

    origin=Node(stationList[1],None)            # Charpennes L1


    #------------------------------------------------------------------#

    # Code the function RemoveCycles in SearchAlgorithm.py







    print("\n________ Ex_3____________")

    childrenList = Expand(origin, city)
    childrenList=RemoveCycles(childrenList)
    print(" Current: [" + str(origin.station.id) + "] " + origin.station.name + " L" + str(origin.station.line))
    for i in childrenList:
        print("  ---  Child: [" + str(i.station.id) + "] " + i.station.name + " L" + str(i.station.line))
    print("\n___________________________")

    print("\n")
    origin2=childrenList[2] #Charpennes L2 (to be connected to the previous route)
    childrenList = Expand(origin2, city)
    childrenList=RemoveCycles(childrenList)
    print("\n")
    print(" Current: [" + str(origin2.station.id) + "] " + origin2.station.name + " L" + str(origin2.station.line))
    for i in childrenList:
        print("  ---  Child: [" + str(i.station.id) + "] " + i.station.name + " L" + str(i.station.line) )



if __name__ == '__main__':
    main()