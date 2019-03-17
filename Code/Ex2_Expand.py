# With this exercise we will work on expand a node to its childs
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


    filename = os.path.join(os.path.dirname(__file__), "..", "CityInformation", city_string, "InfoVelocity.txt")
    infoVelocity = readInformation(filename)

    city = CityInfo(infoVelocity, stationList, timeStations)

    #------------------------------------------------------------------#

    origin=Node(stationList[1],None)            # Charpennes L1


    #------------------------------------------------------------------#
    # Code the function "Expand" in SearchAlgorithm.py

    print("\n________ Ex_2 ____________")
    childrenList=Expand(origin,city)
    print(" Current: [" + str(origin.station.id) + "] " + origin.station.name + " L" + str(origin.station.line)  )

    for i in childrenList:
        print("  ---  Child: [" + str(i.station.id) + "] " + i.station.name + " L" + str(i.station.line) )

    print("\n_________________________")
    origin2 = childrenList[2]
    childrenList2 = Expand(origin2, city)
    print(" Current: [" + str(origin2.station.id) + "] " + origin2.station.name + " L" + str(origin2.station.line))

    for i in childrenList2:
        print("  ---  Child: [" + str(i.station.id) + "] " + i.station.name + " L" + str(i.station.line))

if __name__ == '__main__':
    main()