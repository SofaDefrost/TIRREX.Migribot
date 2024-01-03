import csv
import codecs

articulationLength = 0.4
articulationWidth = 0.2
stepTime = 1.0e-3
stepDisplacement = 2.0e-3
convergenceWaiting = 0
tolerance = 30
totalDisplacement = 0
finalDisplacement = 1
mesh = 6
young = 1.4e3
poisson = 0.48
massLeg = 1.864e-6

actuatorCSV = "data/actuators.csv"
with open(actuatorCSV, "r", encoding='utf-8-sig') as file:
    trajectory = list(csv.reader(file, delimiter=';'))
nbPtTrajectory = len(trajectory)
ptTrajectory = 0

displacementPlatform1 = "data/displacementPlatform1.txt"
displacementPlatform2 = "data/displacementPlatform2.txt"
displacementFinger1 = "data/displacementFinger.txt"
displacementFinger2 = "data/displacementFinger.txt"
