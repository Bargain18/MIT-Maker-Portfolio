import traci
import os
from sumolib import checkBinary;

sumoBinary = checkBinary("sumo-gui")
sumoCmd = [sumoBinary, "-c", "quickfollow.sumocfg"]

traci.start(sumoCmd)

step = 0
while step < 1000:
    traci.simulationStep()
    if traci.inductionloop.getLastStepVehicleNumber("0") > 0:
        traci.trafficlight.setRedYellowGreenState("0", "GrGr")
    step += 1

traci.close()