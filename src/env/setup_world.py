#!/usr/bin/python
from setup_vehicle import SetupVehicle

class SetupWorld():
    def __init__(self):
        self.vehicle = SetupVehicle()
        self.isVehicle = False

    def reset(self, posistion, orientation):
        if self.isVehicle:
            self.vehicle.deleteVehicle()
        self.vehicle.spawnVehicle(posistion, orientation)
        self.isVehicle = True
