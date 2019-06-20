#!/usr/bin/python
from setup_vehicle import SetupVehicle

class SetupWorld():
    def __init__(self):
        self.vehicle = SetupVehicle()

    def reset(self, posistion, orientation):
        self.vehicle.deleteVehicle()
        self.vehicle.spawnVehicle(posistion, orientation)
