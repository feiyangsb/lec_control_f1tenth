import csv
import numpy as np

class DataCollector(object):
    def __init__(self, directory="/home/feiyang/Desktop/current_work/f1tenth/sims_ws/src/lec_control/data/rl_controller/"):
        self.directory = directory
        self.data_csv = []

    def storeDataStep(self, s, a, s_):
        data_line = []
        for i in s:
            i = i*10.0
            data_line.append(i)

        data_line.append(a[0][0])
        
        for i in s_:
            i = i*10.0
            data_line.append(i)
        self.data_csv.append(data_line)

    def writeDataEpisode(self, file_name='data'):
        csv_file = self.directory + file_name
        with open(csv_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerows(self.data_csv)
        self.data_csv = []