#!/usr/bin/python3

from data_generator import DataGenerator
import os
if __name__ == "__main__":
    data_generator = DataGenerator(os.path.join(os.path.dirname(__file__),'calibration_results.json'),
                                   os.path.join(os.path.dirname(__file__),'tact_calibration.json')
                                   )