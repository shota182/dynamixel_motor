import time
import numpy as np
from dynamixel_sdk import *  # Uses Dynamixel SDK library
from xl330 import XL330

def main():
    Xl330 = XL330(range(1,10))
    Xl330.setcurrent(np.full(9, 40), 2)
    # Xl330.setcurrent(np.tile([-25, -50, 50], 3), 2)

if __name__ == "__main__":
    main()