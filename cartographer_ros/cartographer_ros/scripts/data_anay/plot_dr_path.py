import numpy as np

import sys
import matplotlib.pyplot as plt

def PlotPath(path):
    frame = np.loadtxt(path + "/dr_result.txt")
    fig = plt.figure('dr path 2d')
    p2, = plt.plot(frame[:, 0], frame[:, 1],  'b-')
    plt.grid()
    # plt.colorbar()
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('dr path')
    # plt.legend(['RTK', 'optimized', 'RTK status'])
    plt.legend(['dr'])
    plt.show()


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input vaild param !!!')
        exit(1)
    else:
        path = sys.argv[1]
        PlotPath(path)
        exit(1)
        exit(1)
