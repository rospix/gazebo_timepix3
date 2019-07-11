#!/usr/bin/env python

import sys
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

def photoabsorption_coeff(material, energy):
    ## This script interpolates the mass attenuation coefficient from NIST tables
    # Input params: material (string, lowercase), energy (float, MeV)
    # Return params: photoabsorption coefficient (float)

    print 'Material:' , material, ', Photon energy:', energy

    ## TODO make this more robust by searching for the gazebo_timepix package
    input_path='/home/stibipet/mrs_workspace/src/radiation_nodes/gazebo_timepix/nist/' + material + '.csv'
    try:
        data = np.genfromtxt(input_path, delimiter=',')
    except:
        print 'Material properties not defined'
        return 0

    x = data[:,0]
    mac = data[:,1] # mass attenuation coefficient

    mac_interp = interp1d(x, mac, kind='quadratic')


    print material, 'photoabsorption coefficient for', energy, 'MeV photons:', mac_interp(energy)

    # # #{ Plotting
    # x_dense = np.linspace(min(x), max(x), num=x.shape[0]*100, endpoint=True)
    # mac_dense = mac_interp(x_dense)
    # fig1 = plt.figure('Air')
    # plt.plot(x_dense, mac_dense)
    # plt.scatter(float(energy), mac_interp(energy))
    # plt.xscale('log')
    # plt.yscale('log')
    # plt.legend(['Original', 'Interpolation'])
    # plt.show()
    # # #}

    return mac_interp(energy)
