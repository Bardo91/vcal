from numpy import genfromtxt
from pyquaternion import Quaternion
import IPython
import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys



gt = genfromtxt(sys.argv[1], delimiter=' ')
data = genfromtxt(sys.argv[2], delimiter=' ')


initRot = Quaternion( gt[0][7], gt[0][4], gt[0][5], gt[0][6]).rotation_matrix

gtPos = gt[:,1:4].transpose()


for i in range(gtPos.shape[1]):
	gtPos[:,i] = numpy.matmul(initRot,gtPos[:,i])
	

gtPos = gtPos.transpose()

gtPos = gtPos - gtPos[0,:]

fig =  plt.figure()
ax = plt.axes(projection='3d')


ax.plot(gtPos[:,0], gtPos[:,1], gtPos[:,2], 'blue', label='gt')
ax.plot(data[:,0], data[:,1], data[:,2], 'red', label='data')
ax.legend()

plt.show()


