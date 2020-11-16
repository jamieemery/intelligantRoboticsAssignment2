from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy
import copy
import numpy
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

from util import rotateQuaternion, getHeading
import random

from time import time

# import math functions
from math import *


"""Create a covariance matrix representing the Gaussian spread,
   measuring the uncertainty of the particles spread near the exact real location,
   they have to be initialised (at the PoseArray() initial location or the Robot's
   location or just random values and experiment which works best??), bear in mind that
   the diagonal terms(sigma_x2 and sigma_y2) represent the variance
   and the off-diagonal terms (sigma_xy and sigma yx) represent the correlation terms,
   which must be symmetrical and equivalent.
   The terms for variances can be set to different values, but that implies more
   uncertainty for the X or Y xis, depending on which value is lower.
"""

sigma_x2 = 1 #variance for x
sigma_xy = 0 #---correlation for xy
sigma_yx = 0 #---and yx
sigma_y2 = 1 #variance for y
actualSigma = numpy.array([[sigma_x2,sigma_xy],[sigma_yx,sigma_y2]])
sigma_inv = numpy.linalg.inv(actualSigma)

# Our 2-dimensional distribution will be over variables X and Y
N = 60
X = numpy.linspace(-3, 3, N)
Y = numpy.linspace(-3, 4, N)
X, Y = numpy.meshgrid(X, Y)

# Pack X and Y into a single 3-dimensional array
pos = numpy.empty(X.shape + (2,))
pos[:, :, 0] = X
pos[:, :, 1] = Y



class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()

        # ----- Set motion model parameters

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20  # Number of readings to predict



    # gaussian function for the 2D Kalman Filter
    #def gauss_f(mu, sigma, x):
        #dist_from_mean = (x-mu) #distance from the mean
        #dist_from_mean_transposed = numpy.transpose(dist_from_mean)
        ''' gauss_f takes in a mean and squared variance, and an input x
        and returns the gaussian value.'''
        #coefficient = 1.0 / (2.0 * math.pi *sqrt(abs(sigma)))
        #exponential = exp(-0.5 * dist_from_mean_transposed*sigma_inv*dist_from_mean)
        #return coefficient * exponential


    # the update function
    def change_mean_sigma(self, mean1, sigma1, mean2, sigma2):
        new_mean = (sigma2*mean1 + sigma1*mean2)/(sigma2+sigma1)
        new_sigma = 1/(1/sigma2 + 1/sigma1)

        return new_mean, new_sigma


    # the motion update/predict function
    #def predict(mean1, sigma1, mean2, sigma2):
        ''' This function takes in two means and two squared variance terms,
        and returns updated gaussian parameters, after motion.'''
        # Calculate the new parameters
        #new_mean = mean1 + mean2
        #new_sigma = sigma1 + sigma2

        #return [new_mean, new_sigma]

    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.

        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """

	    self.particlecloud = PoseArray()

        particle = Pose()

        mu_x = initialpose.pose.pose.position.x
        mu_y = initialpose.pose.pose.position.y
	particle.position.x = mu_x
        particle.position.y = mu_y
        particle.position.z = initialpose.pose.pose.position.z
	particle.orientation = initialpose.pose.pose.orientation
	self.particlecloud.poses.append(particle)
	print(self.particlecloud.poses)
	
        return self.particlecloud

    def create_C(self, predictedMut, predictedLaserScans):
        Ct = numpy.zeros((20, 2))
        #Ct = predictedLaserScans * numpy.linalg.inv(predictedMut)

        iterator = 0
        for j in predictedLaserScans:
            ctx = (j / predictedMut[0][0])/2
            cty = (j / predictedMut[1][0])/2
            Ct[iterator][0] = ctx
            Ct[iterator][1] = cty
            iterator += 1
        return Ct

    def createPredictedScan(self, predictedMut):
        predictedLaserScans = numpy.zeros((len(self.sensor_model.reading_points),1))
	iter = 0
        for i, obs_bearing in self.sensor_model.reading_points:
            # ----- Predict the scan according to the map
            map_range = self.sensor_model.calc_map_range(predictedMut[0][0], predictedMut[1][0],
                                     getHeading(self.particlecloud.poses[0].orientation) + obs_bearing)
	    # ----- Laser reports max range as zero, so set it to range_max
            if (map_range <= 0.0):
                map_range = self.sensor_model.scan_range_max

            predictedLaserScans[iter][0] = map_range
   	    iter += 1
        return predictedLaserScans

    def createActualScan(self, scan, scanMax):
        actualLaserScans = numpy.zeros((len(self.sensor_model.reading_points),1))
	iter = 0
        for i in self.sensor_model.reading_points:
            if math.isnan(scan[i[0]]):
                actualLaserScans[iter][0] = scanMax
   	    else:
                actualLaserScans[iter][0] = scan[i[0]]
   	    iter += 1
        return actualLaserScans

    def multivariate_gaussian(self, mu, Sigma):
        """Return the multivariate Gaussian distribution on array pos.

        pos is an array constructed by packing the meshed arrays of variables
        x_1, x_2, x_3, ..., x_k into its _last_ dimension.

        """

        n = mu.shape[0]
        Sigma_det = numpy.linalg.det(Sigma)
        Sigma_inv = numpy.linalg.inv(Sigma)
        N = numpy.sqrt((2*numpy.pi)**n * Sigma_det)
        # This einsum call calculates (x-mu)T.Sigma-1.(x-mu) in a vectorized
        # way across all the input variables.
        fac = numpy.einsum('...k,kl,...l->...', pos-mu, Sigma_inv, pos-mu)

        return numpy.exp(-fac / 2) / N

    def update_particle_cloud(self, scan):

	global actualSigma
	Rt = numpy.cov(numpy.random.rand(2,2))
	Qt = numpy.cov(numpy.random.rand(20,20))
	predictedMut = numpy.zeros((2,1))
        predictedMut[0][0] = self.particlecloud.poses[0].position.x
        predictedMut[1][0] = self.particlecloud.poses[0].position.y
        predictedScan = self.createPredictedScan(predictedMut)
        predictedSigma = actualSigma + Rt

        Ct = self.create_C(predictedMut,predictedScan)
	#print(Ct)
        zt = self.createActualScan(scan.ranges, scan.range_max)
        Kt = numpy.dot(numpy.dot(predictedSigma, numpy.transpose(Ct)), numpy.linalg.inv(numpy.dot(numpy.dot(Ct,predictedSigma),numpy.transpose(Ct))+Qt))
        I = numpy.identity(2)
	#print(Kt)
        actualMut = predictedMut + numpy.dot(Kt,(zt - numpy.dot(Ct,predictedMut)))
        actualSigma = numpy.dot((I-numpy.dot(Kt,Ct)),predictedSigma)

        self.particlecloud.poses[0].position.x = actualMut[0][0]
	self.particlecloud.poses[0].position.y = actualMut[1][0]

        print(actualMut-predictedMut)

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).

        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.

        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        # remove the outliers, keep the densest particles

        # compare all the possible distances between particles in our particlecloud

        distances = []
        i = 0
        for p1 in self.particlecloud.poses:
            i += 1
            for p2 in self.particlecloud.poses[i:]:
                distance = numpy.sqrt(((p1.position.x - p2.position.x)**2) \
                                      + ((p1.position.y - p2.position.y)**2) \
                                      + ((p1.position.z - p2.position.z)**2))
                distances.append(distance)

        # sort the distances and keep the first third of them
        min_dist = sorted(distances)[:int(round(len(distances) / 3))]  # testing !!    !!!!!!!!!!!!!!!
        # calculate each particle's number of appearances in the min_dist
        counter = numpy.zeros(len(self.particlecloud.poses))
        i = 0
        # increase the number of appearances depending on if the distance is included in the min_dist set
        for p1 in self.particlecloud.poses:
            i += 1
            j = i
            for p2 in self.particlecloud.poses[i:]:
                distance = numpy.sqrt(((p1.position.x - p2.position.x)**2) \
                                      + ((p1.position.y - p2.position.y)**2) \
                                      + ((p1.position.z - p2.position.z)**2))
                if distance in min_dist:
                    counter[i - 1] += 1
                    counter[j] += 1
                j += 1


        # sort counter and keep the particles corresponding to the last third
        sort_count = sorted(range(len(counter)), key=lambda k: counter[k])
        sort_count = sort_count[int(round(2 * len(sort_count) / 3)):]
        wanted_array=[]
        for i in sort_count:
            wanted_array.append(self.particlecloud.poses[i])
        est_pose = Pose()
        # find the mean position
        x_values = y_values = z_values = 0
        for p in wanted_array:
            x_values += p.position.x     # means -->  x_values = x_values + p.position.x
            y_values += p.position.y
            z_values += p.position.z


        meanX = x_values / len(wanted_array)
        meanY = y_values / len(wanted_array)
        meanZ = z_values / len(wanted_array)
        est_pose.position.x = meanX
        est_pose.position.y = meanY
        est_pose.position.z = meanZ

        # find the mean orientation
        x_values = y_values = z_values = w_values = 0
        for p in wanted_array:
            x_values += p.orientation.x
            y_values += p.orientation.y
            z_values += p.orientation.z
            w_values += p.orientation.w
        meanX = x_values / len(wanted_array)
        meanY = y_values / len(wanted_array)
        meanZ = z_values / len(wanted_array)
        meanW = w_values / len(wanted_array)
        est_pose.orientation.x = meanX
        est_pose.orientation.y = meanY
        est_pose.orientation.z = meanZ
        est_pose.orientation.w = meanW

        return est_pose
