from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy
import copy
import numpy

from util import rotateQuaternion, getHeading
import random

from time import time


class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()

        # ----- Set motion model parameters

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20  # Number of readings to predict

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
        # ----- Initialize the particle cloud as an empty array
        self.particlecloud = PoseArray()

        """Create the noise to multiply by the random Gaussian number that will
        get added to each of the Poses, that are set to a random position
        and orientation around the initial pose"""
        sensorSigma=3 #variance
        sensorMu=0 #mean
        noise=sensorSigma * numpy.random.randn() + sensorMu

        """Create a range for the ammount of random Gaussian values to generate """
        randomGauss = 10*self.NUMBER_PREDICTED_READINGS

        gaussianRandomNumX = []
        gaussianRandomNumY = []
        randomYawArray = []

        for i in range (0,randomGauss):
            gaussianRandomNumX.append(random.gauss(0,1))
            gaussianRandomNumY.append(random.gauss(0,1))
            x=random.randint(1,180)
            randomYaw=(math.pi/x)
            randomYawArray.append(randomYaw)

        iterator = 0

        """
	    Set the particles to a random position and orientation around the initial pose
        """
        particleNumber = 10**2 # 10**3 # 10**4 # 10**5 experiment with different ammounts of particles

        while iterator < particleNumber:
            particle = Pose()
            particle.position.x = initialpose.pose.pose.position.x + (gaussianRandomNumX[iterator] * noise)
            particle.position.y = initialpose.pose.pose.position.y + (gaussianRandomNumY[iterator] * noise)
            particle.position.z = initialpose.pose.pose.position.z
            particle.orientation = rotateQuaternion(initialpose.pose.pose.orientation, randomYawArray[iterator])

            self.particlecloud.poses.append(particle)
            iterator += 1

        return self.particlecloud

    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.

        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """

        """
        Initialise arrays for the new particle cloud,
        particle weights and cummulative weights
        """
        predictedMut =  numpy.zeros((1,2))
        predictedMut[0] = self.particlecloud.poses[0].position.x
        predictedMut[1] = self.particlecloud.poses[0].position.y
        predictedLaserScans = numpy.zeros(1, len(self.sensor_model.reading_points))
        for i, obs_bearing in self.sensor_model.reading_points:

            # ----- Predict the scan according to the map
            map_range = self.sensor_model.calc_map_range(predictedMut[0], predictedMut[1],
                                     getHeading(self.particlecloud.poses[0].orientation) + obs_bearing)
            predictedLaserScans[i] = map_range

        # Ct = numpy.zeros((2, 500))
        Ct = predictedLaserScans * numpy.linalg.inv(predictedMut)
    #    iterator = 0
        #for j in predictedLaserScans:
        #    ctx = j / predictedMut[0]
        #    cty = j / predictedMut[1]
        #    Ct[0][iterator] = ctx
        #    Ct[1][iterator] = cty
        #    iterator += 1
        pass

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
