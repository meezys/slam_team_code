"""
GTSAM Copyright 2010-2018, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Simple robotics example using odometry measurements and bearing-range (laser) measurements
Author: Alex Cunningham (C++), Kevin Deng & Frank Dellaert (Python)
"""
# pylint: disable=invalid-name, E1101

from __future__ import print_function
from scipy.spatial import KDTree
import gtsam
import numpy as np
# from typing import List, Tuple
# import random
# from copy import deepcopy
# #from .particle import Particle
from cone_type import ConeType
from landmark import Landmark
import math
from gtsam.symbol_shorthand import L, X

# Create noise models
PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.4, 0.1]))
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.0425, 0.0425, 0.0425]))
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.2]))

class GraphSLAM:
    """Run the GraphSLAM Algorithm."""
    
    def __init__(
        self, start_state:np.ndarray, association_threshold,landmarks) -> None:
        self.estimates = gtsam.Values() 
        self.association_threshold = association_threshold
        self.estimated_state = start_state
        self.index = 0
        self.poses = []
        self.landmarks = [Landmark]
        self.lm_tree = KDTree([ (0,0) ])
        self.isam = gtsam.ISAM2()
        #print(self.lm_tree)
        
    def optimise(self,estimates) -> np.ndarray:
        # params = gtsam.LevenbergMarquardtParams()
        # #print(estimates)
        # optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, estimates,params)
        # #print(estimates)
        # result = optimizer.optimize()
        # self.estimates = result
        print("optimising")
        #print(self.graph)
        
        self.isam.update(self.graph, estimates)
        
        self.graph.resize(0)
        return self.isam.calculateEstimate()
    
    def add_initial_pose(self,key, state) -> None:
        self.graph.add(
            gtsam.PriorFactorPose2(key, gtsam.Pose2(state[0], state[1],state[2]), PRIOR_NOISE))
        
    def add_motion_constraint(self,x,x_next,motion) -> None: ## add a new pose with your recorded pose difference from last pose
        self.graph.add(
            gtsam.BetweenFactorPose2(x, x_next, gtsam.Pose2(motion), ODOMETRY_NOISE))
        
    def add_measurement_constraint(self, x, l, polarcoords) -> None:
        radius = polarcoords[0] 
        angle = polarcoords[1]
        self.graph.add(
            gtsam.BearingRangeFactor2D(x, l, gtsam.Rot2.fromDegrees(angle),
                                    float(radius), MEASUREMENT_NOISE))

    def _create_landmark(self, lm_x, lm_y, cone_type : ConeType):
        lm = Landmark(lm_x, lm_y)
        lm.update_cone_type(cone_type)
        key = gtsam.symbol('L', len(self.landmarks))
        #print(f"create_landmark keys: {key}")
        self.landmarks.append([key, lm]) # add the new landmark into our list, which is gonna get stored by main.py
        #lm_tree = self._generate_tree()
        return key#, lm_tree # give the id for this new landmark
        
    def _generate_tree(self):
        #print("hi")
        
        return KDTree([lm.pos() for key,lm in self.landmarks])
        #print(self.lm_tree)
        
    def find_landmark_association(self, lm, currpose, conetype: ConeType):
        """Use a threshold distance to find whether a landmark has already been seen, and an already registered landmark if so."""
        r = math.sqrt((lm[0] - currpose[0])**2 + (lm[1] - currpose[1])**2)#lm[0]
        theta = math.atan2( currpose[1] - lm[1], currpose[0] - lm[0])#lm[1]#math.atan3(lm[1] - currpose[1], lm[0] - currpose[0])#lm[1]
        lmx = r* np.cos(theta)
        lmy = r* np.sin(theta)
        #landmark_idx = -1
        dist = np.inf
        truex = lmx - currpose[0]
        #offsetx = lmx - currpose[0]
        truey = lmy - currpose[1]
        #offsety = lmy - currpose[1]
        #count = 0
        landmark = 0
        
        for key,lm in self.landmarks:
            ## lets first find out if the cones are roughly the same distance from the car
            checklmx = currpose[0]-lm.pos_x
            checklmy = currpose[1]-lm.pos_y
            checklmradius = np.sqrt(checklmx**2 + checklmy**2)
            checkdist = abs(checklmradius-r)
            if checkdist > 0.08: # right now just trying 1m
                #count+=1
                #print(count)
                continue
            
            ## from here now we know that we're dealing with 2 landmarks roughly the same distance from the car
            ## but we don't know if these are opposite landmarks on either side
            ## so what we do here is to then check the actual distance between the new observed cone and the one we've already plotted
            
            distancebetweenconesx = truex-lm.pos_x
            distancebetweenconesy = truey-lm.pos_y
            
            conedist = np.sqrt(distancebetweenconesx**2 + distancebetweenconesy**2)# here is the distance between landmarks
            
            if conedist < 0.1: # if the distance between the landmarks is < 0.5m, probably same landmark
                landmark = key
                continue
                
            
            landmark_idx = self._create_landmark(truex, truey, conetype)
            #print(landmark_idx)
            return landmark_idx, False
        return landmark, True
            
        #lm_estimate = Landmark(truex, truey)
        #print(lm_estimate.pos())
        # dist, array_index = self.lm_tree.query(lm_estimate.pos())
        # landmark_idx = self.landmarks[array_index][0]
        
        
        #temp = dist
        
        
        #print (f"Landmark id is {landmark_idx}")
        
        # if landmark_idx > len(self.landmarks) or landmark_idx < 0:
        #     least_distance_sqd = np.inf
        #     # standard brute force search
        #     for idx, landmark in self.landmarks:
        #         delta = lm_estimate.pos() - landmark.pos()
        #         distance_sqd = np.dot(delta, delta)
        #         if distance_sqd < least_distance_sqd:
        #             least_distance_sqd = distance_sqd
        #             landmark_idx = idx
                    
        #     dist = np.sqrt(least_distance_sqd)
        #print(landmark_idx >= len(self.landmarks))
        #print (f"Landmark id is {landmark_idx}")
        # if landmark_idx >= len(self.landmarks) or landmark_idx < 0:
        #     print("Warning: Invalid state! the landmarks are probably invalid")
        #     return -1

        #landmark = self.landmarks[landmark_idx]
        #print(dist > self.association_threshold)
        #print(self.association_threshold)
        #if dist > self.association_threshold:
            
            # make the landmark because its probably not the same one as before
            
        #print(f"2nd one fam Landmark id: {landmark_idx}")
        #landmark_idx = self.landmarks[array_index][0]
        #print(landmark_idx)
                
        #return landmark_idx, None # otherwise return the one thats already probably correct
        
    def add_pose_estimate(self, symbol, pose):
        self.estimates.insert(symbol, gtsam.Pose2(pose)) # insert that into the estimated true position of the poses

    def add_landmark_estimate(self,pose, symbol):
        #print(f"this is add landmark estimate {symbol}")
        self.estimates.insert(symbol, gtsam.Point2(pose[0],pose[1])) # insert that into the estimated true position of these landmarks
    
    def step(self, initial, pose_diff, pose, observations, previous_estimates, graph, landmarklist,poselist,isamgiven):
        self.graph = graph ## its messy i know :'(
        self.index = previous_estimates.size()
        self.landmarks = landmarklist
        self.estimates = previous_estimates
        #print(observations)
        self.poselist = poselist
        self.isam = isamgiven
        
        if len(self.poselist) == 0:
            new_x = gtsam.symbol('X', 0)
            self.add_initial_pose(new_x,initial)
            self.add_pose_estimate(new_x,pose)
            self.poselist.append(new_x)
        else:
            new_x = gtsam.symbol('X', len(self.poselist))
            last_x = self.poselist[-1]
            self.add_pose_estimate(new_x,pose) # finally add the estimated true pose position to the graph
            self.poselist.append(new_x)
            self.add_motion_constraint(last_x,new_x, pose_diff)
        
        for data in observations: # this is specifically for new_x, so we must check if any landmarks have already been seen
            lmpose = (data[0],data[1])
            conetype = data[2]
            if len(self.landmarks) >  0:                
                landmarkidx, status = self.find_landmark_association(lmpose, pose, conetype)
                #print(status)
                if landmarkidx != 0 and status == False:
                    print(lmpose)
                    self.add_measurement_constraint(new_x, landmarkidx, pose) # here the landmarkidx has been fixed by landmark association
                    self.add_landmark_estimate(lmpose,landmarkidx)
                elif status == True and landmarkidx!=0 :
                    self.add_measurement_constraint(new_x, landmarkidx, pose) # here the landmarkidx has been fixed by landmark association
            else:
                landmarkidx = self._create_landmark(lmpose[0]+pose[0], lmpose[1]+pose[1], conetype)
                print(landmarkidx)
                self.add_landmark_estimate(lmpose,landmarkidx)
                self.add_measurement_constraint(new_x, landmarkidx, pose) # here the landmarkidx has been fixed by landmark association
            if landmarkidx == -1:
                continue
            
             

        
        # for key in self.results.keys():         
        #     for index,(lmkey,lm) in enumerate(self.landmarks):
        #         if key == lmkey:
                    
        #             value = self.results.atPoint2(key)
        #             #print(lmkey)
        #             new_lm = Landmark(value[0], value[1])
        #             coneType = lm.get_cone_type()
        #             new_lm.update_cone_type(coneType)
        #             self.landmarks[index] = (key, Landmark(value[0],value[1])) # value has the x and ys
        #             #print(f"Updated {lm.pos()} to {value}")
        #             break
        
def main():
    """Main runner"""
    print("slamming")
        
if __name__ == "__main__":
    main()