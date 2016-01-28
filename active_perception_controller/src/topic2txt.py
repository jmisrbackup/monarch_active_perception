#!/usr/bin/python
import sys, rosbag
import math
import os.path
from math import log
from math import exp

import numpy as np
import matplotlib.pyplot as plt

###################################################
# Caculate the multivariate normal density (pdf)
# Keyword arguments:
#  x = numpy array of a "d x 1" sample vector
#  mu = numpy array of a "d x 1" mean vector
#  cov = "numpy array of a d x d" covariance matrix
###################################################

def pdf_multivariate_gauss(x, mu, cov):
    
    part1 = 1 / ( ((2* np.pi)**(len(mu)/2)) * (np.linalg.det(cov)**(1/2)) )
    part2 = (-1/2) * ((x-mu).T.dot(np.linalg.inv(cov))).dot((x-mu))
    return float(part1 * np.exp(part2))


#################
# Main function
#################

if len(sys.argv) != 3:
        print "\n---\nPlease, run ./topic2txt.py input_bag output_base_name\n---\n"	
else:

	file_path = sys.argv[2] + "_entropy.txt"

	if(os.path.exists(file_path)): 
		os.remove(file_path)

	entropy_file = open(file_path,"aw+")
	entropy_file.write("%secs\tnsecs\t\tRobot X\t\tRobot Y\t\tEntropy (monte-carlo)\tEntropy bound (GMM)\n")

	bag = rosbag.Bag(sys.argv[1])

	robot_x = 0
	robot_y = 0
	valid_pose = 0
	sigma_pose = 0.05	# Std. deviation for person movement
	n_samples = 100		# Samples for montecarlo integration

	entropies_gmm = []
	entropies_montecarlo = []

	for topic, msg, t in bag.read_messages():
		
		valid_pose = 1
		robot_x = 1
		robot_y= 1
		if topic=="/base_pose_ground_truth":
			valid_pose = 1
			robot_x = msg.pose.pose.position.x
			robot_y = msg.pose.pose.position.y

		if topic=="/person_particle_cloud" and valid_pose == 1:

			num_particles = len(msg.points)

			######################################
			# Compute entropy bound based on GMM
			######################################

			ent_bound_gmm = 0.0
			
			for i in range(num_particles):

				w = msg.channels[0].values[i]
				if w > 0:
					ent_bound_gmm = ent_bound_gmm + w*(-log(w) + 0.5*log( ((2*np.pi*exp(1))**2) * (sigma_pose**4) ) )
        
			##################################################################
			# Compute entropy approximation based on monte-carlo integration
			##################################################################

			ent_montecarlo = 0.0

			# Take a sample from distribution b(x), which is approximated from particles. 
			# We convolve each particle with a Gaussian with std. deviation = sigma_pose, obtaining a GMM. 
			# We can reduce hypotheses in GMM first by dismissing unlikely ones and merging similar ones 
			 			
			n_components = 0
			means = []
			covs = []
			weights = []

			prob_th = 0.00001/num_particles
			dist_th = 0
			total_weight = 0

			for i in range(num_particles):
				w = msg.channels[0].values[i]

				if w > prob_th:
					n_components = n_components + 1
					mean_comp = [msg.points[i].x, msg.points[i].y]
					cov_comp = [[sigma_pose**2, 0],[0,sigma_pose**2]]

					means.append(mean_comp)
					covs.append(cov_comp)
					weights.append(w)
					total_weight = total_weight + w

			for i in range(n_components):
				weights[i] = weights[i]/total_weight

			# If euclidian distance between C samples is lower than dist_th, we can merge them like
			# w_merged = sum_C {w_i} 
			# mu_merged = (1/w_merged) * sum_C {w_i * mu_i}
			# cov_merged = sum_C { (w_i/w_merged)* cov_i + (mu_i-mu_merged)*(mu_i-mu_merged)^T}

			print "New point cloud with " + str(num_particles) + " particles, converted to GMM with " + str(n_components) + " components.\n"

			for i in range(n_samples):
				
				# Sample the gaussian hyp_i from GMM
				sample = np.random.multinomial(1,weights)
	
				j = 0
				found = 0

				while found == 0 and j < len(sample):
					if sample[j] == 1:
						found = 1
						hyp_i = j

					j = j + 1


				# Sample x_i from the sampled Gaussian
				mean = [means[hyp_i][0], means[hyp_i][1]]
				cov = np.array(covs[hyp_i])
				x_i = np.random.multivariate_normal(mean,cov)

				x = np.array([[x_i[0]], [x_i[1]]])

				# Compute b(x_i) using the whole GMM
				px = 0.0

				for j in range(n_components):

					mu = np.array([[means[j][0]], [means[j][1]]])
					cov = np.array(covs[j])
					px = px +  weights[j]*pdf_multivariate_gauss(x, mu, cov)

				# Update entropy
				ent_montecarlo = ent_montecarlo - log(px) 				


			ent_montecarlo = ent_montecarlo/n_samples

			entropies_gmm.append(ent_bound_gmm)
			entropies_montecarlo.append(ent_montecarlo)

			# Write result
			entropy_file.write(str(t.secs)+"\t"+str(t.nsecs)+"\t"+str(robot_x)+"\t\t"+str(robot_y)+"\t\t"+str(ent_montecarlo)+"\t\t"+str(ent_bound_gmm)+"\n")
			
		
	entropy_file.close()

	t = np.arange(0, len(entropies_gmm), 1)
	plt.plot(t, entropies_gmm, 'r', t, entropies_montecarlo, 'k')
	plt.axis([0, len(entropies_gmm), 0, 7])
	plt.show()

	#plt.plot(entropies_gmm)
	#plt.ylabel('Entropy based on GMM bound')
	#plt.show()

	#plt.figure()
	#plt.plot(entropies_montecarlo)
	#plt.ylabel('Entropy based on Montecarlo simulations')
	#plt.show()


