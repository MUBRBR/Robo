import numpy as np

class ParticleFilter():
    def __init__(self, min, max, landmarkList, n = 1000):
        self.particles = self.create_random_particles(n, min, max)

        #Update landmarks dict
        self.landmarks = dict()
        for key,x,y in landmarkList:
            self.landmarks[key] = (x,y)
        print("landmarks", self.landmarks)

    def create_random_particles(self, n, min, max): 
        particles = np.empty([n, 4])
        particles[:,0] = 600.0*np.random.rand(n) - 100.0
        particles[:,1] = 600.0*np.random.rand(n) - 250.0
        particles[:,2] = np.mod(2.0*np.pi*np.random.rand(n), 2.0*np.pi)
        particles[:,3] = 1/n

        return particles


    def move_particles(self, delta_x, delta_y, delta_theta):
        self.particles[:,0] += delta_x
        self.particles[:,1] += delta_y
        self.particles[:,2] += delta_theta


    def add_uncertainty(self, sigma, sigma_theta): 
        self.particles[:,0] += np.random.randn(self.particles.shape[0])*sigma
        self.particles[:,1] += np.random.randn(self.particles.shape[0])*sigma
        self.particles[:,2] = np.mod(self.particles[:,2] + np.random.randn(self.particles.shape[0]) * sigma_theta, 2.0 * np.pi)


    def estimate_pose(self):

        pos = np.mean(self.particles,axis=0)[:2]

        sin_sum = np.sum(np.sin(self.particles[:,2]))
        cos_sum = np.sum(np.cos(self.particles[:,2]))
        angle = np.arctan2(sin_sum/self.particles.shape[0], cos_sum/self.particles.shape[0])

        return np.array([pos[0],pos[1],angle])


    def evaluate_pose(self):
        #The position part is probably overcomplicated
        mean_position = self.estimate_pose()[:2]
        distances = np.sqrt(np.sum((self.particles[:,:2] - mean_position)**2, axis=1))
        std_d = np.std(distances)

        std_theta = np.std(self.particles[:,2])

        return std_d + std_theta
    

    def reset_weights(self):
        self.particles[:,3] = 1/self.particles.shape[0]


    def MCL(self, objectIDs, dists, angles, self_localize = False):
        
        #Get all indices that is not a reoccurring objectID
        unique_indices = [i for i in range(len(objectIDs)) if i == 0 and objectIDs[i] in self.landmarks.keys() or objectIDs[i - 1] != objectIDs[i] and objectIDs[i] in self.landmarks.keys()] 
        if (len(unique_indices) == 0):
            return
        #To get the product of weights, the initial weight must be 1
        self.particles[:,3] = 1.0

        # Compute particle weights for each detected landmark
        for i in unique_indices:
            curr_landmark, curr_dist, curr_angle = objectIDs[i], dists[i], angles[i]

            # Get sigma_d and sigma_theta

            sigma_d = 3 * np.std(np.sqrt(np.sum((self.landmarks[curr_landmark] - self.particles[:, :2])**2, axis=1)))
            sigma_theta = 2 * np.std(self.particles[:, 2])

            # Distance-part of weight
            measured_dists = np.sqrt(np.sum((self.landmarks[curr_landmark] - self.particles[:, :2])**2, axis=1))
            # first_term_d = 1 / (np.sqrt(2 * np.pi * sigma_d)) # forsøg
            # second_term_d = np.exp(-((measured_dists - curr_dist)**2) / (2 * sigma_d)) # forsøg
            first_term_d = 1 / (np.sqrt(2 * np.pi * (sigma_d**2))) # Anders original
            second_term_d = np.exp(-((measured_dists - curr_dist)**2) / (2 * (sigma_d**2))) #anders original
            weights_d = first_term_d * second_term_d

            # Angle-part of weight
            e_i_theta = np.column_stack([np.cos(self.particles[:, 2]), np.sin(self.particles[:, 2])])
            e_i_theta_hat = np.column_stack([-np.sin(self.particles[:, 2]), np.cos(self.particles[:, 2])])
            e_i_l = (self.landmarks[curr_landmark] - self.particles[:, :2]) / curr_dist
            # theta_i = np.sign(np.sum(e_i_l * e_i_theta_hat, axis=1)) * np.arccos(np.sum(e_i_l * e_i_theta, axis=1)) # anders original
            theta_i = np.sign(np.dot(e_i_l, e_i_theta_hat) * np.arccos(np.dot(e_i_l, e_i_theta))) # forsøg
            theta_i[np.isnan(theta_i)] = 0.0
            first_term_theta = 1 / (np.sqrt(2 * np.pi * (sigma_theta**2))) # Anders original
            # first_term_theta = 1 / (np.sqrt(2 * np.pi * sigma_theta)) #forsøg
            second_term_theta = np.exp(-((curr_angle - theta_i)**2) / (2 * (sigma_theta**2))) # anders original
            # second_term_theta = np.exp(-((curr_angle - theta_i)**2) / (2 * sigma_theta)) # forsøg
            weights_theta = first_term_theta * second_term_theta

            #Update particle weight
            epsilon = 0.00000001 # Hashtag numerical stability
            weights = weights_d * weights_theta + epsilon
            self.particles[:, 3] *= weights  # Weight is multiplied onto existing weight, as we consider multiple landmarks

        # Give a very low weight to any particles outside of bounds, when we are initially selvlocalizing - as here we´ll always be on an edge on the map. This is semi-hardocding and could be further harcoded so that only points on an edge (or close) are valid. may work really well.
        if (self_localize): 
            x_min, x_max = -10, 410
            y_min, y_max = -10, 410
            condition_x = (self.particles[:, 0] < x_min) | (self.particles[:, 0] > x_max)
            condition_y = (self.particles[:, 1] < y_min) | (self.particles[:, 1] > y_max)
            self.particles[(condition_x | condition_y), 3] *= 0.1
        
        #"Histogramize" the top 10%. Reasonning: We want the top x% to be evenly weighted in the probability distribution, in order to slow down the convergence on a single point. This creates the circle when looking at only one point
        sorted_indices = np.argsort(self.particles[:, 3]) # Get the indices to sort by weight            
        self.particles = self.particles[sorted_indices] # sort
        rows_to_change = int(len(self.particles) * 0.1) #10%
        self.particles[-rows_to_change:, 3] = np.mean(self.particles[-rows_to_change:, 3])

        # Define a normalized probability distribution
        probabilities = self.particles[:,3] / np.sum(self.particles[:,3])


        # Now draw new particles:
        selected_indices = np.random.choice(self.particles.shape[0], size=self.particles.shape[0], p=probabilities)

        # Update the particles array with the resampled particles
        self.particles = self.particles[selected_indices]