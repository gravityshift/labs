from grid import *
from particle import Particle
from utils import *
from setting import *
from numpy.random import choice

def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    for particle in particles:
        offset = rotate_point(odom[0], odom[1], particle.h)
        particle.x += offset[0]
        particle.y += offset[1]
        particle.h += odom[2]

    return particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments: 
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information, 
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    weights = []
    weight_sum = 0;
    for particle in particles:
        if (particle.x<0 or particle.x>grid.width):
            #If the particle is out of bounds on x, rule it out
            score = 0.0
        elif (particle.y<0 or particle.y>grid.height):
            #If the particle is out of bounds on y, rule it out
            score = 0.0
        else:
            #Give the particle a score based on sensor data
            min_diff = 10000
            for measured_marker in measured_marker_list:
                for particle_marker in particle.read_markers(grid):
                    diff = diff_markers(measured_marker, particle_marker)
                    if(diff<min_diff):
                        min_diff = diff

            score = 1/min_diff;

        weights.append(score)
        weight_sum += score

    #Normalize the weights
    for i in range(len(weights)):
        weights[i] = weights[i]/weight_sum

    #Resample the particles
    sampled_particles = []
    for sampled_particle in choice(particles, len(particles), p=weights):
        particle_tuple = (sampled_particle.x, sampled_particle.y, sampled_particle.h)
        permuted_tuple = add_marker_measurement_noise(particle_tuple, trans_sigma=0.1, rot_sigma=1)
        sampled_particles.append(Particle(permuted_tuple[0], permuted_tuple[1], permuted_tuple[2]))

    return sampled_particles

def diff_markers(marker1, marker2):
    if(map_angle_to_orientation(marker1[2]) != map_angle_to_orientation(marker2[2])):
        return 10000
    else:
        return grid_distance(marker1[0], marker1[1], marker2[0], marker2[1])


def map_angle_to_orientation(angle):
    angle = angle % 360
    if(angle>=45 and angle<135):
        return 'U'
    elif(angle>=135 and angle<225):
        return 'L'
    elif(angle>=225 and angle<315):
        return 'D'
    else:
        return 'R'



