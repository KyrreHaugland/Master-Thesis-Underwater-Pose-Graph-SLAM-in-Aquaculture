import numpy as np
from SLAMFunctions import rot3yaw
#import pyransac3d as pyrsc

class dvlPlaneEstimation:
    def __init__(self):
        self.gamma_angle = np.deg2rad(25)
        self.beta_angle = np.deg2rad(np.array([45,135,225,315]))
        self.camera_wrt_dvl = np.array([[0.08,0.17,0.20]])
    
    def calculate_beam_vector(self,dvl_beams,dvl_beams_flag):
        range_vectors = np.zeros((3,4))
        tan_gamma = np.tan(self.gamma_angle)
        for i in range(0,4):
            if (dvl_beams_flag[0,i]):
                range_vectors[:,i] = dvl_beams[0,i]*np.array([1,tan_gamma*np.cos(self.beta_angle[i]),tan_gamma*np.sin(self.beta_angle[i])]).T
        
        idx = np.argwhere(np.all(range_vectors[..., :] == 0, axis=0))
        filtered_range_vectors = np.delete(range_vectors, idx, axis=1)

        #fjerne kolonner som er 0
        return filtered_range_vectors
        
    

    def plane_estimation(self,dvl_beams,dvl_beams_flag):
        #inputs a 4
        filtered_range_vectors = self.calculate_beam_vector(dvl_beams,dvl_beams_flag).T
        #print(filtered_range_vectors)
        if filtered_range_vectors.shape[1]>=3:
            #plane = pyrsc.Plane()
            #normalize based on A
            #best_eq, best_inliers = plane.fit(filtered_range_vectors.T, 0.03)
            #print(best_eq)
            #best_eq /= best_eq[0]
            #print(best_eq)
            A = np.concatenate((filtered_range_vectors[:,1:],np.ones((filtered_range_vectors.shape[0],1))),axis=1)
            b = filtered_range_vectors[:,0]
            #print(b)
            #print(A)
            m = np.linalg.lstsq(A, b, rcond=None)[0] #estimer et plan med RANSAC heller?
            #print('plane_estimate;', m)
            #print('plane_estimate shape;', m.shape)
            return m
        else:
            return None
            #n_d = 
            #print(c)
        #TODO: need to check if there are enough valid beam readings
        #pass
    
    def get_camera_wrt_dvl(self):
        return self.camera_wrt_dvl

    #https://mathinsight.org/distance_point_plane good explanation
    def relative_distance_camera_to_net(self,plane_estimate):
        n_d = np.array([-1,plane_estimate[0],plane_estimate[1]])
        n_d_normalized = n_d/np.linalg.norm(n_d)

        #finding a point on the plane such that: -x0 +b*y0+c*z0 + d = 0 => if y0 = 0, z0 = 0 then x0 = d satisfies the equation
        y0 =0
        z0 =0
        x0 = plane_estimate[2]#d component of plane
        p0 = np.array([[x0,y0,z0]])

        #vector from point in plane to the camera
        v_d = self.get_camera_wrt_dvl() + p0

        #find distance
        distance_from_camera_to_net = np.abs(v_d@n_d_normalized)

        return distance_from_camera_to_net.item()

    @staticmethod
    def calculate_relative_heading(plane_estimate,yaw):
        n_d = np.array([[-1,plane_estimate[0],plane_estimate[1]]]).T
        Z_w = np.array([[0,0,1]]).T
        #n_d_normalized = n_d/np.linalg.norm(n_d)

        #find orientation of body relative to the world:
        Rw_b = rot3yaw(yaw) #litt usikker
        n_w = Rw_b@n_d

        #print('shape n_w; ',n_w.shape)
        #print('shape Z_w; ',Z_w.shape)
        #find heading where the DVL is pointed directly toward the net
        #print(np.cross(n_w[:,0],Z_w[:,0]))
        n_w_proj = -(np.cross(Z_w[:,0],np.cross(n_w[:,0],Z_w[:,0])))

        yaw_d = np.arctan2(n_w_proj[1],n_w_proj[0])

        return yaw_d
    
    def estimate_distance_camera_to_net_and_yaw(self,dvl_beams,dvl_beams_flag,heading):
        plane_estimate = self.plane_estimation(dvl_beams,dvl_beams_flag)
        
        if plane_estimate is None:
            return None, None, None
        else:
            n_d = np.array([-1,plane_estimate[0],plane_estimate[1]])
            n_d_normalized = n_d/np.linalg.norm(n_d)
            distance = self.relative_distance_camera_to_net(plane_estimate)
            yaw_d = self.calculate_relative_heading(plane_estimate,heading)
        return distance, yaw_d, n_d_normalized

def main():
    dvl_class = dvlPlaneEstimation()
    dvl_beams = 3*np.ones((1,4))
    dvl_beams_flag = np.ones((1,4))
    plane_estimate = dvl_class.plane_estimation(dvl_beams,dvl_beams_flag)
    distance = dvl_class.relative_distance_camera_to_net(plane_estimate)
    yaw_d = dvl_class.calculate_relative_heading(plane_estimate,np.deg2rad(45))
    print(distance)
    print('yaw_d: ',yaw_d)
    print("Hello World!")

if __name__ == "__main__":
    main()
