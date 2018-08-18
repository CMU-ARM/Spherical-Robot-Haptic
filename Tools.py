import numpy as np

class Tools(object):

    def exponential_moving_average(last_avg, value, alpha):
        return alpha * value + (1-alpha)*last_avg

    @staticmethod
    def rotate_roll_pitch_yaw(vec, roll, pitch, yaw):

        Z_mat = np.eye(3)
        Z_mat[0,0] = Z_mat[1,1] = np.cos(yaw)
        Z_mat[0,1] = np.sin(yaw)
        Z_mat[1,0] = -np.sin(yaw)

        X_mat = np.eye(3)
        X_mat[1,1] = X_mat[2,2] = np.cos(roll)
        X_mat[1,2] = np.sin(roll)
        X_mat[2,1] = -np.sin(roll) 

        Y_mat = np.eye(3)
        Y_mat[2,2] = Y_mat[0,0] = np.cos(pitch)
        Y_mat[2,0] = np.sin(pitch)
        Y_mat[0,2] = -np.sin(pitch)

        rot_mat = X_mat.dot(Y_mat.dot(Z_mat))
        return rot_mat.dot(vec)

    @staticmethod
    def project_to_plane(vec,plane_normal):

        dot_prod = vec.dot(plane_normal)
        ortho_part = (dot_prod/(np.linalg.norm(plane_normal)**2)) * plane_normal
        proj_vec = vec - ortho_part
        return proj_vec
    
    @staticmethod
    def create_plane_from_roll_pitch(roll,pitch):
        #return the three basis vectors
        #rotate the x-basis vector
        x_basis = np.array([1,0,0])
        x_basis = Tools.rotate_roll_pitch_yaw(x_basis,0,np.deg2rad(pitch),0)

        y_basis = np.array([0,1,0])
        y_basis = Tools.rotate_roll_pitch_yaw(y_basis,np.deg2rad(roll),0,0)
        z_basis = np.cross(x_basis, y_basis)
        #print(x_basis)
        #print(y_basis)
        #print(z_basis)

        x_slope = z_basis[0]
        y_slope = z_basis[2]
        return (0,0,0),z_basis

    @staticmethod
    def get_2D_rotation(v1, v2):
        rot = np.arctan2(v2[1],v2[0]) - np.arctan2(v1[1],v1[0])
        return rot
        #sign = -1 if v2[1] < v1[1] else 1
        #rot = np.arccos(v2.dot(v1))
        #return rot * sign

def main():
    # point, normal = Tools.create_plane_from_roll_pitch(-4,-24)
    # vec = Tools.project_to_plane(normal,np.array([0,0,1]))
    # vec = vec/np.linalg.norm(vec)

    # print(vec)

    vec = np.array([-2000,500,0])
    vec = vec/np.linalg.norm(vec)
    print(vec)
    base_vec = np.array([0,-1,0])
    roll_val = np.rad2deg(Tools.get_2D_rotation(base_vec, vec))

    print(roll_val)


if __name__ == '__main__':
    main()