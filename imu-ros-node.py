#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from scipy.linalg import svd, qr
from scipy.spatial.transform import Rotation
import tf.transformations as tf_trans
from vqf import VQF

class IMUProcessor:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('imu_processor_node')
        
        # Parameters
        self.alpha = rospy.get_param('~complementary_filter_alpha', 0.96)
        self.zupt_acc_threshold = rospy.get_param('~zupt_acc_threshold', 0.1)  # m/s^2
        self.zupt_gyro_threshold = rospy.get_param('~zupt_gyro_threshold', 0.1)  # rad/s
        self.rtls_forgetting_factor = rospy.get_param('~rtls_forgetting_factor', 0.997)
        self.gravity = 9.81
        self.filename = "errors_log.txt"
        self.log_count = 0
        self.recursions = 0
        
        # State variables
        #self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion [x, y, z, w]
        self.orientation = None
        self.velocity = np.zeros(3)
        self.prev_time = None
        self.rtls_buffer_size = 1000
        self.A_matrices = []
        self.velocity_measurements = np.zeros(4)
        self.motor_speed = -1.7  #motor set speed
        
        
        # Publishers and Subscribers
        self.imu_sub = rospy.Subscriber('/IMU_data', Imu, self.imu_callback)
        self.velocity_pub = rospy.Publisher('/imu/velocity', Vector3, queue_size=10)
        self.installation_error_pub = rospy.Publisher('/imu/installation_error', Vector3, queue_size=10)

        self.vqf = VQF(0.01)

        self.system_init = False
        self.V_prev = None
        self.Sigma_prev = None

    def complementary_filter(self, accel, gyro, dt):
        """Update orientation using complementary filter"""
        # Normalize accelerometer readings
        accel_norm = np.linalg.norm(accel)
        if accel_norm > 0:
            accel = accel / accel_norm
            
        # Calculate orientation from accelerometer (roll and pitch only)
        roll = np.arctan2(accel[1], accel[2])
        pitch = np.arctan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2))
        
        # Convert to quaternion
        accel_quat = Rotation.from_euler('xyz', [roll, pitch, 0]).as_quat()
        
        # Integrate gyroscope
        gyro_quat = self.orientation + 0.5 * dt * np.array([
            -self.orientation[1]*gyro[0] - self.orientation[2]*gyro[1] - self.orientation[3]*gyro[2],
            self.orientation[0]*gyro[0] - self.orientation[3]*gyro[1] + self.orientation[2]*gyro[2],
            self.orientation[3]*gyro[0] + self.orientation[0]*gyro[1] - self.orientation[1]*gyro[2],
            -self.orientation[2]*gyro[0] + self.orientation[1]*gyro[1] + self.orientation[0]*gyro[2]
        ])
        
        # Normalize gyro quaternion
        gyro_quat = gyro_quat / np.linalg.norm(gyro_quat)
        
        # Complementary filter
        self.orientation = self.alpha * gyro_quat + (1 - self.alpha) * accel_quat
        self.orientation = self.orientation / np.linalg.norm(self.orientation)
        
        return self.orientation

    def remove_gravity(self, accel):
        """Remove gravity from acceleration based on current orientation"""
        #quat_xyzw = np.zeros(4)
        #quat_xyzw[0] = self.orientation[0]
        #quat_xyzw[1] = self.orientation[1]
        #quat_xyzw[2] = self.orientation[2]
        #quat_xyzw[3] = self.orientation[3]
        #rotation_matrix = Rotation.from_quat(self.orientation).as_matrix()
        rotation_matrix = tf_trans.quaternion_matrix(self.orientation)[:3, :3]
        gravity_vector = np.array([0, 0, self.gravity])
        #return accel - rotation_matrix @ gravity_vector
        return accel - rotation_matrix.T @ gravity_vector

    def zero_velocity_update(self, accel, gyro):
        """Check if IMU is stationary using ZUPT"""
        acc_magnitude = np.linalg.norm(accel)
        gyro_magnitude = np.linalg.norm(gyro)
        
        if (acc_magnitude < self.zupt_acc_threshold and
            gyro_magnitude < self.zupt_gyro_threshold):
            return True
        return False

    def rtls_update(self):
        """Update RTLS algorithm with latest measurements"""
            
        # Convert lists to numpy arrays
        #y_readings = np.array(self.velocity_measurements)
        #A_mats = np.array(self.A_matrices)
        
        # Run RTLS algorithm
        x = self.rtls_imu_errors(self.velocity_measurements, self.A_matrices)
        
        # Clear buffers
        #self.A_matrices = self.A_matrices[1:]
        #self.velocity_measurements = self.velocity_measurements[1:]
        
        return x  # Return latest estimate

    def rtls_imu_errors(self, y_readings, A_matrices, epsilon=1e-10):
        """RTLS algorithm implementation (from previous code)"""
        #n = len(y_readings)
        #x_values = []
        
        # Initial step
        if self.system_init is False:
            z = np.hstack((A_matrices, y_readings[:,None]))
            U, S, Vt = svd(z, True)
            V = Vt.T

            #v12 = V[:-1, -1:]
            #v22 = V[-1:, -1:]

            x = -np.linalg.solve(V[4:,4:].T, V[:4,4:].T).T
            #x_values.append(x)

            self.V_prev = V
            self.Sigma_prev = np.diag(S)
            self.Sigma_prev = np.concatenate((self.Sigma_prev, np.zeros((self.Sigma_prev.shape[0],1))), axis=1) 
            self.system_init = True
            return x
        
        if self.system_init is True:
            z = np.vstack((A_matrices.T, y_readings[:,None].T))          
            a = self.V_prev.T @ z
            #print(f"S matrix size is:{self.Sigma_prev.shape}")
            #print(f"a matrix size is:{a.shape}")
            M = np.vstack((
                0.997 * self.Sigma_prev,
                a.T
            ))
            #print(f"M matrix size is:{M.shape}")
            
            J, K = qr(z - self.V_prev @ a)
            v = np.sqrt(np.linalg.det(K.T @ K))
            
            if v < epsilon:
                P, N, Qt = svd(M,True)
                Sigma = np.diag(N)[:5,:5]
            else:
                P, Sigma, Qt = svd(M,False)
                Sigma = np.diag(Sigma)
                
            Q = Qt.T
            V_new = self.V_prev @ Q
            
            v12 = V_new[:-1, -1:]
            v22 = V_new[-1:, -1:]
            
            x = -np.linalg.solve(V_new[4:,4:].T, V_new[:4,4:].T).T
            #x_values.append(x)
            
            self.V_prev = V_new
            self.Sigma_prev = Sigma
            
        return x

    def create_A_matrix(self, omega):
        """Create A matrix from angular velocity"""
        #omega = np.linalg.norm(angular_velocity)
        return np.array([
            [0, omega, 0, 0],
            [-omega, 0, 0, 0],
            [0, 0, 0, omega],
            [0, 0, omega, 0]
        ])
        
    def log_to_file(self, installation_error):
    	errors = np.array(installation_error).reshape(1,-1)
    	
    	with open(self.filename, "a") as f:
    	    np.savetxt(f, errors, fmt="%.12f", delimiter="\t")

    def imu_callback(self, msg):
        """Process incoming IMU messages"""
        # Extract IMU data
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        print("acc with gravity", np.linalg.norm(accel))
        
        # Calculate time step
        current_time = rospy.Time.now()
        if self.prev_time is None:
            self.prev_time = current_time
            return
        dt = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time
        
        # Update orientation using VQF
        #self.complementary_filter(accel, gyro, dt)
        #self.vqf.update(gyro, accel)
        #self.orientation = self.vqf.getQuat6D()
        self.orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        # Remove gravity
        accel_no_gravity = self.remove_gravity(accel)
        print("acc without gravity", np.linalg.norm(accel_no_gravity))
        # Apply ZUPT

        # Integrate acceleration to get velocity
        if np.linalg.norm(accel_no_gravity) > 0.7 and np.linalg.norm(gyro) > 1.0: 
        	self.velocity += accel_no_gravity * 0.01
        if gyro[0] < -1.0:
            # Update RTLS buffers
            self.A_matrices = self.create_A_matrix(self.motor_speed)

            self.velocity_measurements[0] = gyro[1]
            self.velocity_measurements[1] = gyro[2]
            self.velocity_measurements[2] = self.velocity[1]
            self.velocity_measurements[3] = self.velocity[2]

            # Run RTLS if enough data is collected
            installation_error = self.rtls_update()
            print("installation error in a", installation_error[0])
            print("installation error in b", installation_error[1])
            print("installation error in y", installation_error[2])
            print("installation error in z", installation_error[3])
            self.recursions += 1
            
            if self.log_count < 1000 and self.recursions % 20 == 0:
            	installation_error_ = np.array([installation_error[0],installation_error[1],installation_error[2],installation_error[3]])
            	self.log_to_file(installation_error_)
            	self.log_count += 1

        # Publish results
        vel_msg = Vector3()
        vel_msg.x = self.velocity[0]
        vel_msg.y = self.velocity[1]
        vel_msg.z = self.velocity[2]
        self.velocity_pub.publish(vel_msg)
        
        
        #if installation_error is not None:
            #error_msg = Vector3()
            #error_msg.x = installation_error[0]
            #error_msg.y = installation_error[1]
            #error_msg.z = installation_error[2]
            #self.installation_error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        imu_processor = IMUProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
