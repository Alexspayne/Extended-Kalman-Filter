import numpy as np

class KalmanFilter():
    """Tracks the state."""
    
    def __init__(self, noise):
#   VectorXd x_;

        self.x_ = None
#   // state covariance matrix
#   MatrixXd P_;
        self.P_ = np.matrix(np.ndarray((4,4)))
#   // state transistion matrix
#   MatrixXd F_;
        self.F_ = np.matrix([1, 0, 1, 0,
                             0, 1, 0, 1,
                             0, 0, 1, 0,
                             0, 0, 0, 1]).reshape(4,4)
#   // measurement matrix
#   MatrixXd H_;
        self.H_ = np.matrix([1, 0, 0, 0,
                             0, 1, 0, 0]).reshape(2,4)
#   // measurement covariance matrix
    #   MatrixXd R_;
        self.R_ = np.matrix([noise, 0,
                             0, noise]).reshape(2,2)
        
        self.R_radar = np.matrix([noise, 0, 0,
                                 0, noise, 0,
                                 0, 0, noise]).reshape(3, 3)
    def Predict(self):
        self.x_ = self.F_ * self.x_
        Ft = self.F_.transpose()
        self.P_ = self.F_ * self.P_ * Ft + self.Q_
        
    def Update(self, z):
#         print "Shape of z", z.shape()

        z_pred = self.H_ * self.x_

        z = np.matrix(z).reshape(2,1)
        y = z - z_pred
        Ht = self.H_.transpose()
        S = self.H_ * self.P_ * Ht + self.R_
        Si = S.I
        PHt = self.P_ * Ht
        K = PHt * Si

#       // new estimate

        self.x_ = self.x_ + (K * y)

        # Don't know why but x_ keeps getting assigned a datatype and it prevents matrix mult.
        self.x_.dtype = None
        x_size = len(self.x_)
        I = np.identity(x_size)

        self.P_ = (I - K * self.H_) * self.P_
        
    def UpdateEKF(self, z):
        
#         print "Shape of Hj_", self.Hj_.shape
        z_pred = h(self.x_)
#         print "Shape of z_pred", z_pred.shape
        z = np.matrix(z).reshape(3,1)
        y = z - z_pred
#         print "Shape of y", y.shape
        Ht = self.Hj_.transpose()
#         print "Shape of Ht", Ht.shape
        S = self.Hj_ * self.P_ * Ht + self.R_radar
#         print "Shape of S", S.shape
        Si = S.I
        PHt = self.P_ * Ht
#         print "Shape of PHt", PHt.shape
#         print "Shape of Si", Si.shape
        K = PHt * Si
#         print "Shape of K", K.shape

#       // new estimate

        self.x_ = self.x_ + (K * y)

        # Don't know why but x_ keeps getting assigned a datatype and it prevents matrix mult.
        self.x_.dtype = None
        x_size = len(self.x_)
        I = np.identity(x_size)

        self.P_ = (I - K * self.Hj_) * self.P_

class Tracking():
    """Tracks the state of the Kalman Filter."""
    def __init__(self, process_noise = 5, meas_noise = 0.0225):
        self.kf_ = KalmanFilter(meas_noise)
        self.previous_timestamp_ = 0
        self.noise_ax = process_noise
        self.noise_ay = process_noise
        self.is_initialized_ = False
    def ProcessMeasurement(self, measurement_pack):
        if not self.is_initialized_:
#             print( "Kalman Filter Initialization " )
            # Handle Lidar and Radar differently.
            if measurement_pack.sensor_type_ == "L":
                self.kf_.x_ = np.matrix([measurement_pack.raw_measurements_[0],
                          measurement_pack.raw_measurements_[1],
                          0, 0]).reshape(4,1)
                self.kf_.x_.dtype = None                
            elif measurement_pack.sensor_type_ == "R":
                ro = measurement_pack.raw_measurements_[0]
                phi = measurement_pack.raw_measurements_[1]
                p1_meas = ro * np.cos(phi);
                ps_meas = ro * np.sin(phi);
                self.kf_.x_ = np.matrix([p1_meas, ps_meas, 0, 0]).reshape(4,1)
                self.kf_.x_.dtype = None
                
            self.previous_timestamp_ = measurement_pack.timestamp_
            
            
            self.is_initialized_ = True
            return
        
        dt = (measurement_pack.timestamp_ - self.previous_timestamp_) / 1000000.0
        self.previous_timestamp_ = measurement_pack.timestamp_
        #print("Time: " + str(dt))
#         // 1. Modify the F matrix so that the time is integrated
        self.kf_.F_[0, 2] = dt;
        self.kf_.F_[1, 3] = dt
#         // 2. Set the process covariance matrix Q
        self.kf_.Q_ = np.matrix([(dt**4)*self.noise_ax/4, 0, (dt**3)*self.noise_ax/2, 0,
                                 0, (dt**4)*self.noise_ay/4, 0, (dt**3)*self.noise_ay/2,
                                 (dt**3)*self.noise_ax/2, 0, (dt**2)*self.noise_ax/1, 0,
                                 0, (dt**3)*self.noise_ay/2, 0, (dt**2)*self.noise_ay/1]).reshape(4,4)

#         // 3. Call the Kalman Filter predict() function
        self.kf_.Predict()
#         // 4. Call the Kalman Filter update() function
        if measurement_pack.sensor_type_ == "L":
            self.kf_.Update(measurement_pack.raw_measurements_)
        elif measurement_pack.sensor_type_ == "R":
            self.kf_.Hj_ = CalculateJacobian(self.kf_.x_)
            self.kf_.UpdateEKF(measurement_pack.raw_measurements_)
        
        
        #print("x_= " )
        #print(self.kf_.x_)
        #print("P_= ")
        #print(self.kf_.P_)

## Read the measurement file
class MeasurementPackage():
    def __init__(self):
        self.sensor_type_ = ""
        self.timestamp_ = 0
        self.raw_measurements_ = None
        
class GroundTruthPackage():
    def __init__(self):
        self.sensor_type_ = ""
        self.timestamp_ = 0
        self.gt_values_ = None

# Calculate RMSE

def CalculateRMSE(estimations, ground_truth):
    rmse = np.matrix([0.,0.,0.,0.]).reshape(4,1)

    if len(estimations) != len(ground_truth) or len(estimations) == 0:
        print "Not right size, too bad"
        return rmse

    for estimation, truth in zip(estimations, ground_truth):
        residual = estimation - truth.gt_values_
        
        residual = np.square(residual)
        rmse += residual
        
        #     Calculate the meain
        rmse = rmse/len(estimations)
        # Calculate the sqrt
        rmse = np.sqrt(rmse)

    return rmse

# Calculate Jacobian

def CalculateJacobian(x_state):
    """
    This is necessary for converting the 
    prediction position vector to the radar measurment space.    
    """
    px = float(x_state[0])
    py = float(x_state[1])
    vx = float(x_state[2])
    vy = float(x_state[3])
    
    c1 = px**2+py**2
    c2 = np.sqrt(c1)
    c3 = c1*c2
    
    if (np.abs(c1) < 0.0001):
        # print "CalculateJacobian () - Error - Division by Zero"
        Hj = np.matrix([
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0
            ]).reshape(3,4)
        return Hj;
      
    Hj = np.matrix([
            (px/c2), (py/c2), 0, 0,
            -(py/c1), (px/c1), 0, 0,
            py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2
        ]).reshape(3,4)
    
    return Hj

def h(x_state):
    "This converts to polar."
    px = float(x_state[0])
    py = float(x_state[1])
    vx = float(x_state[2])
    vy = float(x_state[3])
    
    c1 = px**2+py**2
    c2 = np.sqrt(c1)
    
    if (np.abs(c1) < 0.0001):
        # print "H - Error - Division by Zero"
        polar_state_vector = np.matrix([
                0,
                0,
                0]).reshape(3,1)
        return polar_state_vector
      
    polar_state_vector = np.matrix([c2,
                                    np.arctan(py/px),
                                    (px*vx+py*vy)/c2]).reshape(3,1)
    return polar_state_vector

def run_ekf(in_file_name_, noise_params = (.1, 10)):
    """
    An iteration of the EKF.
    It outputs the accuracy.
    This is to figure out what noise values I should use.
    """
    with open(in_file_name_, 'r') as f:
        data_lines = f.readlines()

    measurement_pack_list = []
    gt_pack_list = []

    # Extract data from line

    for line in data_lines:
        entry = line.split("\t")
        meas_package =  MeasurementPackage()
        gt_package = GroundTruthPackage()
        sensor_type = entry.pop(0)
        if sensor_type == "L":
            # Read Measurements
            meas_package.sensor_type_ = sensor_type
            x = entry.pop(0)
            y = entry.pop(0)
            meas_package.raw_measurements_ = (float(x), float(y))
            timestamp = entry.pop(0)
            meas_package.timestamp_ = float(timestamp)
            measurement_pack_list.append(meas_package)
        elif sensor_type == "R":
            meas_package.sensor_type_ = sensor_type
            ro = entry.pop(0)
            phi = entry.pop(0)
            ro_dot = entry.pop(0)
            meas_package.raw_measurements_ = map(float,(ro, phi, ro_dot))
            timestamp = entry.pop(0)
            meas_package.timestamp_ = float(timestamp)
            measurement_pack_list.append(meas_package)

        # Get the ground truth package

        x_gt, y_gt, vx_gt, vy_gt = map(float, entry)
        gt_matrix = np.matrix([x_gt, y_gt, vx_gt, vy_gt]).reshape(4,1)
        gt_package.gt_values_ = gt_matrix

        gt_pack_list.append(gt_package)


    tracking = Tracking(*noise_params)
    estimations = []
    for measurement, ground_truth in zip(measurement_pack_list, gt_pack_list):
        tracking.ProcessMeasurement(measurement)
        estimations.append(tracking.kf_.x_)
        
    rmse = CalculateRMSE(estimations, gt_pack_list)
    return rmse
