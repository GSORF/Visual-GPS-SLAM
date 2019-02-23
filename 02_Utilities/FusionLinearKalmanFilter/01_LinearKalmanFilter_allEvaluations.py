'''
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

The Linear Kalman Filter implementation is provided by LIKE, Friedrich Alexander Universität Erlangen, Germany
'''

import numpy as np
from mathutils import Vector, Matrix
from copy import copy
import bpy

class LinearKalmanFilter():
    '''
    Linear Kalman Filter
    
    private:
        State Vector:               x
        State Covariance Matrix:    P
    
    public:
        __init__: Constructor initializes values
        
        predict: Prediction step 
        (x_k = Phik * x_k-1 + B * u + G * w)
        
        update: Update step
        (y = H * x_k + v)
        
    '''
    
    def __init__(self, x0, P0):
        '''
        Linear Kalman Filter Constructor
        initializes state vector and state covariance matrix
        '''
        self.x = x0          # State vector
        self.P = P0          # State Covariance Matrix
        self.L = np.size(x0) # Length
        print(x0)
    
    def predict(self, Phik, Qk, T, uk=0, Bk=0):
        # if Bk is not set array dimensions have to be set
        if(np.size(Bk) == 1 and Bk == 0):
            Bk = np.zeros((self.L))
        self.x = np.dot(Phik, self.x) + np.dot(Bk, uk)
        self.P = np.dot(Phik, np.dot(self.P, Phik.T)) + Qk
        return self.x, self.P
    
    def update(self, z, H, R):
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(np.dot(H, np.dot(self.P, H.T)) + R))
        y = z - np.dot(H, self.x)
        
        self.x = self.x + np.dot(K, y)
        K0 = np.eye(self.L) - np.dot(K, H)
        self.P = np.dot(K0, np.dot(self.P, K0.T))
        return self.x, self.P
        
        
'''
    Blender specifics:
'''
scene=bpy.context.scene
groundTruthObject = scene.objects["Camera_LinearConstantVelocity_GT"]
dsoObject = scene.objects["Camera_LinearConstantVelocity_DSO_Raw"]
gpsObject = scene.objects["Camera_LinearConstantVelocity_GPS_1m"]
filteredObject = scene.objects["Camera_LinearConstantVelocity_KF"]
covObject = scene.objects["CovarianceVisualization"]
txtxVector = scene.objects["TextX"]
txtPMatrix = scene.objects["TextP"]
fps = scene.render.fps
deltaT = 1.0/fps
frames = scene.frame_end-scene.frame_start

# Save Results in Blender globally, prepare lists:
bpy.types.Scene.KalmanFilter_x = []
bpy.types.Scene.KalmanFilter_P = []
# Display Text in Viewport
np.set_printoptions(precision=6)
bpy.app.handlers.frame_change_pre.clear()
def frameUpdate(scene):
    currentFrame = scene.frame_current-scene.frame_start
    #print("frameUpdate Handler:", currentFrame)
    x = np.array_str(bpy.types.Scene.KalmanFilter_x[currentFrame], precision=10)
    P = np.array_str(bpy.types.Scene.KalmanFilter_P[currentFrame], precision=10)
    scene.objects["TextX"].data.body = x
    scene.objects["TextP"].data.body = P
    
    #scene.objects["TextP"].data.body = str(scene.frame_current)
# Update Blender Object:
def objectUpdate(obj, frame, x, y, z = None):
    scene.frame_set(scene.frame_start + frame) # advance timeline to ith frame
    #print("Print UpdateStep:", scene.frame_start + frame)
    obj.location.x = x
    obj.location.y = y
    if not z==None:
        obj.location.z = z
    obj.keyframe_insert(data_path="location")
def getTranslationQuaternion(obj, frame):
    scene.frame_set(scene.frame_start + frame) # advance timeline to ith frame
    return obj.matrix_world.translation, obj.matrix_world.to_quaternion()
    
def plotResults(N, x, kf_x, zGPS, zDSO, rmse_gps, rmse_dso, rmse_filtered, rmse_velocity, _title, _description, _init_x, _init_y, _init_z, _sigma_p_gps, _sigma_p_dso, _gps_freq, _sigma_x0, _sigma_v0, _sigma_w_x, _sigma_w_v):
    rmse_gps_value_x = np.sqrt(np.sum(rmse_gps[:][0]**2) / (1.0*N) )
    rmse_gps_value_y = np.sqrt(np.sum(rmse_gps[:][1]**2) / (1.0*N) )
    rmse_gps_value = np.sqrt( np.sum( rmse_gps[:][0]**2 + rmse_gps[:][1]**2) / (1.0*N) )
    rmse_dso_value_x = np.sqrt(np.sum(rmse_dso[:][0]**2) / (1.0*N))
    rmse_dso_value_y = np.sqrt(np.sum(rmse_dso[:][1]**2) / (1.0*N))
    rmse_dso_value_z = np.sqrt(np.sum(rmse_dso[:][2]**2) / (1.0*N))
    rmse_dso_value = np.sqrt(np.sum( rmse_dso[:][0]**2 + rmse_dso[:][1]**2 + rmse_dso[:][2]**2) / (1.0*N))
    rmse_filtered_value_x = np.sqrt(np.sum(rmse_filtered[:][0]**2) / (1.0*N))
    rmse_filtered_value_y = np.sqrt(np.sum(rmse_filtered[:][1]**2) / (1.0*N))
    rmse_filtered_value_z = np.sqrt(np.sum(rmse_filtered[:][2]**2) / (1.0*N))
    rmse_filtered_value = np.sqrt(np.sum( rmse_filtered[:][0]**2 + rmse_filtered[:][1]**2 + rmse_filtered[:][2]**2) / (1.0*N))
    rmse_velocity_value_x = np.sqrt(np.sum(rmse_velocity[:][0]**2) / (1.0*N))
    rmse_velocity_value_y = np.sqrt(np.sum(rmse_velocity[:][1]**2) / (1.0*N))
    rmse_velocity_value_z = np.sqrt(np.sum(rmse_velocity[:][2]**2) / (1.0*N))
    rmse_velocity_value = np.sqrt(np.sum( rmse_velocity[:][0]**2 + rmse_velocity[:][1]**2 + rmse_velocity[:][2]**2) / (1.0*N))
    print("RMSE (GPS): ", rmse_gps_value_x, rmse_gps_value_y )
    print("RMSE (DSO): ", rmse_dso_value_x, rmse_dso_value_y, rmse_dso_value_z )
    print("RMSE (State): ", rmse_filtered_value_x, rmse_filtered_value_y, rmse_filtered_value_z )
        
    '''
    print("rmse_measurement: ", rmse_measurement)
    print("rmse_filtered data: ", rmse_filtered)
    print("x (ground truth) data: ", x[:,0:3:2])
    print("z (measurements) data: ", z)
    '''
    
    
    plot_filename = bpy.path.abspath("//") + ('Blender_%s_%s_%s_%s_%s_%s_%s_%s_%s_%s_%s_%s.txt'%(str(_title), str(_description), str(_init_x), str(_init_y), str(_init_z), str(_gps_freq), str(_sigma_p_gps), str(_sigma_p_dso), str(_sigma_x0), str(_sigma_v0), str(_sigma_w_x), str(_sigma_w_v) ))
    with open(plot_filename, 'w') as f:
        # Write accumulated RMSE errors:
        f.write("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n" % (rmse_gps_value_x, rmse_gps_value_y, rmse_gps_value, \
                                                rmse_dso_value_x, rmse_dso_value_y, rmse_dso_value_z, rmse_dso_value, \
                                                rmse_filtered_value_x, rmse_filtered_value_y, rmse_filtered_value_z, rmse_filtered_value, \
                                                rmse_velocity_value_x, rmse_velocity_value_y, rmse_velocity_value_z, rmse_velocity_value ))
        for frame in np.arange(0, N):
            # Write individual values per frame:
            f.write("%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n" % (scene.frame_start+frame, \
                                                    x[frame][0], x[frame][2], x[frame][4], \
                                                    zGPS[frame][0], zGPS[frame][1], \
                                                    zDSO[frame][0], zDSO[frame][2], zDSO[frame][4], \
                                                    kf_x[frame][0], kf_x[frame][2], kf_x[frame][4], \
                                                    rmse_gps[frame][0], rmse_gps[frame][1], \
                                                    rmse_dso[frame][0], rmse_dso[frame][1], rmse_dso[frame][2], \
                                                    rmse_filtered[frame][0], rmse_filtered[frame][1], rmse_filtered[frame][2], \
                                                    x[frame][1], x[frame][3], x[frame][5], \
                                                    kf_x[frame][1], kf_x[frame][3], kf_x[frame][5], \
                                                    rmse_velocity[frame][0], rmse_velocity[frame][1], rmse_velocity[frame][2] ) )
    
    
    
    
    
    
    return True

def visualizeCovariance(kf_x, P):
    # Visualize CoVariance Matrix P:
    # Source: https://geus.wordpress.com/2011/09/15/how-to-represent-a-3d-normal-function-with-ros-rviz/
    # get Eigenvalues and Eigenvectors of P:
    (eigValues, eigVectors) = np.linalg.eig(P) # Possible Bug: Required to reduce to 3x3 Matrix
    eigx = Vector( [eigVectors[0,0],eigVectors[0,2],eigVectors[0,4]] )
    eigy = Vector( [eigVectors[2,0],eigVectors[2,2],eigVectors[2,4]] )
    eigz = Vector( [eigVectors[4,0],eigVectors[4,2],eigVectors[4,4]] )
    eigx.normalize()
    eigy.normalize()
    eigz.normalize()
    # build rotation matrix:
    rot = Matrix( [eigx, eigy, eigz] )
    rot.transpose()
    # retrieve Quaternion
    quat = rot.to_quaternion()
    # update sphere object:
    covObject.location.x = kf_x[0]
    covObject.location.y = kf_x[2]
    covObject.location.z = kf_x[4]
    covObject.rotation_quaternion = quat
    covObject.scale.x = eigValues[0] * 100
    covObject.scale.y = eigValues[2] * 100
    covObject.scale.z = eigValues[4] * 100
    covObject.keyframe_insert(data_path="location")
    covObject.keyframe_insert(data_path="rotation_quaternion")
    covObject.keyframe_insert(data_path="scale")



# Simulation der Positionsmesswerte einer Bewegung mit konstanter Geschwindigkeit
# Parameter:
# - N   : Anzahl der Messungen
# - T   : Zeit zwischen zwei Messungen
# - vx  : konstante Geschwindigkeit in x
# - vy  : konstante Geschwindigkeit in y
# - sigma_n : 1x2 Vektor Messrauschen
def simulate_measurements_position(N, T, vx, vy, vz):
    # Zustandsuebergang   
    Phi = np.array([[1, T, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0],
                    [0, 0, 1, T, 0, 0],
                    [0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 1, T],
                    [0, 0, 0, 0, 0, 1]], dtype=float)
                    
    # Wahrer Zustandsraum
    x = np.zeros((N, 6))    # prepare Array with size: N times a 6D Vector [x, vx, y, vy, z, vz]
    
    # Go through animation to get ground truth data
    scene.frame_set(scene.frame_start) # advance timeline to first frame
    oldTranslation, oldQuaternion = getTranslationQuaternion(groundTruthObject, 0)
    print(oldTranslation)
    x[0, :] = np.array([oldTranslation.x, vx, oldTranslation.y, vy, oldTranslation.z, vz]) # initialize 0th element with the 6D Vector [x, vx, y, vy, z, vz]
    for frame in np.arange(1, N): # from state 1 to N
        #print("Get GroundTruth data:", frame)
        
        # get true positions and calculate velocities
        currentTranslation, currentQuaternion = getTranslationQuaternion(groundTruthObject, frame)
        pos_x = currentTranslation.x
        vel_x = (currentTranslation.x - oldTranslation.x) / deltaT
        pos_y = currentTranslation.y
        vel_y = (currentTranslation.y - oldTranslation.y) / deltaT
        pos_z = currentTranslation.z
        vel_z = (currentTranslation.z - oldTranslation.z) / deltaT
        x[frame, :] = np.array([pos_x, vel_x, pos_y, vel_y, pos_z, vel_z ]) # initialize ith element with the 6D Vector [x, vx, y, vy, z, vz]
        # COPY Problem (solved) print("posx, velx, posy, vely, posz, velz = ", pos_x, vel_x, pos_y, vel_y, pos_z, vel_z)
        # COPY Problem (solved) print("oldTranslation, currentTranslation = ", oldTranslation, currentTranslation)
        oldTranslation = copy(currentTranslation)
        oldQuaternion = copy(currentQuaternion)
        
    
    # GPS and DSO Measurements
    zGPS = np.zeros((N, 2))    # prepare Array with size: N times a 2D Vector [x, y] (NO z!!!)
    zDSO = np.zeros((N, 6))    # prepare Array with size: N times a 6D Vector [x, vx, y, vy, z, vz] (note: we need to keep both position and velocity for plotting!) 
    oldTranslation, oldQuaternion = getTranslationQuaternion(dsoObject, 0)
    #z = x[:,0:3:2] + np.random.randn(N, 2) * sigma_n # for every state, get every component and slice out index 0 to 3 and skip every second element (take index=0 and index=2)
    for frame in np.arange(0, N):
        gpsTranslation, gpsQuaternion = getTranslationQuaternion(gpsObject, frame) # (obj, frame, x, y)
        zGPS[frame][0] = gpsTranslation.x
        zGPS[frame][1] = gpsTranslation.y
        
        dsoTranslation, dsoQuaternion = getTranslationQuaternion(dsoObject, frame)
        zDSO[frame][0] = dsoTranslation.x
        zDSO[frame][1] = 1.0 * (dsoTranslation.x - oldTranslation.x) / deltaT
        zDSO[frame][2] = dsoTranslation.y
        zDSO[frame][3] = 1.0 * (dsoTranslation.y - oldTranslation.y) / deltaT
        zDSO[frame][4] = dsoTranslation.z
        zDSO[frame][5] = 1.0 * (dsoTranslation.z - oldTranslation.z) / deltaT
        
        oldTranslation = copy(dsoTranslation)
        oldQuaternion = copy(dsoQuaternion)

    
    #print("simulateMeasurements: ", zGPS, zDSO, x)
    return zGPS, zDSO, x

def simulate_measurements_velocity(N, T, vx, vy, vz):
    # Zustandsuebergang   
    Phi = np.array([[1, T, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0],
                    [0, 0, 1, T, 0, 0],
                    [0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 1, T],
                    [0, 0, 0, 0, 0, 1]], dtype=float)
                    
    # Wahrer Zustandsraum
    x = np.zeros((N, 6))    # prepare Array with size: N times a 6D Vector [x, vx, y, vy, z, vz]
    
    # Go through animation to get ground truth data
    scene.frame_set(scene.frame_start) # advance timeline to first frame
    oldTranslation, oldQuaternion = getTranslationQuaternion(groundTruthObject, 0)
    print(oldTranslation)
    x[0, :] = np.array([oldTranslation.x, vx, oldTranslation.y, vy, oldTranslation.z, vz]) # initialize 0th element with the 6D Vector [x, vx, y, vy, z, vz]
    for frame in np.arange(1, N): # from state 1 to N
        #print("Get GroundTruth data:", frame)
        
        # get true positions and calculate velocities
        currentTranslation, currentQuaternion = getTranslationQuaternion(groundTruthObject, frame)
        pos_x = currentTranslation.x
        vel_x = (currentTranslation.x - oldTranslation.x) / deltaT
        pos_y = currentTranslation.y
        vel_y = (currentTranslation.y - oldTranslation.y) / deltaT
        pos_z = currentTranslation.z
        vel_z = (currentTranslation.z - oldTranslation.z) / deltaT
        x[frame, :] = np.array([pos_x, vel_x, pos_y, vel_y, pos_z, vel_z ]) # initialize ith element with the 6D Vector [x, vx, y, vy, z, vz]
        # COPY Problem (solved) print("posx, velx, posy, vely, posz, velz = ", pos_x, vel_x, pos_y, vel_y, pos_z, vel_z)
        # COPY Problem (solved) print("oldTranslation, currentTranslation = ", oldTranslation, currentTranslation)
        oldTranslation = copy(currentTranslation)
        oldQuaternion = copy(currentQuaternion)
        
    
    # GPS and DSO Measurements
    zGPS = np.zeros((N, 2))    # prepare Array with size: N times a 2D Vector [x, y] (NO z!!!)
    zDSO = np.zeros((N, 6))    # prepare Array with size: N times a 6D Vector [x, vx, y, vy, z, vz] (note: we need to keep both position and velocity for plotting!) 
    oldTranslation, oldQuaternion = getTranslationQuaternion(dsoObject, 0)
    
    #z = x[:,0:3:2] + np.random.randn(N, 2) * sigma_n # for every state, get every component and slice out index 0 to 3 and skip every second element (take index=0 and index=2)
    for frame in np.arange(0, N):
        gpsTranslation, gpsQuaternion = getTranslationQuaternion(gpsObject, frame) # (obj, frame, x, y)
        zGPS[frame][0] = gpsTranslation.x
        zGPS[frame][1] = gpsTranslation.y
        
        dsoTranslation, dsoQuaternion = getTranslationQuaternion(dsoObject, frame)
        zDSO[frame][0] = dsoTranslation.x
        zDSO[frame][1] = 1.0 * (dsoTranslation.x - oldTranslation.x) / deltaT
        zDSO[frame][2] = dsoTranslation.y
        zDSO[frame][3] = 1.0 * (dsoTranslation.y - oldTranslation.y) / deltaT
        zDSO[frame][4] = dsoTranslation.z
        zDSO[frame][5] = 1.0 * (dsoTranslation.z - oldTranslation.z) / deltaT

        
        oldTranslation = copy(dsoTranslation)
        oldQuaternion = copy(dsoQuaternion)
    
    #print("simulateMeasurements: ", zGPS, zDSO, x)
    return zGPS, zDSO, x


def test_kf_position(_title="title", _description="description", _init_x=0.0, _init_y=0.0, _init_z=0.0, _sigma_p_gps=[1.0, 1.0], _sigma_p_dso=[5., 5., 5.], _gps_freq = 1, _sigma_x0 = 0.02, _sigma_v0 = 0.4, _sigma_w_x = 0.2, _sigma_w_v = 0.2):
    N = frames # NumFrames Messpunkte
    T = deltaT # Zeit zwischen Messungen

    # Initial Velocities
    vx = 0.0
    vy = 0.0
    vz = 0.0

    #sigma_p = np.array([5., 5.])
    sigma_p_gps = np.array(_sigma_p_gps) # std. deviation (x,y) of 1m was set in the VSLAM addon
    sigma_p_dso = np.array(_sigma_p_dso) # std. deviation (x,y,z) is not clear yet, so set pretty high

    # simulate measurements
    zGPS, zDSO, x = simulate_measurements_position(N,T, vx, vy, vz)
    
    # Kalman Init Parameter:
    x0 = np.array([_init_x, vx, _init_y, vy, _init_z, vz]);    # init measurements
    sigma_x0 = _sigma_x0 #std.dev. for position
    sigma_v0 = _sigma_v0  #std.dev. for velocity
    P0 = np.diag(np.array([sigma_x0, sigma_v0, sigma_x0, sigma_v0, sigma_x0, sigma_v0])**2)
    
    # Parameter der Systemrauschprozesse (Standardabweichungen)
    sigma_w_x = _sigma_w_x #1e-4
    sigma_w_v = _sigma_w_v #1e-10
    
    # System Uebergangsmatrix
    Phik = np.array([[1, T, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0],
                     [0, 0, 1, T, 0, 0],
                     [0, 0, 0, 1, 0, 0],
                     [0, 0, 0, 0, 1, T],
                     [0, 0, 0, 0, 0, 1]], dtype=float)    
    
    # System Rauschkovarianz
    T2 = np.power(T,2)
    T3 = T2 * T
    Qk = np.array([[sigma_w_v*T3/3 + sigma_w_v*T, sigma_w_v*T2/2,   0,                            0,                0,                            0                 ],
                   [sigma_w_v*T2/2,               sigma_w_v*T,      0,                            0,                0,                            0                 ],
                   [0,                            0,                sigma_w_v*T3/3 + sigma_w_v*T, sigma_w_v*T2/2,   0,                            0                 ],
                   [0,                            0,                sigma_w_v*T2/2,               sigma_w_v*T,      0,                            0                 ],
                   [0,                            0,                0,                            0,                sigma_w_v*T3/3 + sigma_w_v*T, sigma_w_v*T2/2    ],
                   [0,                            0,                0,                            0,                sigma_w_v*T2/2,               sigma_w_v*T       ]], dtype=float) 
    # Messmatrix
    # Only measure x and y position (GPS)
    H_GPS = np.array([[1, 0, 0, 0, 0, 0],
                     [0, 0, 1, 0, 0, 0]], dtype=float)
    # Measure x, y and z position (DSO)
    H_DSO = np.array([[1, 0, 0, 0, 0, 0],
                     [0, 0, 1, 0, 0, 0],
                     [0, 0, 0, 0, 1, 0]], dtype=float)
    # Messung Rauschkovarianz
    R_GPS = np.diag(sigma_p_gps**2)    
    R_DSO = np.diag(sigma_p_dso**2)    
    
    kf_x = np.zeros((N, 6))
    kf_x[0] = x0
    kf = LinearKalmanFilter(x0, P0)
    
    #Blender: Store initial state vector, covariance matrix and update object
    bpy.types.Scene.KalmanFilter_x.append(kf.x)
    bpy.types.Scene.KalmanFilter_P.append(kf.P)
    objectUpdate(filteredObject, 0, kf.x[0], kf.x[2], kf.x[4]) # (obj, frame, x, y, z)
    
    # Run Kalman filter:
    for frame in np.arange(1, N):
        #print("Kalman Filter step:", frame)
        kf.predict(Phik, Qk, T)
        
        if _title == "GPS":
            # Only use GPS as measurements
            if(frame % _gps_freq == 0):
                kf.update(zGPS[frame, :], H_GPS, R_GPS)
        elif _title == "DSO":
            # Only use DSO raw position as measurements (indices 0, 2, 4 for x,y,z)
            kf.update(zDSO[frame, 0::2], H_DSO, R_DSO)
        else:
            # Fuse GPS and DSO
            # -> GPS: Only take new Measurements every 5th iteration:
            if(frame % _gps_freq == 0):
                kf.update(zGPS[frame, :], H_GPS, R_GPS)
            # -> DSO: Take new raw dso position Measurements every timestep:
            kf.update(zDSO[frame, 0::2], H_DSO, R_DSO)
            
        
        # Speichere aktuellen Zustand
        kf_x[frame, :] = kf.x
        
        # Update filtered Blenderobject
        objectUpdate(filteredObject, frame, kf.x[0], kf.x[2], kf.x[4]) # (obj, frame, x, y, z)
        
        # Visualize Covariance Matrix
        visualizeCovariance(kf.x, kf.P)
        
        # Store P Matrix from KalmanFilter for display
        bpy.types.Scene.KalmanFilter_x.append(kf.x)
        bpy.types.Scene.KalmanFilter_P.append(kf.P)
        
    # Remove all assigned handler functions:
    bpy.app.handlers.frame_change_pre.clear()
    # Now register a handler function to update the 3D Data Plot
    bpy.app.handlers.frame_change_pre.append(frameUpdate)
    
    # Save file for plotting via matplotlib
    rmse_gps = np.zeros((N, 2))
    rmse_dso = np.zeros((N, 3))
    rmse_filtered = np.zeros((N, 3))
    rmse_velocity = np.zeros((N, 3))
    
    for frame in np.arange(0, N):
        rmse_gps[frame][0] = zGPS[frame][0] - x[frame][0]
        rmse_gps[frame][1] = zGPS[frame][1] - x[frame][2]
        rmse_dso[frame][0] = zDSO[frame][0] - x[frame][0]
        rmse_dso[frame][1] = zDSO[frame][2] - x[frame][2]
        rmse_dso[frame][2] = zDSO[frame][4] - x[frame][4]
        rmse_filtered[frame][0] = kf_x[frame][0] - x[frame][0]
        rmse_filtered[frame][1] = kf_x[frame][2] - x[frame][2]
        rmse_filtered[frame][2] = kf_x[frame][4] - x[frame][4]
        # Velocity
        rmse_velocity[frame][0] = kf_x[frame][1] - x[frame][1]
        rmse_velocity[frame][1] = kf_x[frame][3] - x[frame][3]
        rmse_velocity[frame][2] = kf_x[frame][5] - x[frame][5]
    
    '''
    print(rmse_dso[:][0])
    print("now comes filtered:")
    print(rmse_filtered[:][0])
    '''
    
    plotResults(N, x, kf_x, zGPS, zDSO, rmse_gps, rmse_dso, rmse_filtered, rmse_velocity, _title, _description, _init_x, _init_y, _init_z, _sigma_p_gps, _sigma_p_dso, _gps_freq, _sigma_x0, _sigma_v0, _sigma_w_x, _sigma_w_v)
    
    
    
    '''
    '''
    #End of Main Loop and plotting        
    
def test_kf_velocity(_title="title", _description="description", _init_x=0.0, _init_y=0.0, _init_z=0.0, _sigma_p_gps=[1.0, 1.0], _sigma_p_dso=[5., 5., 5.], _gps_freq = 1, _sigma_x0 = 0.02, _sigma_v0 = 0.4, _sigma_w_x = 0.2, _sigma_w_v = 0.2):
    N = frames # NumFrames Messpunkte
    T = deltaT # Zeit zwischen Messungen

    # Initial Velocities
    vx = 0.0
    vy = 0.0
    vz = 0.0

    #sigma_p = np.array([5., 5.])
    sigma_p_gps = np.array(_sigma_p_gps) # std. deviation (x,y) of 1m was set in the VSLAM addon
    sigma_p_dso = np.array(_sigma_p_dso) # std. deviation (x,y,z) is not clear yet, so set pretty high

    # simulate measurements
    zGPS, zDSO, x = simulate_measurements_velocity(N,T, vx, vy, vz)
    
    # Kalman Init Parameter:
    x0 = np.array([_init_x, vx, _init_y, vy, _init_z, vz]);    # init measurements
    sigma_x0 = _sigma_x0 #std.dev. for position
    sigma_v0 = _sigma_v0  #std.dev. for velocity
    P0 = np.diag(np.array([sigma_x0, sigma_v0, sigma_x0, sigma_v0, sigma_x0, sigma_v0])**2)
    
    # Parameter der Systemrauschprozesse (Standardabweichungen)
    sigma_w_x = _sigma_w_x #1e-4
    sigma_w_v = _sigma_w_v #1e-10
    
    # System Uebergangsmatrix
    Phik = np.array([[1, T, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0],
                     [0, 0, 1, T, 0, 0],
                     [0, 0, 0, 1, 0, 0],
                     [0, 0, 0, 0, 1, T],
                     [0, 0, 0, 0, 0, 1]], dtype=float)    
    
    # System Rauschkovarianz
    T2 = np.power(T,2)
    T3 = T2 * T
    Qk = np.array([[sigma_w_v*T3/3 + sigma_w_x*T, sigma_w_v*T2/2,   0,                            0,                0,                            0                 ],
                   [sigma_w_v*T2/2,               sigma_w_v*T,      0,                            0,                0,                            0                 ],
                   [0,                            0,                sigma_w_v*T3/3 + sigma_w_v*T, sigma_w_v*T2/2,   0,                            0                 ],
                   [0,                            0,                sigma_w_v*T2/2,               sigma_w_v*T,      0,                            0                 ],
                   [0,                            0,                0,                            0,                sigma_w_v*T3/3 + sigma_w_v*T, sigma_w_v*T2/2    ],
                   [0,                            0,                0,                            0,                sigma_w_v*T2/2,               sigma_w_v*T       ]], dtype=float) 
    # Messmatrix
    # Only measure x and y position (GPS)
    H_GPS = np.array([[1, 0, 0, 0, 0, 0],
                     [0, 0, 1, 0, 0, 0]], dtype=float)
    # Measure x, y and z VELOCITY (DSO)
    H_DSO = np.array([[0, 1, 0, 0, 0, 0],
                     [0, 0, 0, 1, 0, 0],
                     [0, 0, 0, 0, 0, 1]], dtype=float)
    # Messung Rauschkovarianz
    R_GPS = np.diag(sigma_p_gps**2)    
    R_DSO = np.diag(sigma_p_dso**2)    
    
    kf_x = np.zeros((N, 6))
    kf_x[0] = x0
    kf = LinearKalmanFilter(x0, P0)
    
    #Blender: Store initial state vector, covariance matrix and update object
    bpy.types.Scene.KalmanFilter_x.append(kf.x)
    bpy.types.Scene.KalmanFilter_P.append(kf.P)
    objectUpdate(filteredObject, 0, kf.x[0], kf.x[2], kf.x[4]) # (obj, frame, x, y, z)
    
    # Run Kalman filter:
    for frame in np.arange(1, N):
        #print("Kalman Filter step:", frame)
        kf.predict(Phik, Qk, T)
        
        if _title == "GPS":
            # Only use GPS as measurements
            if(frame % _gps_freq == 0):
                kf.update(zGPS[frame, :], H_GPS, R_GPS)
        elif _title == "DSO":
            # Only use DSO as measurements (indices 1, 3, 5 for vx,vy,vz)
            kf.update(zDSO[frame, 1::2], H_DSO, R_DSO)
        else:
            # Fuse GPS and DSO
            # -> GPS: Only take new Measurements every 5th iteration:
            if(frame % _gps_freq == 0):
                kf.update(zGPS[frame, :], H_GPS, R_GPS)
            # -> DSO: Take new Measurements every timestep:
            kf.update(zDSO[frame, 1::2], H_DSO, R_DSO)
            
        
        # Speichere aktuellen Zustand
        kf_x[frame, :] = kf.x
        
        # Update filtered Blenderobject
        objectUpdate(filteredObject, frame, kf.x[0], kf.x[2], kf.x[4]) # (obj, frame, x, y, z)
        
        # Visualize Covariance Matrix
        visualizeCovariance(kf.x, kf.P)
        
        # Store P Matrix from KalmanFilter for display
        bpy.types.Scene.KalmanFilter_x.append(kf.x)
        bpy.types.Scene.KalmanFilter_P.append(kf.P)
        
    # Remove all assigned handler functions:
    bpy.app.handlers.frame_change_pre.clear()
    # Now register a handler function to update the 3D Data Plot
    bpy.app.handlers.frame_change_pre.append(frameUpdate)
    
    # Save file for plotting via matplotlib
    rmse_gps = np.zeros((N, 2))
    rmse_dso = np.zeros((N, 3))
    rmse_filtered = np.zeros((N, 3))
    rmse_velocity = np.zeros((N, 3))
    
    for frame in np.arange(0, N):
        rmse_gps[frame][0] = zGPS[frame][0] - x[frame][0]
        rmse_gps[frame][1] = zGPS[frame][1] - x[frame][2]
        rmse_dso[frame][0] = zDSO[frame][0] - x[frame][0]
        rmse_dso[frame][1] = zDSO[frame][2] - x[frame][2]
        rmse_dso[frame][2] = zDSO[frame][4] - x[frame][4]
        rmse_filtered[frame][0] = kf_x[frame][0] - x[frame][0]
        rmse_filtered[frame][1] = kf_x[frame][2] - x[frame][2]
        rmse_filtered[frame][2] = kf_x[frame][4] - x[frame][4]
        # Velocity
        rmse_velocity[frame][0] = kf_x[frame][1] - x[frame][1]
        rmse_velocity[frame][1] = kf_x[frame][3] - x[frame][3]
        rmse_velocity[frame][2] = kf_x[frame][5] - x[frame][5]
    
    '''
    print(rmse_dso[:][0])
    print("now comes filtered:")
    print(rmse_filtered[:][0])
    '''
    
    plotResults(N, x, kf_x, zGPS, zDSO, rmse_gps, rmse_dso, rmse_filtered, rmse_velocity, _title, _description, _init_x, _init_y, _init_z, _sigma_p_gps, _sigma_p_dso, _gps_freq, _sigma_x0, _sigma_v0, _sigma_w_x, _sigma_w_v)
    
    
    
    #End of Main Loop and plotting


    
if __name__ == "__main__":
    '''
    The idea is to test various scenarios:
    
    1.) Raw GPS Position only filtering
    2.) Raw DSO Position only filtering
    3.) Manually Scaled DSO Trajectory and Fusion of GPS and DSO by using Position measurements (Expectation: Filtered Trajectory very close to GroundTruth, but unrealistically)
    4.) Raw output DSO Trajectory and Fusion of GPS and DSO by using Position measurements (Expectation: Oscilating Kalman Filtering between GPS and DSO)
    5.) Raw output DSO Trajectory and Fusion of GPS Position and DSO Velocity measurements (Expectation: Proper approach to perform fusion and get a notion of scale and drift offset)
    '''
    
    
    
    '''
    Example: Linear Constant Velocity:
        GPS 1m std. dev.
    ---------------------------------------
    '''
    scene.frame_end = 1001
    frames = scene.frame_end-scene.frame_start
    
    # Position manually scaled:
    groundTruthObject = scene.objects["Camera_LinearConstantVelocity_GT"]
    dsoObject = scene.objects["Camera_LinearConstantVelocity_DSO_Scaled"]
    gpsObject = scene.objects["Camera_LinearConstantVelocity_GPS_1m"]
    filteredObject = scene.objects["Camera_LinearConstantVelocity_KF"]
    
    # Only GPS updates
    test_kf_position(_title="GPS",_description="Linear Constant Velocity 1m Only GPS Updates, every frame", _gps_freq = 1)
    test_kf_position(_title="GPS",_description="Linear Constant Velocity 1m Only GPS Updates, every 50th frame", _gps_freq = 50)
    test_kf_position(_title="GPS",_description="Linear Constant Velocity 1m Only GPS Updates, every 50th frame, initialized", _gps_freq = 50, _init_x=-30.0, _init_y=-9.3)
    
    # Only DSO updates
    test_kf_position(_title="DSO",_description="Linear Constant Velocity 1m Only DSO updates")
    
    # Manually scaled
    test_kf_position(_title="GPS+DSO",_description="Linear Constant Velocity 1m GPS Position Manually scaled", _gps_freq = 1)
    test_kf_position(_title="GPS+DSO",_description="Linear Constant Velocity 1m GPS Position Manually scaled", _gps_freq = 50)
    test_kf_position(_title="GPS+DSO",_description="Linear Constant Velocity 1m GPS Position Manually scaled", _gps_freq = 50, _init_x=-30.0, _init_y=-9.3)
    
    # Position raw:
    dsoObject = scene.objects["Camera_LinearConstantVelocity_DSO_Raw"]
    test_kf_position(_title="GPS+DSO",_description="Linear Constant Velocity 1m GPS Position raw", _gps_freq = 1)
    test_kf_position(_title="GPS+DSO",_description="Linear Constant Velocity 1m GPS Position raw", _gps_freq = 50)
    test_kf_position(_title="GPS+DSO",_description="Linear Constant Velocity 1m GPS Position raw", _gps_freq = 50, _init_x=-30.0, _init_y=-9.3)
    
    # Position+Velocity raw:
    test_kf_velocity(_title="GPS+DSO",_description="Linear Constant Velocity 1m GPS Velocity raw",_gps_freq = 1)
    test_kf_velocity(_title="GPS+DSO",_description="Linear Constant Velocity 1m GPS Velocity raw",_gps_freq = 50)
    test_kf_velocity(_title="GPS+DSO",_description="Linear Constant Velocity 1m GPS Velocity raw",_gps_freq = 50, _init_x=-30.0, _init_y=-9.3)
    
    '''
    Example: Linear Constant Velocity:
        GPS 10m std. dev.
    ---------------------------------------
    '''
    
    dsoObject = scene.objects["Camera_LinearConstantVelocity_DSO_Scaled"]
    gpsObject = scene.objects["Camera_LinearConstantVelocity_GPS_10m"]
    test_kf_position(_title="GPS+DSO",_description="Linear Constant Velocity 10m GPS Position Manually scaled", _gps_freq = 1)
    test_kf_position(_title="GPS+DSO",_description="Linear Constant Velocity 10m GPS Position Manually scaled", _gps_freq = 50)
    test_kf_position(_title="GPS+DSO",_description="Linear Constant Velocity 10m GPS Position Manually scaled", _gps_freq = 50, _init_x=-30.0, _init_y=-9.3)
    
    # Position raw:
    dsoObject = scene.objects["Camera_LinearConstantVelocity_DSO_Raw"]
    test_kf_position(_title="GPS+DSO",_description="Linear Constant Velocity 10m GPS Position raw", _gps_freq = 1)
    test_kf_position(_title="GPS+DSO",_description="Linear Constant Velocity 10m GPS Position raw", _gps_freq = 50)
    test_kf_position(_title="GPS+DSO",_description="Linear Constant Velocity 10m GPS Position raw", _gps_freq = 50, _init_x=-30.0, _init_y=-9.3)
    
    # Position+Velocity raw:
    test_kf_velocity(_title="GPS+DSO",_description="Linear Constant Velocity 10m GPS Velocity raw",_gps_freq = 50)
    test_kf_velocity(_title="GPS+DSO",_description="Linear Constant Velocity 10m GPS Velocity raw",_gps_freq = 50, _init_x=-30.0, _init_y=-9.3)




    '''
    Example: Scene City Loop Closure
        GPS 1m std. dev.
    ---------------------------------------
    '''
    scene.frame_end = 599
    frames = scene.frame_end-scene.frame_start

    groundTruthObject = scene.objects["Camera_withRotation_GT"]
    dsoObject = scene.objects["Camera_withRotation_DSO_Scaled"]
    gpsObject = scene.objects["Camera_withRotation_GPS_1m"]
    filteredObject = scene.objects["Camera_withRotation_KF"]
    
    
    # Only GPS updates
    test_kf_position(_title="GPS",_description="SceneCity Loop 1m Only GPS Updates, every frame", _gps_freq = 1)
    test_kf_position(_title="GPS",_description="SceneCity Loop 1m Only GPS Updates, every 50th frame", _gps_freq = 50)
    test_kf_position(_title="GPS",_description="SceneCity Loop 1m Only GPS Updates, every 50th frame, initialized", _gps_freq = 50, _init_x=1.02, _init_y=-4)
    
    # Only DSO updates
    test_kf_position(_title="DSO",_description="SceneCity Loop 1m Only DSO updates")
    
    # Manually scaled
    test_kf_position(_title="GPS+DSO",_description="SceneCity Loop 1m GPS Position Manually scaled", _gps_freq = 1)
    test_kf_position(_title="GPS+DSO",_description="SceneCity Loop 1m GPS Position Manually scaled", _gps_freq = 50)
    test_kf_position(_title="GPS+DSO",_description="SceneCity Loop 1m GPS Position Manually scaled", _gps_freq = 50, _init_x=1.02, _init_y=-4)
    
    # Position raw:
    dsoObject = scene.objects["Camera_withRotation_DSO_Raw"]
    test_kf_position(_title="GPS+DSO",_description="SceneCity Loop 1m GPS Position raw", _gps_freq = 1)
    test_kf_position(_title="GPS+DSO",_description="SceneCity Loop 1m GPS Position raw", _gps_freq = 50)
    test_kf_position(_title="GPS+DSO",_description="SceneCity Loop 1m GPS Position raw", _gps_freq = 50, _init_x=1.02, _init_y=-4)
    
    # Position+Velocity raw:
    test_kf_velocity(_title="GPS+DSO",_description="SceneCity Loop 1m GPS Velocity raw",_gps_freq = 1)
    test_kf_velocity(_title="GPS+DSO",_description="SceneCity Loop 1m GPS Velocity raw",_gps_freq = 50)
    test_kf_velocity(_title="GPS+DSO",_description="SceneCity Loop 1m GPS Velocity raw",_gps_freq = 50, _init_x=1.02, _init_y=-4)


    '''
    Example: Scene City Loop Closure
        GPS 10m std. dev.
    ---------------------------------------
    '''

    dsoObject = scene.objects["Camera_withRotation_DSO_Scaled"]
    gpsObject = scene.objects["Camera_withRotation_GPS_10m"]
    test_kf_position(_title="GPS+DSO",_description="SceneCity Loop 10m GPS Position Manually scaled", _gps_freq = 1)
    test_kf_position(_title="GPS+DSO",_description="SceneCity Loop 10m GPS Position Manually scaled", _gps_freq = 50)
    test_kf_position(_title="GPS+DSO",_description="SceneCity Loop 10m GPS Position Manually scaled", _gps_freq = 50, _init_x=1.02, _init_y=-4)
    
    # Position raw:
    dsoObject = scene.objects["Camera_withRotation_DSO_Raw"]
    test_kf_position(_title="GPS+DSO",_description="SceneCity Loop 10m GPS Position raw", _gps_freq = 1)
    test_kf_position(_title="GPS+DSO",_description="SceneCityLoop 10m GPS Position raw", _gps_freq = 50)
    test_kf_position(_title="GPS+DSO",_description="SceneCityLoop 10m GPS Position raw", _gps_freq = 50, _init_x=1.02, _init_y=-4)
    
    # Position+Velocity raw:
    test_kf_velocity(_title="GPS+DSO",_description="SceneCity Loop 10m GPS Velocity raw",_gps_freq = 1)
    test_kf_velocity(_title="GPS+DSO",_description="SceneCity Loop 10m GPS Velocity raw",_gps_freq = 50)
    test_kf_velocity(_title="GPS+DSO",_description="SceneCity Loop 10m GPS Velocity raw",_gps_freq = 50, _init_x=1.02, _init_y=-4)



    '''
    Example: Venice
        GPS 1m std. dev.
    ---------------------------------------
    '''
    scene.frame_end = 999
    frames = scene.frame_end-scene.frame_start

    groundTruthObject = scene.objects["Camera_Venice_GT"]
    dsoObject = scene.objects["Camera_Venice_DSO_Scaled"]
    gpsObject = scene.objects["Camera_Venice_GPS_1m"]
    filteredObject = scene.objects["Camera_Venice_KF"]
    
    # Only GPS updates
    test_kf_position(_title="GPS",_description="Venice Loop 1m Only GPS Updates, every frame", _gps_freq = 1)
    test_kf_position(_title="GPS",_description="Venice Loop 1m Only GPS Updates, every 50th frame", _gps_freq = 50)
    test_kf_position(_title="GPS",_description="Venice Loop 1m Only GPS Updates, every 50th frame, initialized", _gps_freq = 50, _init_x=-992.412, _init_y=362.021)
    
    # Only DSO updates
    test_kf_position(_title="DSO",_description="Venice Loop 1m Only DSO updates")
    
    # Manually scaled
    test_kf_position(_title="GPS+DSO",_description="Venice Loop 1m GPS Position Manually scaled", _gps_freq = 1)
    test_kf_position(_title="GPS+DSO",_description="Venice Loop 1m GPS Position Manually scaled", _gps_freq = 50)
    test_kf_position(_title="GPS+DSO",_description="Venice Loop 1m GPS Position Manually scaled", _gps_freq = 50, _init_x=-992.412, _init_y=362.021)
    
    # Position raw:
    dsoObject = scene.objects["Camera_Venice_DSO_Raw"]
    test_kf_position(_title="GPS+DSO",_description="Venice Loop 1m GPS Position raw", _gps_freq = 1)
    test_kf_position(_title="GPS+DSO",_description="Venice Loop 1m GPS Position raw", _gps_freq = 50)
    test_kf_position(_title="GPS+DSO",_description="Venice Loop 1m GPS Position raw", _gps_freq = 50, _init_x=-992.412, _init_y=362.021)
    
    # Position+Velocity raw:
    test_kf_velocity(_title="GPS+DSO",_description="Venice Loop 1m GPS Velocity raw",_gps_freq = 1)
    test_kf_velocity(_title="GPS+DSO",_description="Venice Loop 1m GPS Velocity raw",_gps_freq = 50)
    test_kf_velocity(_title="GPS+DSO",_description="Venice Loop 1m GPS Velocity raw",_gps_freq = 50, _init_x=-992.412, _init_y=362.021)


    '''
    Example: Venice
        GPS 10m std. dev.
    ---------------------------------------
    '''

    dsoObject = scene.objects["Camera_Venice_DSO_Scaled"]
    gpsObject = scene.objects["Camera_Venice_GPS_10m"]
    test_kf_position(_title="GPS+DSO",_description="Venice Loop 10m GPS Position Manually scaled", _gps_freq = 1)
    test_kf_position(_title="GPS+DSO",_description="Venice Loop 10m GPS Position Manually scaled", _gps_freq = 50)
    test_kf_position(_title="GPS+DSO",_description="Venice Loop 10m GPS Position Manually scaled", _gps_freq = 50, _init_x=-992.412, _init_y=362.021)
    
    # Position raw:
    dsoObject = scene.objects["Camera_Venice_DSO_Raw"]
    test_kf_position(_title="GPS+DSO",_description="Venice Loop 10m GPS Position raw", _gps_freq = 1)
    test_kf_position(_title="GPS+DSO",_description="Venice Loop 10m GPS Position raw", _gps_freq = 50)
    test_kf_position(_title="GPS+DSO",_description="Venice Loop 10m GPS Position raw", _gps_freq = 50, _init_x=-992.412, _init_y=362.021)
    
    # Position+Velocity raw:
    test_kf_velocity(_title="GPS+DSO",_description="Venice Loop 10m GPS Velocity raw",_gps_freq = 1)
    test_kf_velocity(_title="GPS+DSO",_description="Venice Loop 10m GPS Velocity raw",_gps_freq = 50)
    test_kf_velocity(_title="GPS+DSO",_description="Venice Loop 10m GPS Velocity raw",_gps_freq = 50, _init_x=-992.412, _init_y=362.021)



    '''
    Example: Destructed
        GPS 1m std. dev.
    ---------------------------------------
    '''
    scene.frame_end = 800
    frames = scene.frame_end-scene.frame_start
    
    groundTruthObject = scene.objects["Camera_BlenderDestructed_GT"]
    dsoObject = scene.objects["Camera_BlenderDestructed_WithRotation_DSO_Scaled"]
    gpsObject = scene.objects["Camera_BlenderDestructed_GPS_1m"]
    filteredObject = scene.objects["Camera_BlenderDestructed_KF"]
    
    
    # Only GPS updates
    test_kf_position(_title="GPS",_description="Destructed Loop 1m Only GPS Updates, every frame", _gps_freq = 1)
    test_kf_position(_title="GPS",_description="Destructed Loop 1m Only GPS Updates, every 50th frame", _gps_freq = 50)
    test_kf_position(_title="GPS",_description="Destructed Loop 1m Only GPS Updates, every 50th frame, initialized", _gps_freq = 50, _init_x=4.84254, _init_y=-72.6998)
    
    # Only DSO updates
    test_kf_position(_title="DSO",_description="Destructed Loop 1m Only DSO updates")
    
    # Position scaled
    test_kf_position(_title="GPS+DSO",_description="Destructed Loop 1m GPS Position Manually scaled", _gps_freq = 1)
    test_kf_position(_title="GPS+DSO",_description="Destructed Loop 1m GPS Position Manually scaled", _gps_freq = 50)
    test_kf_position(_title="GPS+DSO",_description="Destructed Loop 1m GPS Position Manually scaled", _gps_freq = 50, _init_x=4.84254, _init_y=-72.6998)
    
    # Position raw:
    dsoObject = scene.objects["Camera_BlenderDestructed_WithRotation_DSO_Raw"]
    test_kf_position(_title="GPS+DSO",_description="Destructed Loop 1m GPS Position raw", _gps_freq = 1)
    test_kf_position(_title="GPS+DSO",_description="Destructed Loop 1m GPS Position raw", _gps_freq = 50)
    test_kf_position(_title="GPS+DSO",_description="Destructed Loop 1m GPS Position raw", _gps_freq = 50, _init_x=4.84254, _init_y=-72.6998)
    
    # Position+Velocity raw:
    test_kf_velocity(_title="GPS+DSO",_description="Destructed Loop 1m GPS Velocity raw",_gps_freq = 1)
    test_kf_velocity(_title="GPS+DSO",_description="Destructed Loop 1m GPS Velocity raw",_gps_freq = 50)
    test_kf_velocity(_title="GPS+DSO",_description="Destructed Loop 1m GPS Velocity raw",_gps_freq = 50, _init_x=4.84254, _init_y=-72.6998)



    '''
    Example: Destructed
        GPS 10m std. dev.
    ---------------------------------------
    '''

    dsoObject = scene.objects["Camera_BlenderDestructed_WithRotation_DSO_Scaled"]
    gpsObject = scene.objects["Camera_BlenderDestructed_GPS_10m"]
    test_kf_position(_title="GPS+DSO",_description="Destructed Loop 10m GPS Position Manually scaled", _gps_freq = 1)
    test_kf_position(_title="GPS+DSO",_description="Destructed Loop 10m GPS Position Manually scaled", _gps_freq = 50)
    test_kf_position(_title="GPS+DSO",_description="Destructed Loop 10m GPS Position Manually scaled", _gps_freq = 50, _init_x=4.84254, _init_y=-72.6998)
    
    # Position raw:
    dsoObject = scene.objects["Camera_BlenderDestructed_WithRotation_DSO_Raw"]
    test_kf_position(_title="GPS+DSO",_description="Destructed Loop 10m GPS Position raw", _gps_freq = 1)
    test_kf_position(_title="GPS+DSO",_description="Destructed Loop 10m GPS Position raw", _gps_freq = 50)
    test_kf_position(_title="GPS+DSO",_description="Destructed Loop 10m GPS Position raw", _gps_freq = 50, _init_x=4.84254, _init_y=-72.6998)
    
    # Position+Velocity raw:
    test_kf_velocity(_title="GPS+DSO",_description="Destructed Loop 10m GPS Velocity raw",_gps_freq = 1)
    test_kf_velocity(_title="GPS+DSO",_description="Destructed Loop 10m GPS Velocity raw",_gps_freq = 50)
    test_kf_velocity(_title="GPS+DSO",_description="Destructed Loop 10m GPS Velocity raw",_gps_freq = 50, _init_x=4.84254, _init_y=-72.6998)



        
