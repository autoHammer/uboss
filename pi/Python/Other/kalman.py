import numpy as np
import time


class Kalman:
    def __init__(self, x, p, H, R, process_variance):
        self.x = x
        self.p = p
        self.t_old = time.time()
        self.H = H.copy()
        self.Ht = np.transpose(H)
        self.z = np.zeros((H.shape[0], 1))
        self.R = R.copy()
        self.process_variance = process_variance
        self.dt = self.A = self.Q = self.G = self.Gt = self.K = None

    def a_priori(self):
        self.build_a()
        self.build_q()
        self.x = self.A * self.x
        self.p = self.A * self.p * np.transpose(self.A) + self.Q
        #print(self.x, self.p)

    def a_posteriori(self, measurement):
        self.K = self.p * self.Ht * np.linalg.inv(self.H * self.p * self.Ht + self.R)
        self.build_z(measurement)
        self.x = self.x + self.K * (self.z - self.H * self.x)
        size = np.shape(self.p)
        self.p = (np.identity(size[0]) - self.K * self.H) * self.p

    def a_posteriori_asynchronous(self, measurement, sensor):
        self.K = self.p * self.Ht[:, sensor-1] * np.linalg.inv(self.H[sensor-1, :] * self.p * self.Ht[:, sensor-1] + self.R[sensor-1, sensor-1])
        self.build_z(measurement)
        self.x = self.x + self.K * (self.z[sensor-1] - self.H[sensor-1, :] * self.x)
        size = np.shape(self.p)
        self.p = (np.identity(size[0]) - self.K * self.H[sensor-1, :]) * self.p

    def build_a(self):
        self.dt = time.time() - self.t_old
        self.t_old = time.time()
        self.A = np.matrix([[1.0, self.dt, 0.5 * self.dt ** 2],
                            [0.0, 1.0, self.dt],
                            [0.0, 0.0, 1.0]])

    def build_q(self):
        self.G = np.matrix([[self.dt**3 / 6],
                            [self.dt**2 / 2],
                            [self.dt]])
        self.Gt = np.transpose(self.G)
        self.Q = np.matrix(self.G * self.Gt * self.process_variance)

    def build_z(self, measurements):
        row = 0
        for measurement in measurements:
            self.z[row] = measurement
            row += 1
