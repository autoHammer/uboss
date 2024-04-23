import time

class PID:
    def __init__(self, kp, ki, kd, output_min=None, output_max=None):
        self.kp_ = kp
        self.ki_ = ki
        self.kd_ = kd
        self.i_lim_ = 100
        self.i_ = 0
        self.old_error_ = 0
        self.t_old = time.time()
        self.min_output_ = output_min
        self.max_output_ = output_max
        self.error_ = self.dt = self.sv_ = self.p_ = self.d_ = None

    def new_sv(self, sv):
        self.sv_ = sv

    def new_kp(self, kp):
        self.kp_ = kp

    def new_ki(self, ki):
        self.ki_ = ki

    def new_kd(self, kd):
        self.kd_ = kd

    def output(self, current_pv):
        self.dt = time.time() - self.t_old
        self.t_old = time.time()
        self.error_ = self.sv_ - current_pv
        self.p_calc()
        self.i_calc()
        self.d_calc()
        output = self.p_ + self.i_ + self.d_

        # Constrain if desired
        if self.min_output_ is not None:
            if output < self.min_output_:
                output = self.min_output_
        if self.max_output_ is not None:
            if output > self.max_output_:
                output = self.max_output_

        return output

    def p_calc(self):
        self.p_ = self.error_ * self.kp_

    def i_calc(self):
        if abs(self.i_ + self.error_ * self.ki_) < self.i_lim_:
            self.i_ = self.i_ + self.error_ * self.ki_

    def d_calc(self):
        self.d_ = (self.error_ - self.old_error_) / self.dt * self.kd_
        self.old_error_ = self.error_

