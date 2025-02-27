class PID:
    def __init__(self, kp, ki, kd, i_limit, cycle_time_seconds):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = i_limit
        self.cycle_time_seconds = cycle_time_seconds
        self.last_error = 0
        self.last_integral = 0
        
    def calculate(self, error):
        p:float = error * self.kp
        i:float =  self.last_integral + (error * self.cycle_time_seconds * self.ki)
        i = max(min(i, self.i_limit), -self.i_limit) # constraint withing I-term limit
        d:float = self.kd * (error - self.last_error) / self.cycle_time_seconds
        
        self.last_integral = error
        self.last_integral = i
        
        return p + i + d