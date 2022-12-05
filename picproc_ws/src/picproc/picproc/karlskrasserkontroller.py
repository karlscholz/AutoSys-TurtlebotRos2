from time import sleep



class Class:
  P_Gain = 1.0 / 10.0
  I_Gain = 1.0 / 10.0
  D_Gain = 1.0 / 10.0

  I_Limit = 10
  Max_Control = .5

  last_error = 0.0
  integral = 0.0
  
  
  def __init__(self):
    print("initiated")
  
  def constrain(self, val, min_val, max_val):
    return min(max_val, max(min_val, val))

  def method(self, measurement, setpoint):
    error = setpoint - measurement
    self.integral = self.constrain(self.integral + error, -self.I_Limit, self.I_Limit)
    derivative = error - self.last_error
    output = self.constrain(self.P_Gain * error + self.I_Gain * self.integral + self.D_Gain * derivative, -self.Max_Control, self.Max_Control)
    self.last_error = error

    
    print("output", output, "error", error, "integral", self.integral, "derivative", derivative)
    return output

def main(args=None):
 
  Object = Class()
  measurement = 10
  setpoint = 0

  while(True):
    measurement = measurement + Object.method(measurement, 0)
    print("Measurement", measurement)
    sleep(.1)
    
  
if __name__ == '__main__':
  main()