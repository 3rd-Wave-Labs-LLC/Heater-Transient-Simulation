import time

class PID:

    AUTOMATIC = 1
    MANUAL = 0
    DIRECT = 0
    REVERSE = 1
    P_ON_M = 0
    P_ON_E = 1
    START = time.time()

    def millis(self):
        self.time_counter_ms = self.time_counter_ms + self.SampleTime
        return self.time_counter_ms

    '''
    Constructor(...)***************************************************************
        The parameters specified here are those for for which we can't set up
        reliable defaults, so we need to have the user set them.
    '''
    def __init__(self, Input:float, Output:float, Setpoint:float, Kp:float, Ki:float, Kd:float, controller_direction:int):
        
        self.time_counter_ms:int = 0
        self.lastTime:int
        self.outputSum:float

        self.Input = Input
        self.lastInput = Input
        self.Output = Output
        self.Setpoint = Setpoint

        self.outMin:float
        self.outMax:float
        self.inAuto = False
        self.SetOutputLimits(0, 255)  # default output limit corresponds to the arduino pwm limits

        self.kp:float = 1
        self.ki:float = 0
        self.kd:float = 0
        self.pOn = self.P_ON_E
        self.controllerDirection:int
        self.SetControllerDirection(controller_direction)

        self.dispKp:float
        self.dispKi:float
        self.dispKd:float

        self.SampleTime = 1  # default Controller Sample Time is 0.1 seconds
        self.SetTunings(Kp, Ki, Kd, self.P_ON_E)

        self.lastTime = self.millis() - self.SampleTime

    '''
    Compute() **********************************************************************
        This, as they say, is where the magic happens.  this function should be called
    every time "void loop()" executes.  the function will decide for itself whether a new
    pid Output needs to be computed.  returns true when the output is computed,
    false when nothing has been done.
    '''
    def Compute(self):
        if not self.inAuto:
            return False
        now:int = self.millis()
        timeChange:int = now - self.lastTime
        if timeChange >= self.SampleTime:
            # Compute all the working error variables #
            input:float = self.Input
            error:float = self.Setpoint - input
            dInput:float = input - self.lastInput
            self.outputSum = self.outputSum + (self.ki * error)

            # Add Proportional on Measurement, if P_ON_M is specified #
            if not self.pOnE:
                self.outputSum = self.outputSum - (self.kp * dInput)

            if self.outputSum > self.outMax:
                self.outputSum = self.outMax
            elif self.outputSum < self.outMin:
                self.outputSum = self.outMin

            # Add Proportional on Error, if P_ON_E is specified #
            output:float
            if self.pOnE:
                output = self.kp * error
            else:
                output = 0
            
            # Compute Rest of PID Output #
            output = output + (self.outputSum - (self.kd * dInput))

            if output > self.outMax:
                output = self.outMax
            elif output < self.outMin:
                output = self.outMin
            self.Output = output

            # Remember some variables for next time #
            self.lastInput = input
            self.lastTime = now
            return True
        return False
    '''
    SetTunings(...)************************************************************
        This function allows the controller's dynamic performance to be adjusted.
        it's called automatically from the constructor, but tunings can also
        be adjusted on the fly during normal operation
    '''
    def SetTunings(self, Kp:float, Ki:float, Kd:float, POn:int = 1):
        if (Kp < 0) or (Ki < 0) or (Kd < 0):
            return
        
        self.pOn = POn
        self.pOnE = (POn == self.P_ON_E)

        self.dispKp = Kp
        self.dispKi = Ki
        self.dispKd = Kd

        SampleTimeInSec:float = self.SampleTime / 1000
        self.kp = Kp
        self.ki = Ki * SampleTimeInSec
        self.kd = Kd / SampleTimeInSec

        if self.controllerDirection == self.REVERSE:
            self.kp = (0 - self.kp)
            self.ki = (0 - self.ki)
            self.kd = (0 - self.kd)

    '''
    SetSampleTime(...) *********************************************************
       sets the period, in Milliseconds, at which the calculation is performed
    '''
    def SetSampleTime(self, NewSampleTime:int):
        if NewSampleTime > 0:
            ratio = NewSampleTime / self.SampleTime
            self.ki *= ratio
            self.kd /= ratio
            self.SampleTime = NewSampleTime

    '''
    SetOutputLimits(...)****************************************************
        This function will be used far more often than SetInputLimits.  while
    the input to the controller will generally be in the 0-1023 range (which is
    the default already,)  the output will be a little different.  maybe they'll
    be doing a time window and will need 0-8000 or something.  or maybe they'll
    want to clamp it from 0-125.  who knows.  at any rate, that can all be done
    here.
    '''
    def SetOutputLimits(self, Min:float, Max:float):
        if Min >= Max:
            return
        self.outMin = Min
        self.outMax = Max
        if self.inAuto:
            if self.Output > self.outMax:
                self.Output = self.outMax
            elif self.Output < self.outMin:
                self.Output = self.outMin
            if self.outputSum > self.outMax:
                self.outputSum = self.outMax
            elif self.outputSum < self.outMin:
                self.outputSum = self.outMin

        '''
    SetMode(...)****************************************************************
       Allows the controller Mode to be set to manual (0) or Automatic (non-0)
    when the transition from manual to auto occurs, the controller is
    automatically initialized
    '''
    def SetMode(self, mode:int):
        newAuto:bool = (mode == self.AUTOMATIC)
        if newAuto and not self.inAuto:
            self.Initialize()
        self.inAuto = newAuto 

    '''
    SetSampleTime(...) *********************************************************
       sets the period, in Milliseconds, at which the calculation is performed
    '''
    def Initialize(self):
        self.outputSum = self.Output
        self.lastInput = self.Input
        if self.outputSum > self.outMax:
            self.outputSum = self.outMax
        elif self.outputSum < self.outMin:
            self.outputSum = self.outMin

    '''
    SetControllerDirection(...)*************************************************
        The PID will either be connected to a DIRECT acting process (+Output leads
        to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
        know which one, because otherwise we'll be increasing the output when we
        should be decreasing.  This is called from the constructor.
        '''
    def SetControllerDirection(self, Direction:int):
        if self.inAuto and Direction != self.controllerDirection:
            self.kp = (0 - self.kp)
            self.ki = (0 - self.ki)
            self.kd = (0 - self.kd)
        self.controllerDirection = Direction

    def GetKp(self):
        return self.dispKp
    
    def GetKi(self):
        return self.dispKi  
    
    def GetKd(self):
        return self.dispKd  
    
    def GetMode(self):
        return self.inAuto
    
    def GetDirection(self):
        return self.controllerDirection