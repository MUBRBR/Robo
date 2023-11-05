class Radar(): #Will eventually house maped objects
    def __init__(self, robot):
        self.front_distance = 0.0 # until first update, everything is assumed to be really far away
        self.back_distance = 0.0
        self.weighted_distance = 999.0 # until first update, everything is assumed to be really far away
        self.obstacle_angle = 0 
        self.arlo = robot
        self.active = False

    def Angle(self):
        return self.obstacle_angle
    
    def DistF(self):
        return self.front_distance 
    
    def DistW(self):
        return self.weighted_distance
    
    def backCameraSafe(self):
        return True

    def Update(self):
        
        # If self.active is false, not fields are updated, it keeps beleiving everything is far away
        # if (not self.active):
        #     pass

        left_distance = self.arlo.read_left_ping_sensor() / 10
 
        front_distance = self.arlo.read_front_ping_sensor() / 10
        
        right_distance = self.arlo.read_right_ping_sensor() / 10

        back_distance = self.arlo.read_back_ping_sensor() / 10


        self.weighted_distance = min((left_distance)*2,front_distance,(right_distance)*2, back_distance) # We need to know if something is close, we use the closest regardless of direction. We dont care as much about the side sensors

        self.obstacle_angle = -1 if ((-left_distance + right_distance) < 0) else 1 # if positive is obstacle is to the left (ie. turn right)

        self.front_distance = front_distance