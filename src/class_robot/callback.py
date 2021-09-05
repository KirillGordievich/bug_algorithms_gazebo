
def callback_pose(self, data):
    self.x = data.x
    self.y = data.y 
    self.yaw = data.theta
    # if the status is initialization
    # save the initial position
    if self.status == 'initialization':
        self.initial_position = [data.x, data.y]

def callback_laser(self, data):
    # split laser data into 5 regions
    # 'right', 'fright', 'front', 'fleft' and 'left'
    self.regions = {
        'right':  min(min(data.ranges[0:143]), 10),
        'fright': min(min(data.ranges[144:287]), 10),
        'front':  min(min(data.ranges[288:431]), 10),
        'fleft':  min(min(data.ranges[432:575]), 10),
        'left':   min(min(data.ranges[576:719]), 10),
        }
