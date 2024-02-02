import pygame
import math
import numpy as np

def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1-point2)

class Robot:
    def __init__(self, startpos, width):
        self.m2p = 3779.52 # To convert from meters to pixels so that it can be fit and viewed on screen
        # dimensions of robot
        self.w = width
        self.x = startpos[0]
        self.y = startpos[1]
        
        # vl and vr are the velocities of left wheel and right wheel
        self.heading = 0
        self.vl = 0.01*self.m2p
        self.vr = 0.01*self.m2p
        # setting maximum and minimum speeds
        self.maxspeed = 0.02*self.m2p
        self.minspeed = 0.01*self.m2p
        
        self.min_obj_dis = 100
        self.count_down = 5
       
        
        
    def avoid_obstacles(self,point_cloud,dt):
        closest_obj = None
        dist = np.inf#distance of obstacle from robot is set to infinite
        
        if len(point_cloud) > 1:
            for point in point_cloud:
                if dist > distance([self.x,self.y], point):
                    dist = distance([self.x,self.y], point)
                    closest_obj = (point,dist) # storing closest obstacle coordinates and distance in a tuple
            
            if closest_obj[1] < self.min_obj_dis and self.count_down > 0:
                self.count_down -= dt
                self.move_backward()
                
                # If currently moving backward
            else:
                # Reset countdown
                    self.count_down = 5
                # move forward
                    self.move_forward()

    def move_backward(self):
        self.vr = -self.minspeed
        self.vl = -self.minspeed/2
    
    def move_forward(self):
        self.vr = self.minspeed
        self.vl = self.minspeed
        
    def kinematics(self, dt):
        self.x += ((self.vl+self.vr)/2) * math.cos(self.heading) * dt
        self.y -= ((self.vl+self.vr) / 2) * math.sin(self.heading) * dt
        self.heading += (self.vr - self.vl) / self.w * dt
        
        # Now change the self heading value to zero
        if self.heading>2*math.pi or self.heading<-2*math.pi:
            self.heading = 0 

        self.vr = max(min(self.maxspeed, self.vr), self.minspeed)
        self.vl = max(min(self.maxspeed, self.vl), self.minspeed)
        
class Graphics:
    def __init__(self, dimensions, robot_image_path, map_image_path): 
        pygame.init()
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.yel = (255, 255, 0)
        
        # Load images
        self.robot = pygame.image.load(robot_image_path)
        self.map_img = pygame.image.load(map_image_path)
        
        # dimensions
        self.height, self.width = dimensions

        # Window settings
        pygame.display.set_caption("Obstacle Avoidance")
        self.map = pygame.display.set_mode((self.width, self.height))
        self.map.blit(self.map_img, (0, 0))

    def draw_robot(self, x, y, heading):
        rotated = pygame.transform.rotozoom(self.robot, math.degrees(heading), 1)
        rect = rotated.get_rect(center=(x, y))
        self.map.blit(rotated, rect)
    
    def draw_sensor_data(self, point_cloud):
        for point in point_cloud:
            pygame.draw.circle(self.map, self.red, point, 3, 0)

class Ultrasonic:
    def __init__(self, sensor_range, map):
        self.sensor_range = sensor_range
        self.map_width, self.map_height= pygame.display.get_surface().get_size()
        self.map = map
        
    def sense_obstacles(self, x, y, heading):
        obstacles = []
        x1, y1 = x, y 
        
        start_angle = heading - self.sensor_range[1]
        finish_angle = heading + self.sensor_range[1]
        
        for angle in np.linspace(start_angle, finish_angle, 10, False):
            # Adjust angle based on robot heading
            
           
            x2 = x1 + self.sensor_range[0] * math.cos(angle)
            y2 = y1 - self.sensor_range[0] * math.sin(angle)
            
            for i in range(0, 100):
                u = i / 100
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                
                if 0 < x < self.map_width and 0 < y < self.map_height:
                    color = self.map.get_at((x, y))
                    self.map.set_at((x, y), (0, 208, 255))
                    
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        obstacles.append([x, y])
                        break
        return obstacles

# Main code
MAP_DIMENSIONS = (600, 1200)

gfx = Graphics(MAP_DIMENSIONS, 'robot.png', 'obstacle_2.png')

start = (200, 200)
robot = Robot(start, 0.01 * 3779.52)

sensor_range = (250, math.radians(40))
ultra_sonic = Ultrasonic(sensor_range, gfx.map)

dt = 0
last_time = pygame.time.get_ticks()
running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    dt = (pygame.time.get_ticks()-last_time)/1000
    last_time = pygame.time.get_ticks()
    
    gfx.map.blit(gfx.map_img,(0,0))
    
    robot.kinematics(dt)
    gfx.draw_robot(robot.x, robot.y, robot.heading)
    point_cloud = ultra_sonic.sense_obstacles(robot.x, robot.y, robot.heading)
    robot.avoid_obstacles(point_cloud, dt)
    gfx.draw_sensor_data(point_cloud)
    
    pygame.display.update()
