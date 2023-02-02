

import math
import numpy as np

# settings
ROTATION_SLOW_DOWN = 0.2#-rad/s
MOVING_DRAG = 0.02
MAX_SPEED = 15

def closest_point_online(lx1, ly1, lx2, ly2, x0, y0):
    A1 = ly2 - ly1
    B1 = lx1 - lx2
    C1 = (ly2 - ly1)*lx1 + (lx1 - lx2)*ly1
    C2 = -B1*x0 + A1*y0
    det = A1*A1 - -B1*B1
    cx = 0
    cy = 0
    if det != 0:
        cx = (A1*C1 - B1*C2)/det
        cy = (A1*C2 - -B1*C1)/det
    else:
        cx = x0
        cy = y0
    return [cx, cy]



class PhysicsObject():
    def __init__(self, x=0, y=0, vx=0, vy=0, mass=1, angular_velocity=0) -> None:
        self.pos = np.array([x, y])
        self.vel = np.array([vx, vy], dtype=np.float64)
        self.forces = []
        self.mass:float = mass
        self.rot:float = 0
        self.type = "None"
        self.angular_velocity:float = angular_velocity
        
    @property
    def x(self):
        return self.pos[0]
    
    @x.setter
    def x(self, value):
        self.pos[0] = value
        
    @property
    def y(self):
        return self.pos[1]
    
    @y.setter
    def y(self, value):
        self.pos[1] = value
        
    @property
    def vx(self):
        return self.vel[0]
    
    @vx.setter
    def vx(self, value):
        self.vel[0] = value
        
    @property
    def vy(self):
        return self.vel[1]
    
    @vy.setter
    def vy(self, value):
        self.vel[1] = value
        
    def apply_force(self, force: list[float, int], time_step_size = 1):
        self.vel[0] += force[0] / self.mass * time_step_size
        self.vel[1] += force[1] / self.mass * time_step_size
    
    def apply_forces(self, time_step_size):
        for force in self.forces:
            self.apply_force(force, time_step_size)
    
    def apply_velocity(self, time_step_size):
        self.pos[0] += self.vel[0] * time_step_size
        self.pos[1] += self.vel[1] * time_step_size
    
    def change_angular_vel(self, force: list[float, int], offset : list[float, int]):
        collision_pos_slope = offset[1] / offset[0]
        perpendicular_pos_slope = math.atan(collision_pos_slope) + math.pi / 2
        
        force_slope = force[0] / force[1]
        radian_diff = perpendicular_pos_slope - force_slope
        
        effective_force = math.cos(radian_diff)

        
class Circle(PhysicsObject):
    def __init__(self, x=0, y=0, vx=0, vy=0, radius=1, mass=1, angular_velocity=0) -> None:
        super().__init__(x, y, vx, vy, mass, angular_velocity)
        self.radius = radius
        self.type = "circle"
    
    def update(self, time_step_size:float=1):
        self.rot = self.rot * ROTATION_SLOW_DOWN * time_step_size
    
    def detect_collision(self, obj= None):
        if obj.type == "None":
            pass
    
    def detect_line_collision(self, lx1, ly1, lx2, ly2):
        closest_point = closest_point_online(lx1, ly1, lx2, ly2, self.x, self.y)
        if (closest_point[0] - self.x) ** 2 + (closest_point[1] - self.y) ** 2 < self.radius:
            return True
        else:
            return False
    
    def overlaps(self, circle):
        return np.hypot(*(self.pos - circle.pos)) < self.radius + circle.radius

    def detect_border_collision(self, sim_width, sim_height):
        if  self.pos[0] - self.radius < 0 or self.pos[0] + self.radius > sim_width or self.pos[1] - self.radius < sim_height or self.pos[1] + self.radius > sim_height:
            return True
        else:
            return False
            
class PhysicsEnvironment():
    def __init__(self, sizex, sizey, objects=[]) -> None:
        self.size : list = [sizex, sizey]
        self.objects :list[PhysicsObject, Circle] = objects
    
    def run_tick(self, time_step_size=1):
        
        for obj in self.objects:
            obj.forces = []
            obj.forces.append([0, -1])
            
            
        circle_list = enumerate(list(filter(lambda x: x.type == "circle", self.objects)))
        detection_dict = {}
        for index1, circle in circle_list:
            detection_dict[circle] = []
            for index2, circle2 in circle_list:
                if index2 > index1:
                    detection_dict[circle].append(circle2)

        for circle1 in list(detection_dict.keys()):
            for circle2 in detection_dict[circle1]:
                if circle1.overlaps(circle2):
                    self.circle_collision(circle1, circle2, time_step_size)

        for obj in self.objects:
            obj.apply_forces(time_step_size)
            speed = math.sqrt(obj.vx ** 2 + obj.vy ** 2)
            obj.vel *= 1 - MOVING_DRAG * time_step_size * (1 / ( -speed + MAX_SPEED))
            
        
        for obj in self.objects:
            pos_change = obj.vel * time_step_size
            obj.pos = obj.pos + pos_change
            
        for obj in self.objects:
            if (obj.detect_line_collision(0, 0, self.size[0], 0 ) or
                obj.detect_line_collision(0, self.size[1], self.size[0], self.size[1])):
                obj.vy *= -1
            if (obj.detect_line_collision(0, 0, 0, self.size[1]) or 
                obj.detect_line_collision(0, self.size[1], self.size[0], self.size[1])):
                obj.vx *= -1
        
            






    
    
    def circle_collision(self, circle1: Circle, circle2: Circle, time_step_size=1):
        m1, m2 = circle1.mass, circle2.mass
        M = m1 + m2
        r1, r2 = circle1.pos, circle2.pos
        d = np.linalg.norm(r1 - r2)**2
        v1, v2 = circle1.vel, circle2.vel
        u1 = v1 - 2*m2 / M * np.dot(v1-v2, r1-r2) / d * (r1 - r2)
        u2 = v2 - 2*m1 / M * np.dot(v2-v1, r2-r1) / d * (r2 - r1)
        circle1.forces.append(u1)
        circle2.forces.append(u2)
            
        
        
        
        
    
    
        