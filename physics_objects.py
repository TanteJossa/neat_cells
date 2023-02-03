

import math
import numpy as np

# settings
ROTATION_SLOW_DOWN = 0.2#-rad/s
MOVING_DRAG = 0.02
MAX_SPEED = 15

def closest_point_on_line(lx1, ly1, lx2, ly2, x0, y0):
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

def line_intersection(p0, p1, p2, p3):
    p0_x, p0_y = p0 if type(p0) == tuple else p0[0], p0[1]
    p1_x, p1_y = p1 if type(p1) == tuple else p1[0], p1[1]
    p2_x, p2_y = p2 if type(p2) == tuple else p2[0], p2[1]
    p3_x, p3_y = p3 if type(p3) == tuple else p3[0], p3[1]
    s1_x = p1_x - p0_x
    s1_y = p1_y - p0_y
    s2_x = p3_x - p2_x
    s2_y = p3_y - p2_y
    d = (-s2_x * s1_y + s1_x * s2_y)
    if d != 0:
        s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / d
        t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / d
        if 0 <= s <= 1 and 0 <= t <= 1:
            intX = p0_x + (t * s1_x)
            intY = p0_y + (t * s1_y)
            return (intX, intY)
    return None
    

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
        closest_point = closest_point_on_line(lx1, ly1, lx2, ly2, self.x, self.y)
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

class Line():
    def __init__(self, p1=[0,0], p2=[0,0],) -> None:
        self.p1 = p1
        self.p2 = p2
        self.type = "line"
        
        
class PhysicsEnvironment():
    def __init__(self, sizex, sizey, objects=[], lines=[]) -> None:
        self.size : list = [sizex, sizey]
        self.objects :list[PhysicsObject, Circle, Line] = objects
        self.lines : list[Line] = lines
    
    @property.physics_objects
    def physics_objects(self):
        return filter(lambda x: x.type in ["circle"], self.objects)
    
    def run_tick(self, time_step_size=1):
        
        # filter all the times that can not be in range of the obj
        collisions_test_dict = {}
        for obj in self.objects:
            collisions_test_dict[obj] : dict[PhysicsObject: list[Line, Circle]] = []
            for obj2 in self.objects:
                if (math.abs(obj.x - obj2.x) < MAX_SPEED or
                    math.abs(obj.y - obj2.y) < MAX_SPEED):
                    collisions_test_dict[obj].append(obj2)                
            
            for line in self.lines:
                distance_to_line = closest_point_on_line(line.p1[0], line.p1[1], line.p2[0], line.p2[1], obj.x, obj.y)
                if (math.abs(obj.x - distance_to_line[0]) < MAX_SPEED or
                    math.abs(obj.y - distance_to_line[1]) < MAX_SPEED):
                    collisions_test_dict[obj].append(line)

            
        # check if the items collided
        collided_list = {}
        for obj in list(collisions_test_dict.keys()):
            for obj2 in collisions_test_dict[obj]:
                if obj2.type == "line":
                    pass                    
                if obj2.type == "circle":
                    pass
        
        
        
        
        
        
        
        
        
        
        
        
        for obj in self.objects:
            obj.apply_forces()
    
        
        
        
        
        
        
        
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
    
    def closest_point_on_line(self, p1: list[int, float], p2: list[int, float], x0, y0):
        lx1 = p1[0]
        ly1 = p1[1]
        lx2 = p2[0]
        ly2 = p2[1]
        
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
    
    # segments
    def line_segment_intersection(self, p0, p1, p2, p3):
        p0_x, p0_y = p0 if type(p0) == tuple else p0[0], p0[1]
        p1_x, p1_y = p1 if type(p1) == tuple else p1[0], p1[1]
        p2_x, p2_y = p2 if type(p2) == tuple else p2[0], p2[1]
        p3_x, p3_y = p3 if type(p3) == tuple else p3[0], p3[1]
        s1_x = p1_x - p0_x
        s1_y = p1_y - p0_y
        s2_x = p3_x - p2_x
        s2_y = p3_y - p2_y
        d = (-s2_x * s1_y + s1_x * s2_y)
        if d != 0:
            s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / d
            t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / d
            if 0 <= s <= 1 and 0 <= t <= 1:
                intX = p0_x + (t * s1_x)
                intY = p0_y + (t * s1_y)
                return [intX, intY]
        return False
    
    # two infinite lines intersection point
    def intersection_point(p1, p2, p3, p4):
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4
        
        # Calculate the denominator
        denominator = ((y4 - y3) * (x2 - x1)) - ((x4 - x3) * (y2 - y1))

        # Check if the lines are parallel
        if denominator == 0:
            return None

        # Calculate the numerator for the first line
        numerator_a = ((x4 - x3) * (y1 - y3)) - ((y4 - y3) * (x1 - x3))

        # Calculate the numerator for the second line
        numerator_b = ((x2 - x1) * (y1 - y3)) - ((y2 - y1) * (x1 - x3))

        # Calculate the intersection point
        ua = numerator_a / denominator
        ub = numerator_b / denominator
        x = x1 + (ua * (x2 - x1))
        y = y1 + (ua * (y2 - y1))

        return [x, y]

    def distance(self, p1, p2):
        return math.sqrt((p2[0] - p1[0])**2 + (p2[1]**2 - p1[1]**2))

    def point_on_line(self, line: Line, p):        
        return self.distance(line.p1, p) + self.distance(line.p2, p) == self.distance(line.p1, line.p2);
    
    def check_line_circle_intersection(self, circle : Circle, line: Line):
        closest_point = self.closest_point_on_line(line.p1, line.p2, circle.pos[0], circle.pos[1])
        if self.distance(closest_point,  circle.pos) < circle.radius:
            return True
        else:
            return False
    
    def point_in_rectangle(p1, p2, p3, p4, point):
        A = np.array(p1)
        B = np.array(p2)
        C = np.array(p3)
        M = np.array(p4)
        return 0 <= np.dot(A*B,A*M) <= np.dot(A*B,A*B) and 0 <= np.dot(B*C,B*M) <= np.dot(B*C,B*C)
    
    def check_circle_line_collision(self, circle : Circle, line: Line, time_step_size=1):
        #  3 area's to check if the ball along the vector line crosses
        vector_end_pos = list((circle.pos + circle.vel) * time_step_size)
        # ends at the "next" position
        movement_vector = Line(list(circle.pos), vector_end_pos)
        
        raw_offset = [vector_end_pos[0] - circle.x, vector_end_pos[1] - circle.y]
        
        # the offset if flipped 90 degrees
        offset = np.arrat([circle.radius * (raw_offset[1] / self.distance(vector_end_pos, circle.pos)), circle.radius * (raw_offset[0] / self.distance(vector_end_pos, circle.pos))])
        
        # a box that covers the entire path the circle will follow this tick
        line1 = Line(circle.pos - offset, circle.pos + offset)
        line2 = Line(circle.pos + offset, vector_end_pos + offset)
        line3 = Line(vector_end_pos + offset, vector_end_pos - offset)
        line4 = Line(vector_end_pos - offset, circle.pos - offset)
        
        is_touching_line1 = type(self.line_segment_intersection(line1.p1, line1.p2, line.p1, line.p2)) == list
        is_touching_line2 = type(self.line_segment_intersection(line2.p1, line2.p2, line.p1, line.p2)) == list
        is_touching_line3 = type(self.line_segment_intersection(line3.p1, line3.p2, line.p1, line.p2)) == list
        is_touching_line4 = type(self.line_segment_intersection(line4.p1, line4.p2, line.p1, line.p2)) == list
        
        is_touching_current_circle = self.check_line_circle_intersection(circle, line)
        is_touching_final_circle = self.check_line_circle_intersection(vector_end_pos, line)
        
        # check if the line is in the box
        p1_in_box = line.p1[0] < 
        
        if (
            is_touching_line1 or 
            is_touching_line2 or
            is_touching_line3 or
            is_touching_line4 or
            is_touching_current_circle or
            is_touching_final_circle):
            return True
        else:
            return False        
        
    
        
        
    
    def circle_line_collision(self, circle : Circle, line: Line, time_step_size=1):
        # https://ericleong.me/research/circle-line/
        # Moving Circle and Static Line Segment
        
        vector_end_pos = list((circle.pos + circle.vel) * time_step_size)
        # ends at the "next" position
        movement_vector = Line(list(circle.pos), vector_end_pos)
        
        # point between vector and line line (so infinitely long lines)
        intersection_point = self.intersection_point(line.p1, line.p2, movement_vector.p1, movement_vector.p2)
        # closest point on line to the end of the vector
        closest_to_vector_end = self.closest_point_on_line(line.p1, line.p2, vector_end_pos[0], vector_end_pos[1])
        # closest point on vector to start
        closest_point_from_line_start  = self.closest_point_on_line(movement_vector.p1, movement_vector.p2, line.p1[0], line.p1[1])
        # closest point on vector to end 
        closest_point_from_line_end  = self.closest_point_on_line(movement_vector.p1, movement_vector.p2, line.p2[0], line.p2[1])
        
        
        # if the intersection point is on both segments (it can be on a point that falls out of the line)
        is_intersection_between_segments = self.point_on_line(movement_vector, intersection_point) and self.point_on_line(line, intersection_point)
        
        # if the circle touches the line at the movement endpoint
        touches_line_at_next_tick = self.distance(vector_end_pos, closest_to_vector_end) < circle.radius and self.point_on_line(line, closest_to_vector_end)
        
        # if the closest point to the start of the line is closer than the radius
        touches_at_start = self.distance(closest_point_from_line_start, line.p1) < circle.radius # and self.point_on_line(movement_vector, closest_point_from_line_start)
        # if the closest point to the end of the line is closer than the radius
        touches_at_end = self.distance(closest_point_from_line_end, line.p2) < circle.radius # and self.point_on_line(movement_vector, closest_point_from_line_end)
        
        # if one of these cases is true
        if (is_intersection_between_segments or
            touches_line_at_next_tick or
            touches_at_start or
            touches_at_end):
            
            # point of collision = 
            # the intersection between the movement vector and the line
            # -
            # the length from the circle_center to the closest point on the line 
            # /
            # the length from the intersection to the circle center
            # times the movement vector of the circle divided by its length (so you get the x and y ratio)
            closest_to_vector_start = self.closest_point_on_line(line.p1, line.p2, circle.pos[0], circle.pos[1])

            
            point_of_collision = (
                list(
                    np.array(
                        intersection_point - (self.distance(intersection_point, circle.pos) 
                                              / 
                                              self.distance(closest_to_vector_start, circle.pos)) 
                                            * circle.radius 
                                            * (np.array(movement_vector) 
                                               / 
                                               self.distance(movement_vector[0], movement_vector[1]))
                    )
                )   
            )
        

            
        
        # c is less than r away from (x1, y1) and on the movement vector.
        if closest_to_vector_end[0] ** 2 + closest_to_vector_end[1] ** 2 < circle.radius ** 2:
            return False        
        
        
        

    def detect_circle_circle_collision(circle, circle2):
        pass    
    
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
            
        
        
        
        
    
    
        