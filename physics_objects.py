

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
    def __init__(self, x=0.0, y=0.0, vx=0.0, vy=0.0, mass=1.0, angular_velocity=0) -> None:
        self.pos = np.array([x, y], dtype=np.float64)
        self.vel = np.array([vx, vy], dtype=np.float64)
        self.forces = []
        self.mass:float = mass
        self.rot:float = 0
        self.type = "None"
        self.angular_velocity:float = angular_velocity
    
    def end_pos(self, time_step_size):
        return self.pos + (self.vel - self.pos) * time_step_size
    
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
        
    def apply_force(self, force: list[float, int],time_step_size=1): 

        self.vel[0] += force[0] / self.mass * time_step_size
        self.vel[1] += force[1] / self.mass * time_step_size
    
    def  apply_forces(self, time_step_size=1):
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
    
    def end_pos(self, time_step_size):
        return self.pos + (self.vel - self.pos) * time_step_size
        
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
        self.lines += [Line([0,0], [sizex, 0]), Line([sizex, 0], [sizex, sizey]), Line([sizex, sizey], [0, sizey]), Line([0, sizey], [0, 0]), ]
    
    @property
    def moving_objects(self):
        return list(filter(lambda x: x.type == 'circle', self.objects))
    
    def run_tick(self, time_step_size=1):
        
        # filter all the times that can not be in range of the obj
        collisions_test_dict : dict[Circle: list[Line | Circle]]  = {}
        for circle in filter(lambda x: x.type == 'circle', self.objects):
            collisions_test_dict[circle] = []
            for obj2 in self.objects:
                if circle != obj2:

                    if (np.abs(circle.x - obj2.x) < MAX_SPEED * time_step_size + circle.radius + obj2.radius or
                        np.abs(circle.y - obj2.y) < MAX_SPEED * time_step_size + circle.radius + obj2.radius):
                        collisions_test_dict[circle].append(obj2)                
            
            for line in self.lines:
                closest_point = closest_point_on_line(line.p1[0], line.p1[1], line.p2[0], line.p2[1], circle.x, circle.y)
                distance_to_point = self.distance(circle.pos, closest_point)
                if distance_to_point < (MAX_SPEED) * time_step_size + circle.radius:
                    collisions_test_dict[circle].append(line)

            
        # check if the items collided (fast)
        collided_list = {}
        for circle in list(collisions_test_dict.keys()):
            collided_list[circle] = []
            for obj2 in collisions_test_dict[circle]:
                if obj2.type == "line":
                    if self.check_circle_line_collision(circle, obj2):
                        collided_list[circle].append(obj2)
                        
                if obj2.type == "circle":
                    if self.check_circle_circle_collision(circle, obj2):
                        collided_list[circle].append(obj2)
        
        
        # work out the collision time for every collision
        collision_time:dict[Circle:Line|Circle] = {}
        for circle in list(collided_list.keys()):
            collision_time[circle] = []
            for obj2 in collided_list[circle]:
                if obj2.type == "line":
                    time, position = self.circle_line_collision_time(circle, obj2, time_step_size)
                    if time:  
                        collision_time[circle].append({'time': time, 'position': position, 'obj': obj2})
                if obj2.type == "circle":
                    time = self.circle_circle_collision_time(circle.pos, circle.end_pos(time_step_size), obj2.pos, obj2.end_pos(time_step_size), circle.radius, obj2.radius, time_step_size)
                    if time != None:
                        collision_time[circle].append({'time': time, 'obj': obj2})
            collision_time[circle] = list(filter(lambda x: not math.isnan(float(x['time'])) and 0 < x['time'] < 1,collision_time[circle]))

        
        # work out the collision response (1. the new end pos 2. the new speed vector))
        for circle in list(collision_time.keys()):
            sorted_cells = sorted(collision_time[circle],key=lambda x: x['time'])

            for collision_data in sorted_cells:
                if collision_data['obj'].type == 'line':

                    new_vel = self.circle_line_collision_response(circle, collision_data['obj'], collision_data['time'], time_step_size)

                    vel_diff = new_vel - circle.vel
                    force_to_get_vel = vel_diff * circle.mass

                    circle.forces.append(list(force_to_get_vel))
                    
                    
                if collision_data['obj'].type == 'circle':
                    circle2 = collision_data['obj']
                    circle1_final_vel, circle2_final_vel = self.circle_circle_collision_response(circle, circle2, collision_data['time'])

                    c1_current_vel = circle.vel - circle.pos
                    c2_current_vel = circle2.vel - circle2.pos
                    
                    c1_vel_diff = circle1_final_vel - c1_current_vel
                    c2_vel_diff = circle2_final_vel - c2_current_vel
                    
                    c1_force_to_get_vel = c1_vel_diff * circle.mass
                    c2_force_to_get_vel = c2_vel_diff * circle2.mass
                                            
                    circle.forces.append(list(c1_force_to_get_vel))
                    circle2.forces.append(list(c2_force_to_get_vel))
            
            
        for obj in self.moving_objects:
            obj.forces.append([0, -1])
            obj.apply_forces(time_step_size)
            obj.apply_velocity(time_step_size)
            obj.forces = []
    

    
    def closest_point_on_line(self, p1: list[int | float], p2: list[int | float], x0, y0):
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
    def intersection_point(self, p1, p2, p3,p4):
        line1 = [p1, p2]
        line2 = [p3, p4]
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            return False

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return [x, y]

    def distance(self, p1, p2):
        return math.sqrt(np.abs((p2[0] - p1[0])**2 + (p2[1]**2 - p1[1]**2)))

    def point_on_line(self, line: Line, p):        
        return self.distance(line.p1, p) + self.distance(line.p2, p) == self.distance(line.p1, line.p2);
    
    def check_line_circle_intersection(self, circle : Circle, line: Line):
        closest_point = self.closest_point_on_line(line.p1, line.p2, circle.pos[0], circle.pos[1])
        if self.distance(closest_point,  circle.pos) < circle.radius:
            return True
        else:
            return False
        
    def point_on_percent_of_line(self, p1, p2, percent: float):
        line_vector =  p2 - p1
        return p1 + (line_vector * percent) 
    
    def circle_line_collision_response(self, circle: Circle, line: Line, time: float, time_step_size):
        # closest point
        point_1 = np.array(self.closest_point_on_line(line.p1, line.p2, circle.x, circle.y))
        point_2 = circle.pos + time * circle.vel
        point_c = np.array(self.closest_point_on_line(line.p1, line.p2, point_2[0], point_2[1]))
        point_3 = point_2 + (point_1 - point_c)
        
        print(
            'point1', point_1,
            'point2', point_2,
            'point3', point_3,
            'pointc', point_c
        )
        
        direction = point_2 - (point_2 + 2 * (point_2 - point_3))
        direction = np.flip(direction)
        unit_direction = direction / math.sqrt(direction[0]**2 + direction[1]**2 )
        print(unit_direction, direction)
        final_vel = unit_direction * math.sqrt(direction[0]**2 + direction[1]**2 ) 
        
        return final_vel
    
    def circle_circle_collision_response(self, circle1: Circle, circle2: Circle, time: float):
        circle1_collision_point = circle1.pos + time * circle1.vel
        circle2_collision_point = circle2.pos + time * circle2.vel

        
        distance_between_collision = self.distance(circle1_collision_point, circle2_collision_point)
        
        norm = np.divide(circle2_collision_point - circle1_collision_point, distance_between_collision)
        
        p = np.divide((2 * (circle1.vel * norm - circle2.vel * norm)), (circle1.mass + circle2.mass))
        
        c1_final_vel = circle1.vel - p * circle1.mass * norm
        c2_final_vel = circle2.vel + p * circle2.mass * norm
                
        return c1_final_vel, c2_final_vel
    
    def circle_circle_collision_time(self, p1:list|tuple, p2:list|tuple, p3:list|tuple, p4:list|tuple, r1:float|int,r2:float|int, time_step_size=1):
        
        v1 = (p2 - p1) * time_step_size
        v2 = (p4 - p3) * time_step_size
        
        a, b, c, d, g, h, m, n, r = p1[0], p1[1], p3[0], p3[1], v1[0], v1[1], v2[0], v2[1], r1 + r2
        discriminant = ((2*(a - c + g - m - b + d - h + n))**2 
                        - 4 * (a**2 - 2*a*c + b**2 - 2*b*d + c**2 + d**2 - r**2) 
                            * (a**2 - 2*a*c + 2*a*g - 2*a*m + b**2 - 2*b*d + 2*b*h - 2*b*n + c**2 - 2*c*g + 2*c*m + d**2 - 2*d*h + 2*d*n + g**2 - 2*g*m + h**2 - 2*h*n + m**2 + n**2))
        
        if discriminant < 0:
            return None
        
        numerator = (-math.sqrt(discriminant) + 2*a - 2*c + 2*g - 2*m + 2*b - 2*d + 2*h - 2*n)
        denominator = 2 * (a**2 - 2*a*c + 2*a*g - 2*a*m + b**2 - 2*b*d + 2*b*h - 2*b*n + c**2 - 2*c*g + 2*c*m + d**2 - 2*d*h + 2*d*n + g**2 - 2*g*m + h**2 - 2*h*n + m**2 + n**2)

        return numerator / denominator
  
    def circle_line_collision_time(self, circle : Circle, line: Line, time_step_size=1):
        closest_point = self.closest_point_on_line(line.p1, line.p2, circle.x, circle.y)
        distance_to_closest_point = self.distance(circle.pos, closest_point)
        
        ratio_current_vs_collision_triangle = circle.radius / distance_to_closest_point
        
        circle_end_pos = circle.end_pos(time_step_size)
                
        intersection_point = self.intersection_point(circle.pos, circle_end_pos, line.p1, line.p2)

        intersection_point_offset = intersection_point - circle.pos

        collision_point = intersection_point - intersection_point_offset * ratio_current_vs_collision_triangle
        
        if not intersection_point:
            return False, False
                        
        distance_to_collision = self.distance(circle.pos, collision_point)
        distance_to_end = self.distance(circle.pos, circle_end_pos)
        
        time = distance_to_collision / distance_to_end
        
        if ((0 > time or time > 1)):
            return False, False

        return time, collision_point
        
    def point_in_rectangle(self, A : list, B : list, C, D, P):
        AB = [B[0] - A[0], B[1] - A[0]]
        AP = [P[0] - A[0], P[1] - A[0]]
        BC = [C[0] - B[0], B[1] - B[0]]
        BP = [P[0] - B[0], P[1] - B[0]]
        
        # check the x offset to the maximum possible offset
        X = np.dot(AB, AP)
        Xmax = np.dot(AB, AB)
        Y = np.dot(BC, BP)
        Ymax = np.dot(BC, BC)
        
        return 0 <= X <= Xmax and 0 <= Y <= Ymax
    
    # only need 
    def check_rectangle_intersection(self, p1, p2, p3, p4, r1, r2, r3, r4):
        # check if each line of the path of the circles crosses another
        # rect 1 lines
        rect1_lines = [
            Line(p1, p2),
            Line(p2, p3),
            Line(p3, p4),
            Line(p4, p1),
        ]
        # rect 2 lines
        rect2_lines = [
            Line(r1, r2),
            Line(r2, r3),
            Line(r3, r4),
            Line(r4, r1),
        ]
        
        all_comparisons = []
        
        for line1 in rect1_lines:
            for line2 in rect2_lines:
                all_comparisons.append(type(self.line_segment_intersection(line1.p1, line1.p2, line2.p1, line2.p2)) == list)
        
        # check if one of the points is inside the rect
        points_in_rect = [
            self.point_in_rectangle(p1, p2, p3, p4, r1),
            self.point_in_rectangle(p1, p2, p3, p4, r2),
            self.point_in_rectangle(p1, p2, p3, p4, r3),
            self.point_in_rectangle(p1, p2, p3, p4, r4),
        ]

        return any(all_comparisons) and any(points_in_rect)
        
    # moving circles
    def check_circle_circle_collision(self, circle1: Circle, circle2 : Circle, time_step_size=1):
        circle1_end_pos = circle1.end_pos(time_step_size)
        # ends at the "next" position
        circle1_vector = Line(list(circle1.pos), circle1_end_pos)

        circle2_end_pos = circle2.end_pos(time_step_size)
        # ends at the "next" position
        circle2_vector = Line(list(circle2.pos), circle2_end_pos)        
        # 1: lines intersect
        if (self.line_segment_intersection(circle1_vector.p1, circle1_vector.p2, circle2_vector.p1, circle2_vector.p2)):
            return True
        
        # 2: the end pos is both the radii from the other line
        close_c1start = self.closest_point_on_line(circle2.pos, circle2_end_pos, circle1.x, circle1.y)
        close_c1end = self.closest_point_on_line(circle2.pos, circle2_end_pos, circle1_end_pos[0], circle1_end_pos[1])
        close_c2start = self.closest_point_on_line(circle1.pos, circle1_end_pos, circle2.x, circle2.y)
        close_c2end = self.closest_point_on_line(circle1.pos, circle1_end_pos, circle2_end_pos[0], circle2_end_pos[1])
        
        intersects_on_endpoint = any([
            self.distance(close_c1start, circle1.pos) < circle1.radius + circle2.radius,
            self.distance(close_c1end, circle1_end_pos) < circle1.radius + circle2.radius,
            self.distance(close_c2start, circle2.pos) < circle1.radius + circle2.radius,
            self.distance(close_c2end, circle2_end_pos) < circle1.radius + circle2.radius,
        ])
        
        if intersects_on_endpoint:
            return True
        
        return False

    def check_circle_line_collision(self, circle : Circle, line: Line, time_step_size=1):
        circle_end_pos = circle.end_pos(time_step_size)
        # ends at the "next" position
        circle_vector = Line(list(circle.pos), circle_end_pos)
 
        # 1: lines intersect
        if (self.line_segment_intersection(circle_vector.p1, circle_vector.p2, line.p1, line.p2)):
            return True
        
        # 2: the end pos is both the radius from the other line
        # c = circle vector
        # l = line
        close_cstart = self.closest_point_on_line(line.p1, line.p2, circle.x, circle.y)
        close_cend = self.closest_point_on_line(line.p1, line.p2, circle_end_pos[0], circle_end_pos[1])
        close_lstart = self.closest_point_on_line(circle_vector.p1, circle_vector.p2, line.p1[0], line.p1[1])
        close_lend = self.closest_point_on_line(circle_vector.p1, circle_vector.p2, line.p2[0], line.p2[1])
        
        intersects_on_endpoint = any([
            self.distance(close_cstart, circle.pos) < circle.radius,
            self.distance(close_cend, circle_end_pos) < circle.radius,
            self.distance(close_lstart, line.p1) < circle.radius,
            self.distance(close_lend, line.p2) < circle.radius,
        ])
        
        if intersects_on_endpoint:
            return True
        
        return False
  
    def circle_line_collision(self, circle : Circle, line: Line, time_step_size=1):
        # https://ericleong.me/research/circle-line/
        # Moving Circle and Static Line Segment
        
        vector_end_pos = circle.end_pos(time_step_size)
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
    
    def point_to_line_dist(self, point, p1, p2):
        line = np.array([p1, p2])
        
        # unit vector
        unit_line = line[1] - line[0]
        norm_unit_line = unit_line / np.linalg.norm(unit_line)

        # compute the perpendicular distance to the theoretical infinite line
        segment_dist = (
            np.linalg.norm(np.cross(line[1] - line[0], line[0] - point)) /
            np.linalg.norm(unit_line)
        )

        diff = (
            (norm_unit_line[0] * (point[0] - line[0][0])) + 
            (norm_unit_line[1] * (point[1] - line[0][1]))
        )

        x_seg = (norm_unit_line[0] * diff) + line[0][0]
        y_seg = (norm_unit_line[1] * diff) + line[0][1]

        endpoint_dist = min(
            np.linalg.norm(line[0] - point),
            np.linalg.norm(line[1] - point)
        )

        # decide if the intersection point falls on the line segment
        lp1_x = line[0][0]  # line point 1 x
        lp1_y = line[0][1]  # line point 1 y
        lp2_x = line[1][0]  # line point 2 x
        lp2_y = line[1][1]  # line point 2 y
        is_betw_x = lp1_x <= x_seg <= lp2_x or lp2_x <= x_seg <= lp1_x
        is_betw_y = lp1_y <= y_seg <= lp2_y or lp2_y <= y_seg <= lp1_y
        if is_betw_x and is_betw_y:
            return segment_dist
        else:
            # if not, then return the minimum distance to the segment endpoints
            return endpoint_dist

    
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
            
        
        
        
        
    
    
        