import math
import pygame
from physics_objects import Circle, PhysicsEnvironment
import time


scaling = 1

pygame.init()

(width, height) = (500, 400)
sim_width, sim_height = 20, 20
sim_scaling = 10
screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
pygame.display.flip()
clock = pygame.time.Clock()  
font = pygame.font.Font('freesansbold.ttf', 32)

    
circle1 = Circle(5, 5)
circle2 = Circle(7, 7)
environment = PhysicsEnvironment(sim_scaling, sim_scaling, [circle1, circle2])

time_delta = 1
running = True
while running:
    start_time = time.time()
    
    screen_width, screen_height = pygame.display.get_surface().get_size()
    
    if screen_width / screen_height > 1:
        screen_scaling = screen_height / sim_scaling
        sim_field_x_offset = (screen_width - screen_height) / 2
        sim_field_y_offset = screen_height


    else:
        screen_scaling = screen_width / sim_scaling
        sim_field_x_offset = 0
        sim_field_y_offset = screen_height - ((screen_height - screen_width) / 2)
    
    toScreenCoords = lambda pos: [(pos[0]) * screen_scaling + sim_field_x_offset, (-pos[1]) * screen_scaling + sim_field_y_offset]
    screen.fill((0, 0, 0))
    
    origin_point = toScreenCoords((0,0))
    top_right = toScreenCoords((10, 10))

    sim_area = pygame.rect.Rect(origin_point[0], top_right[1], top_right[0] - origin_point[0], origin_point[1] - top_right[1])
    pygame.draw.rect(screen, (100, 100, 100), sim_area)
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    environment.run_tick(time_delta)
        
        
    for object in environment.objects:
        centre_on_screen = toScreenCoords((object.pos[0], object.pos[1]))
        pygame.draw.circle(screen, (0, 0, 255), centre_on_screen, radius=object.radius * screen_scaling)
        point_on_circle = (math.cos(object.rot) * object.radius + object.pos[0], math.sin(object.rot) * object.radius + object.pos[1],)
        point_on_circle_on_screen = toScreenCoords(point_on_circle)
        
        pygame.draw.line(screen, (0, 255, 0), centre_on_screen,  point_on_circle_on_screen)

    fps = font.render(str(round(1 / time_delta)), True, (255, 255, 255))
    textRect = fps.get_rect()
    textRect.center = (50, 20)
    screen.blit(fps, textRect)



    pygame.display.flip()
    end_time = time.time()
    time_delta = end_time - start_time + 0.001
      
