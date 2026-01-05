from sticks import Node, Stick, Vector2d
import pygame
import time
import random
import os
import json

pygame.init()
pygame.font.init()
# screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
screen = pygame.display.set_mode((1400, 700))
font = pygame.font.SysFont('lucidaconsole', 30)
pygame.display.set_caption('Cloth Simulation')

WIDTH, HEIGHT = screen.get_size()
FPS = 60

node_radius = 5
click_radius = 30
big_click_radius = 40
stick_width = 2

default_gravity = 0.7
gravity = default_gravity
# 0.7

colors = {
    'locked':(255, 0, 0),
    'node': (255, 255, 255), 
    'clicked': (0, 255, 0),
    'stick': (255, 255, 255),
    'mouse_edge': (128, 128, 128),
    'hovering': (0, 0, 255)
}

def drawNodes(screen, nodes, focus = None):
    for i, node in enumerate(nodes):
        if i == focus:
            color = colors['clicked']
        elif i == hover:
            color = colors['hovering']
        else:
            color = colors['locked' if node.locked else 'node']
        if show_nodes or color != colors['node']:
            pygame.draw.circle(screen, color, node.tuple(), node_radius)

def drawSticks(screen, sticks, nodes = None, focus = None, mouse = None):
    for stick in sticks:
        pygame.draw.line(screen, colors['stick'], stick.p1.tuple(), stick.p2.tuple(), stick_width)
    if focus is not None and menu == 0:
        pygame.draw.line(screen, colors['mouse_edge'], nodes[focus].tuple(), mouse, stick_width)

def getMouseCollision(nodes, mx, my):
    for i, node in enumerate(nodes):
        if node.pos.dist(mx, my) < click_radius:
            return (i, node)
    return None

def overlap(nodes, mx, my):
    for i, node in enumerate(nodes):
        if node.pos.dist(mx, my) < 2*node_radius:
            return True
    return False

def unlockAll(nodes):
    for node in nodes:
        node.locked = False

def deleteSticks(node, sticks):
    invalid = lambda stick: stick.p1.equals(node) or stick.p2.equals(node)
    sticks = [stick for stick in sticks if not invalid(stick)]
    return sticks

def prompt(prompt, func):
    stuff = ''
    while not func(stuff):
        stuff = input(prompt)
        if stuff == 'q': return False
    return stuff

def save(nodes, sticks):
    os.makedirs('saves', exist_ok = True)
    path = prompt("file to save to: ", lambda x: x)
    if not path:
        return
    with open(f"saves/{path}.json", "w") as outfile:
        d = {}
        d['nodes'] = [x.tuple() for x in nodes]
        d['sticks'] = [[nodes.index(stick.p1), nodes.index(stick.p2)] for stick in sticks]
        d['locked'] = [x.locked for x in nodes]
        json.dump(d, outfile)

def load():
    os.makedirs('saves', exist_ok = True)
    path = prompt("file to load from: ", lambda x: os.path.isfile(f'saves/{x}.json'))
    if not path:
        return [False, [], []]
    with open(f"saves/{path}.json", "r") as infile:
        d = json.load(infile)
        nodes = [Node(x, y, False) for (x,y) in d['nodes']]
        sticks = [Stick(nodes[a], nodes[b]) for (a, b) in d['sticks']]
        for i in range(len(d['locked'])):
            nodes[i].locked = d['locked'][i]
    return [True, nodes, sticks]

def draw_grid():
    # 1200, 420 (20)
    for x in range(200, 1200, 20):
        for y in range(100, 500, 20):
            nodes.append(Node(x, y, y == 100))
    for node1 in nodes:
        for node2 in nodes:
            if (node1.pos - node2.pos).magnitude() == 20:
                sticks.append(Stick(node1, node2))

def points():
    for x in range(200, 1200, 10):
        for y in range(100, 400, 20):
            nodes.append(Node(x, y, False))

focus = None
show_nodes = True
boundaries = False
menu = 0
nodes = []
sticks = []

draw_grid()
# points()

mx, my = pygame.mouse.get_pos()
mx_prev, my_prev = mx, my

clock = pygame.time.Clock()
prev_time = time.time()
running = True
while running:
    now = time.time()
    dt = now - prev_time
    prev_time = now
    click = False
    hover = None

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                running = False
            if event.key == pygame.K_g:
                gravity = 0 if gravity else default_gravity
            if event.key == pygame.K_u:
                ans = prompt("are you sure you want to unlock all nodes? ", lambda x: x.lower() in ['y', 'n'])
                if ans == 'y':
                    unlockAll(nodes)
            if event.key == pygame.K_c:
                sticks = []
                nodes = []
                focus = None
                hover = None
            if event.key == pygame.K_n:
                show_nodes = not show_nodes
            if event.key == pygame.K_b:
                boundaries = not boundaries
            if event.key == pygame.K_s:
                save(nodes, sticks)
            if event.key == pygame.K_l:
                success, l_nodes, l_sticks = load()
                if success:
                    nodes = l_nodes
                    sticks = l_sticks
            if event.key == pygame.K_ESCAPE:
                focus = None
            if event.key == pygame.K_d:
                if focus is not None:
                    sticks = deleteSticks(nodes[focus], sticks)
                    del nodes[focus]
                    focus = None
            if event.key == pygame.K_BACKSPACE:
                menu = menu - 1 if menu > 0 else menu
                focus = None
            if event.key == pygame.K_RETURN:
                menu = menu + 1 if menu < 2 else menu
                focus = None
                
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            click = True
    
    hold = pygame.mouse.get_pressed()[0]
    
    mx, my = pygame.mouse.get_pos()
    m_vel = Vector2d(mx - mx_prev, my - my_prev)
    mx_prev, my_prev = mx, my
    collision = getMouseCollision(nodes, mx, my)
    if collision:
        mouse_i, mouse_node = collision
    

    if menu == 0:
        if collision and click:
            click = False
            if focus is not None and mouse_i != focus:
                stick = Stick(nodes[focus], mouse_node)
                sticks.append(stick)
                focus = None
            elif focus is not None and mouse_i == focus:
                focus = None
            elif focus is None:
                focus = mouse_i
        elif collision and not click:
            if hover != focus or focus is None:
                hover = mouse_i
        elif not collision and click:
            if not overlap(nodes, mx, my):
                node = Node(mx, my, False)
                nodes.append(node)
                if focus is not None:
                    stick = Stick(nodes[focus], node)
                    sticks.append(stick)
                    focus = None

    if menu == 1:
        if collision and not click:
            if not mouse_node.locked:
                hover = mouse_i
        elif collision and click:
            mouse_node.locked = not mouse_node.locked

    if menu == 2:
        if collision:
            if click:
                focus = mouse_i
            else:
                hover = mouse_i

        if hold and focus is not None:
            nodes[focus].pos.x = mx
            nodes[focus].pos.y = my
        else:
            focus = None

        for node in nodes:
            node.update(gravity, dt * FPS, 0.99)
        # sticks = random.sample(sticks, len(sticks))
        for _ in range(3):
            for stick in sticks:
                stick.constrain()
            if boundaries:
                for node in nodes:
                    node.constrain(node_radius, WIDTH, HEIGHT, 0.9)

    mode_text = font.render(f'{menu}', False, (255, 255, 255))
    fps_text = font.render(f'{int(clock.get_fps())}', False, (255, 255, 255))


    screen.fill((0, 0, 0))
    drawNodes(screen, nodes, focus)
    drawSticks(screen, sticks, nodes, focus, (mx, my))
    screen.blit(mode_text, (50, 50))
    screen.blit(fps_text, (50, 90))
    pygame.display.update()
    clock.tick(FPS)