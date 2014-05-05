#!/usr/bin/env python

import math
import pygame
import pygame.locals as pl
import sys
import copy

class Footprint:
    x = 0
    y = 0
    theta = 0
    hand = "left"
    def __init__(self, hand="left", x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.hand = hand
    def __repr__(self):
        return "[" + str(self.x) + "," + str(self.y) + " @ " + str(self.theta) + "]"

def circle(r, d, v, Lmax, dThetaMax, thetaToe, initLeft=None, initRight=None, stance="left"):
    """Does the math to generate a set of footprints for walking in a
    circular arc of radius r and distance d. The circle starts with
    the robot at x=0, y=0 and facing in the theta=0 direction.

    r: radius of the circle
    d: distance to move the center of the gait along the circle
    Lmax: maximum arc length between two steps with the same foot
    dThetaMax: maximum angle between successive steps
    thetaToe: angle by which to to rotate the toes out to prevent knee collisions
    initLeft: initial configuration of the left foot
    initRight: initial configuration of the right foot
    stance: which foot starts out fixed

    """
    
    # initialize result list
    result = [None]
    
    # fill out initial footsteps and transforms
    stance_foot = None
    if stance == "left":
        stance_foot = initLeft;
    else:
        stance_foot = initRight;
    def circle_to_world(coord):
        s = math.sin(stance_foot.theta)
        c = math.cos(stance_foot.theta)
        offset = v * (-1 if stance == "left" else 1)
        return [stance_foot.x + offset * -s + coord[0] * c + coord[1] * -s,
                stance_foot.y + offset * c + coord[0] * s + coord[1] * c,
                stance_foot.theta + coord[2]]
    
    if (d < .0001): return result
    
    # select minimize K subject to conditions, compute resulting dTheta
    K = int(math.ceil(d / Lmax * abs((r - v) / r))) # number of "stages"
    dTheta = d / (K * r)
    if (abs(dTheta) > dThetaMax):
        K = int(math.ceil(d / (abs(r) * dThetaMax)))
        dTheta = d / (K * r)
        
    # loop over footsteps 2 to K+1
    for i in xrange(2, K+1):
        theta_i = dTheta * (i - 1)
        if (i % 2 == (1 if stance == "left" else 0)):         # i odd means this step is for the stance foot
            result.append(Footprint(hand = "left",
                                    x = (r - v) * math.sin(theta_i),
                                    y = r - (r - v) * math.cos(theta_i),
                                    theta = theta_i + thetaToe))
        else:                   # odd, right foot
            result.append(Footprint(hand = "right",
                                    x = (r + v) * math.sin(theta_i),
                                    y = r - (r + v) * math.cos(theta_i),
                                    theta = theta_i - thetaToe))
    
    # fill out footsteps K+1 (N-2) and K+2 (N-1) manually
    theta_last = dTheta * K
    if (K % 2 == (0 if stance == "left" else 1)):         # k even means we end on the stance foot
        result.append(Footprint(hand = "left",
                                x = (r - v) * math.sin(theta_last),
                                y = r - (r - v) * math.cos(theta_last),
                                theta = theta_last + thetaToe))
        result.append(Footprint(hand = "right",
                                x = (r + v) * math.sin(theta_last),
                                y = r - (r + v) * math.cos(theta_last),
                                theta = theta_last + thetaToe))
    else:
        result.append(Footprint(hand = "right",
                                x = (r + v) * math.sin(theta_last),
                                y = r - (r + v) * math.cos(theta_last),
                                theta = theta_last + thetaToe))
        result.append(Footprint(hand = "left",
                                x = (r - v) * math.sin(theta_last),
                                y = r - (r - v) * math.cos(theta_last),
                                theta = theta_last + thetaToe))

    # run through result list transforming steps into original frame of reference
    for r in result:
        if not r: continue
        t = circle_to_world([r.x, r.y, r.theta])
        r.x = t[0]
        r.y = t[1]
        r.theta = t[2]
    if stance == "left":
        result[0] = copy.deepcopy(initLeft)
        # result[1] = copy.deepcopy(initRight)
    else:
        result[0] = copy.deepcopy(initRight)
        # result[1] = copy.deepcopy(initLeft)
        
    
    # return resulting list of footsteps
    return result


SCREENRECT = pl.Rect(0, 0, 640, 480) # default screen size

if __name__ == "__main__":
    radius = 1.
    distance = 2.
    width = .2
    max_length = .5
    max_angle = math.pi / 6
    
    foot_l = Footprint(hand="left", x = 0, y = width, theta = 0)
    foot_r = Footprint(hand="right", x = 0, y = -width, theta = 0)
    stance = "right"
    footprints = []
    
    # footprints = circle(r = radius,
    #                     d = distance,
    #                     v = width,
    #                     Lmax = max_length,
    #                     dThetaMax = max_angle,
    #                     thetaToe = 0,
    #                     initLeft = foot_l,
    #                     initRight = foot_r,
    #                     stance = stance)

    # for fp in footprints:
    #     print(str(fp.x) + ", "  # x
    #           + str(fp.y) + ", " # y
    #           + str(.3 * math.cos(fp.theta)) + ", " # xdelta
    #           + str(.3 * math.sin(fp.theta)))       # ydelta
    # for fp in footprints:
    #     print(fp)


    pygame.init()
    winstyle = 0
    bestdepth = pygame.display.mode_ok(SCREENRECT.size, winstyle, 32)
    screen = pygame.display.set_mode(SCREENRECT.size, winstyle, bestdepth)
    done = False

    dist = 1
    rad = 10000
    shoulder = .2
    Lmax = .5
    dThetaMax = math.pi / 6
    
    foot_l = Footprint(hand="left", x = 0, y = shoulder, theta = 0)
    foot_r = Footprint(hand="right", x = 0, y = -shoulder, theta = 0)
    stance = "left"
    footprints = []

    pix_per_meter = 100.0
    hubo_screen_pos = [SCREENRECT.width / 2, 400]
    def screen_to_world(coord):
        return [(hubo_screen_pos[1] - coord[1]) / pix_per_meter,
                -(coord[0] - hubo_screen_pos[0]) / pix_per_meter]
    def world_to_screen(coord):
        return [int(-coord[1] * pix_per_meter + hubo_screen_pos[0]),
                int(-(coord[0] * pix_per_meter - hubo_screen_pos[1]))]

    while not done:
        # clear screen
        pygame.display.get_surface().fill((0,0,0))

        # handle events
        for event in pygame.event.get():
            if event.type == pl.QUIT:
                done = True
            if event.type == pl.KEYDOWN and event.key == pl.K_ESCAPE:
                done = True
            if event.type == pl.MOUSEMOTION:
                mouse_pos = list(event.pos)
                t = screen_to_world(mouse_pos)
                if t[0] == 0 or t[1] == 0: continue
                rad = (t[0]**2 + t[1]**2) / (2 * t[1])
                dist = 2 * abs(rad * math.atan2(-t[1], t[0]))
            if event.type == pl.MOUSEBUTTONDOWN:
                if stance == "left":
                    foot_r = footprints[1]
                    stance = "right"
                else:
                    foot_l = footprints[1]
                    stance = "left"
        
        # recalculate footprints
        footprints = circle(r = rad,
                            d = dist,
                            v = width,
                            Lmax = max_length,
                            dThetaMax = max_angle,
                            thetaToe = 0,
                            initLeft = foot_l,
                            initRight = foot_r,
                            stance = stance)

        # and now draw everything
        pygame.draw.circle(pygame.display.get_surface(),
                           (255,255,255),
                           world_to_screen([0, rad]),
                           abs(int(rad * pix_per_meter)),
                           1)

        foot_len = .2
        foot_wid = .1
        def drawFootprint(fp, center_color, outline_color):
            pts = []
            # pts.append(world_to_screen([fp.x, fp.y]))
            pts.append(world_to_screen([fp.x + foot_len * math.cos(fp.theta) + foot_wid * math.sin(fp.theta),
                                        fp.y + foot_len * math.sin(fp.theta) - foot_wid * math.cos(fp.theta)]))
            pts.append(world_to_screen([fp.x - foot_len * math.cos(fp.theta) + foot_wid * math.sin(fp.theta),
                                        fp.y - foot_len * math.sin(fp.theta) - foot_wid * math.cos(fp.theta)]))
            pts.append(world_to_screen([fp.x - foot_len * math.cos(fp.theta) - foot_wid * math.sin(fp.theta),
                                        fp.y - foot_len * math.sin(fp.theta) + foot_wid * math.cos(fp.theta)]))
            pts.append(world_to_screen([fp.x + foot_len * math.cos(fp.theta) - foot_wid * math.sin(fp.theta),
                                        fp.y + foot_len * math.sin(fp.theta) + foot_wid * math.cos(fp.theta)]))
            # pts.append(world_to_screen([fp.x + foot_len * math.cos(fp.theta) + foot_wid * math.sin(fp.theta),
            #                             fp.y + foot_len * math.sin(fp.theta) - foot_wid * math.cos(fp.theta)]))

            pygame.draw.polygon(pygame.display.get_surface(),
                                outline_color,
                                pts,
                                1)
            pygame.draw.circle(pygame.display.get_surface(),
                               center_color,
                               world_to_screen([fp.x,fp.y]),
                               3)

        for fp in footprints:
            drawFootprint(fp, (0,255,0), (0,0,255))
        drawFootprint(foot_l if stance == "left" else foot_r, (0,255,0), (255,0,0))
        pts = []
        for fp in footprints:
            pts.append(world_to_screen([fp.x, fp.y]))
        pygame.draw.lines(pygame.display.get_surface(),
                          (255, 255, 0),
                          False,
                          pts,
                          1)
        
        pygame.display.update()
