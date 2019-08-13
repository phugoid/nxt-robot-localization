import nxt.bluesock
from nxt.motor import *
from nxt.sensor import *
import pdb
from math import *
import turtle
import random
from time import sleep

# Local module
from intersection import *

VERBOSE        = True

# GEOMETRY AND ROBOT PARAMS
TABLE_X_MAX    = 120.  # cm bigtable=180
TABLE_Y_MAX    = 60.   # cm bigtable=90
WHEEL_RADIUS   = 2.78  # cm determined empirically
BETWEEN_WHEELS = 13.   # cm, lateral distance between wheels, used for turning geometry
HEIGHT_THRESH  = 10.   # cm
FWD_POWER      = 60    # percent, less than 60 motor won't turn, but the lower/slower the better
TURN_POWER     = 80    # percent
ARM_LENGTH     = 13.   # cm, dist from center of axle to ultrasonic sensor
PULL_BACK      = True  # after sensing edge of table, robot pulls back until sensing table edge again
PULL_BACK_DIST = 3.    # how far we pull back from edge, on average (bluetooth latency / control lag)
AFTER_MOVE_WAIT= 0.5   # sec time for robot to stabilize after turning

# PARTICLE FILTER
SIGMA          = 5.    # 20 cm, stdev of distance, noisy due to movement/control uncertainty
NUM_PARTICLES  = 50000
NUM_PARTICLES2 = 5000  # 5000
NUM_XY_EST     = 10    # how many particles to use for XY estimate

# FUZZING
FRAC_UNFUZZED  = 0.25 # 0.75
FUZZ_POS       = 3.  # 5.
FUZZ_HDG       = 5. * pi/180.  # 5.

# SIMULATION
SIMULATION = False
SIM_INIT_X = TABLE_X_MAX/2.
SIM_INIT_Y = TABLE_Y_MAX/2.
SIM_INIT_H = 0.
SIM_DIST_NOISE = 5.
SIM_ANGLE_NOISE = 10. * pi/180.

# DISPLAY
PLOT = True
PLOT_WINDOW_X = 0 # Window offset
PLOT_WINDOW_Y = 0 # Window offset
PLOT_CANVAS_X = TABLE_X_MAX * 16 # 6 Controls size of window
PLOT_CANVAS_Y = TABLE_Y_MAX * 16
PLOT_SHOW_ESTIMATE = False
PLOT_MARGIN = 0.2
PLOT_WAIT = 2. # sec

# MISC
EPSILON = 1e-16
BREAK_ON_NORMALIZE_DIV0 = False

def truncate_angle(t):
    return ((t + pi) % (2 * pi)) - pi
    
def collision_ray_to_segment(x,y,h, x0,y0, x1,y1):
    """Extend a ray from current pos x,y to a long farther away (R) than we could ever need,
    then check for intersection between this segment and the one defined by (x0,y0), (x1,y1)"""
    R = 2.* (TABLE_X_MAX**2 + TABLE_Y_MAX**2) ** 0.5
    p0 = (x,y)
    p1 = (x + cos(h)*R, y + sin(h)*R)
    p2 = (x0, y0)
    p3 = (x1, y1)
    return find_intersection(p0, p1, p2, p3)
    
def get_dist_to_edge(x,y,h):
    """Compute distance to edges (intersections), for a particle sitting at x,y with heading h.
    A bit tricky, because it might start out sitting "outside" the table.
    So we're looking for a collision with the far edges of the table."""
    found = False
    edges = {
        'N': ( (0., TABLE_Y_MAX), (TABLE_X_MAX, TABLE_Y_MAX) ),
        'E': ( (TABLE_X_MAX, TABLE_Y_MAX), (TABLE_X_MAX, 0.) ),
        'S': ( (0., 0.), (TABLE_X_MAX, 0.)),
        'W': ( (0., TABLE_Y_MAX), (0., 0.))
    }
    collisions = {}
    for k,v in edges.iteritems():
        p = collision_ray_to_segment(x, y, h, v[0][0], v[0][1], v[1][0], v[1][1])
        if p is not None:
            collisions[k] = p
            
    # Lots of edge cases - our particle starting point might be outside the table
    # and its ray intersects with edges twice, etc.  So check only for collisions that
    # are relevant to current particle heading.
    # By quadrant:
    cd = collisions.keys()
    cp = None
    if h >= 0. and h < pi/2.:
        if 'N' in cd or 'E' in cd:
            if 'N' in cd and not 'E' in cd:
                cp = collisions['N']
            elif 'E' in cd and not 'N' in cd:
                cp = collisions['E']
            else:
                exit('FATAL: Collision anomaly')

    elif h >= pi/2. and h <= pi:
        if 'N' in cd or 'W' in cd:
            if 'N' in cd and not 'W' in cd:
                cp = collisions['N']
            elif 'W' in cd and not 'N' in cd:
                cp = collisions['W']
            else:
                exit('FATAL: Collision anomaly')
                
    elif h >= -pi and h <= -pi/2.:
        if 'S' in cd or 'W' in cd:
            if 'S' in cd and not 'W' in cd:
                cp = collisions['S']
            elif 'W' in cd and not 'S' in cd:
                cp = collisions['W']
            else:
                exit('FATAL: Collision anomaly')
                
    else:
        if 'S' in cd or 'E' in cd:
            if 'S' in cd and not 'E' in cd:
                cp = collisions['S']
            elif 'E' in cd and not 'S' in cd:
                cp = collisions['E']
            else:
                exit('FATAL: Collision anomaly')
                
    if cp is None: return 10000.  # TODO: This should get rid of the particle, but...
    xc, yc = cp
    return ((xc-x)**2 + (yc-y)**2)**0.5 # Distance to collision point

    
class RobotSimulated():
    def __init__(self):
        self.x = SIM_INIT_X
        self.y = SIM_INIT_Y
        self.h = SIM_INIT_H
        
    def fwd_to_edge(self):
        to_edge = get_dist_to_edge(self.x, self.y, self.h)
        d = to_edge - ARM_LENGTH + random.gauss(0., SIM_DIST_NOISE)
        if PULL_BACK: # If we pull back, we're assured sensor will end up over the table
            d -= PULL_BACK_DIST
            d = min(d, to_edge - ARM_LENGTH - 1.0)
        self.x += cos(self.h) * d
        self.y += sin(self.h) * d
        return d + PULL_BACK_DIST + ARM_LENGTH
    
    def turn_angle(self, angle):
        # Location of center of turn (position of tire contact point we're turning about)
        TURN_RAD = BETWEEN_WHEELS / 2.
        if angle > 0.:                                # Rotate about the right tire
            cx = self.x + cos(self.h - pi/2.) * TURN_RAD
            cy = self.y + sin(self.h - pi/2.) * TURN_RAD
        else:                                         # Rotate about the left tire
            cx = self.x + cos(self.h + pi/2.) * TURN_RAD
            cy = self.y + sin(self.h + pi/2.) * TURN_RAD
        # New shifted position of robot coords origin
        if angle > 0.:
            self.x = cx + cos(self.h + pi/2. + angle) * TURN_RAD
            self.y = cy + sin(self.h + pi/2. + angle) * TURN_RAD
        else:
            self.x = cx + cos(self.h - pi/2. + angle) * TURN_RAD 
            self.y = cy + sin(self.h - pi/2. + angle) * TURN_RAD
            
        # Update heading
        self.h += angle + random.gauss(0., SIM_ANGLE_NOISE)
        self.h = truncate_angle(self.h)
        
    def fwd_dist(self, d):
        self.x += cos(self.h) * d
        self.y += sin(self.h) * d

    def back_dist(self, d):
        self.x -= cos(self.h) * d
        self.y -= sin(self.h) * d

    def idle(self):
        pass
    
    
class Robot():
    def __init__(self):
        try:
            if VERBOSE: print "Connecting to LEGO brick..."
            self.b = nxt.bluesock.BlueSock('00:16:53:06:43:F0').connect()
        except Exception as e:
            print "FATAL: Cannot connect to NXT brick"
            print e
            exit(-1)
        if VERBOSE: "Connection successful!"
        self.ml = Motor(self.b, PORT_A)
        self.mr = Motor(self.b, PORT_B)
        self.m = SynchronizedMotors(self.ml, self.mr, 1.)
        self.fwd_power = FWD_POWER
        self.turn_power = TURN_POWER
        self.tacho1 = 0.
        self.tacho2 = 0.
        # self.h = 0. # Need to keep track of heading if our steering commands are not "in-place" and induce position shift

    def fwd_dist(self, distance, direction=1.):
        circum = 2. * pi * WHEEL_RADIUS
        degrees = 360. * distance / circum
        self.m.turn(self.fwd_power*direction, degrees, True) # power, tacho_units, brake=True
        sleep(AFTER_MOVE_WAIT) # Let robot stabilize

    def turn_angle(self,angle):
        # Left motor  (ml): +power for robot fwd
        # Right motor (mr): +power for robot fwd
        a = abs(angle)

        # Only supporting a few discrete turn angles
        if abs(a-pi) < EPSILON:
            degrees = 710 # These values are tuned empirically
        elif abs(a-pi/2.) < EPSILON:
            degrees = 320
        elif abs(a-pi/4.) < EPSILON:
            degrees = 153
        else:
            exit("turn angle not supported")
            
        if angle > 0.:
            self.ml.turn(-self.turn_power, degrees, True) # power, tacho_units, brake=True
        else:
            self.mr.turn(-self.turn_power, degrees, True)
        sleep(AFTER_MOVE_WAIT) # Let robot stabilize
            
    def back_dist(self, distance):
        self.fwd_dist(distance, direction=-1.)

    def fwd(self):
        self.m.run(self.fwd_power)

    def back(self):
        self.m.run(-self.fwd_power)

    def idle(self):
        self.m.idle()
        
    def brake(self):
        self.m.brake()

    def ultrasonic(self):
        return Ultrasonic(self.b, PORT_1).get_sample()
        
    def get_distance(self):
        t = self.m.get_tacho()
        dt1 = t.leader_tacho.tacho_count - self.tacho1
        dt2 = t.follower_tacho.tacho_count - self.tacho2
        dt = (dt1 + dt2) / 2.
        return dt/360. * 2 * pi * WHEEL_RADIUS

    def reset_distance(self):
        t = self.m.get_tacho()
        self.tacho1 = t.leader_tacho.tacho_count
        self.tacho2 = t.follower_tacho.tacho_count
    
    def fwd_to_edge(self):
        self.reset_distance()
        self.fwd()
        while True:
            if r.ultrasonic() > HEIGHT_THRESH:
                r.brake()
                sleep(AFTER_MOVE_WAIT) # Let robot stabilize
                break
        if PULL_BACK:  # Moving slightly back from edge for better accuracy
            r.back()
            while True:
                if r.ultrasonic() < HEIGHT_THRESH:
                    r.brake()
                    sleep(AFTER_MOVE_WAIT) # Let robot stabilize
                    break
        return self.get_distance() + ARM_LENGTH
    

class Particle():
    def __init__(self, x, y, h):
        self.x = x
        self.y = y
        self.h = h

    def dist_to_edge(self):
        return get_dist_to_edge(self.x, self.y, self.h)

    def fwd_dist(self, dist):
        self.x += dist * cos(self.h)
        self.y += dist * sin(self.h)

    def turn_angle(self, angle):
        # Location of center of turn (position of tire contact point we're turning about)
        TURN_RAD = BETWEEN_WHEELS / 2.
        if angle > 0.:                                # Rotate about the right tire
            cx = self.x + cos(self.h - pi/2.) * TURN_RAD
            cy = self.y + sin(self.h - pi/2.) * TURN_RAD
        else:                                         # Rotate about the left tire
            cx = self.x + cos(self.h + pi/2.) * TURN_RAD
            cy = self.y + sin(self.h + pi/2.) * TURN_RAD
        # New shifted position of robot coords origin
        if angle > 0.:
            self.x = cx + cos(self.h + pi/2. + angle) * TURN_RAD
            self.y = cy + sin(self.h + pi/2. + angle) * TURN_RAD
        else:
            self.x = cx + cos(self.h - pi/2. + angle) * TURN_RAD 
            self.y = cy + sin(self.h - pi/2. + angle) * TURN_RAD
            
        # Update heading
        self.h += angle
        self.h = truncate_angle(self.h)

        
def clone_particle(p):
    return Particle(p.x, p.y, p.h)


class ParticleFilter():
    def _get_random_particle(self):
        x = random.uniform(0., TABLE_X_MAX)
        y = random.uniform(0., TABLE_Y_MAX)
        h = random.uniform(-pi, pi)
        return Particle(x,y,h)
        
    def __init__(self):
        self.N = NUM_PARTICLES
        self.N_second_pass = NUM_PARTICLES2
        self.sigma = SIGMA
        self.first_pass = True
        self.p = [] # Particle swarm
        for i in range(self.N):
            self.p.append(self._get_random_particle())
        self.x = 0.
        self.y = 0.
        self.h = 0.
        self.fuzz_pos = FUZZ_POS
        self.fuzz_hdg = FUZZ_HDG

    def _normalize_weights(self, w):
        norm = sum(w)
        if BREAK_ON_NORMALIZE_DIV0 and abs(norm) < 1e-10:
            pdb.set_trace()
        else:
            norm = max(norm, 1e-50) # Avoid div by zero
        for i in range(len(w)):
            w[i] /= norm
        
    def _update_importance_weights(self, measured_dist):
        self.w = []
        for i in range(self.N):
            error = measured_dist - self.p[i].dist_to_edge()
            self.w.append( exp(- (error ** 2) / (self.sigma ** 2) / 2.0) )
        self._normalize_weights(self.w)

    def _reduce(self):
        # Keep only the most likely particles
        if self.first_pass and self.N_second_pass:
            self.first_pass = False
            self.N = self.N_second_pass # Dramatic reduction in particles after first pass
        else:
            return
         
        ws = list(enumerate(self.w))
        ws.sort(key=lambda a: -a[1])
        ws = ws[:self.N]
        p2 = []
        self.w = []
        for i in range(len(ws)):
            p2.append(self.p[ws[i][0]])
            self.w.append(ws[i][1])
        # Now that we're done with previous particle list...
        self.p = p2
        self._normalize_weights(self.w)

    def _update_xy(self):
        # Update estimate of robot's X,Y
        # Assumes that self.p and self.w are the particles and weights,
        # already sorted by decreasing likelihood
        x = 0.
        y = 0.
        h = 0.
        wp = self.w[:NUM_XY_EST]
        self._normalize_weights(wp)
        for i in range(len(wp)):
            x += wp[i] * self.p[i].x
            y += wp[i] * self.p[i].y
            h += wp[i] * self.p[i].h # WHOA - this is problematic in edge cases
        self.x = x
        self.y = y
        self.h = truncate_angle(h)

    def _resample(self):
        p2 = []
        wr = []
        index = int(random.random() * self.N)
        beta = 0.0
        mw = max(self.w)
        for i in range(self.N):
            beta += random.random() * 2.0 * mw
            while beta > self.w[index]:
                beta -= self.w[index]
                index = (index + 1) % self.N
            p2.append(clone_particle(self.p[index]))
            wr.append(self.w[index])
        self.p = p2
        self.w = wr # Warning, self.p and self.w are no longer sorted by decreasing likelihood
      
        # Resort particles by descending likelihood
        ws = list(enumerate(self.w))
        ws.sort(key=lambda a: -a[1])
        ws = ws[:self.N]
        p2 = []
        self.w = []
        for i in range(len(ws)):
            p2.append(self.p[ws[i][0]])
            self.w.append(ws[i][1])
            # Now that we're done with previous particle list...
        self.p = p2

    def _scatter(self):
        for i in range(int(FRAC_UNFUZZED * self.N), self.N):
            self.p[i].x += random.uniform(-self.fuzz_pos, self.fuzz_pos)
            self.p[i].y += random.uniform(-self.fuzz_pos, self.fuzz_pos)
            self.p[i].h += random.uniform(-self.fuzz_hdg, self.fuzz_hdg)
            
    def sense(self, d):
        self._update_importance_weights(d)
        self._reduce()
        self._update_xy()
        self._resample()
        self._scatter()
        
    def fwd_dist(self, dist):
        for i in range(self.N):
            self.p[i].fwd_dist(dist)

    def turn_angle(self, angle):
        for i in range(self.N):
            self.p[i].turn_angle(angle)


def plot_setup():
    if not PLOT: return
    turtle.setup(width=PLOT_CANVAS_X, height=PLOT_CANVAS_Y, startx=PLOT_WINDOW_X, starty=PLOT_WINDOW_Y)
    turtle.mode("standard")
    turtle.setworldcoordinates(
        -TABLE_X_MAX*PLOT_MARGIN,
        -TABLE_Y_MAX*PLOT_MARGIN,
        TABLE_X_MAX*(1.+PLOT_MARGIN),
        TABLE_Y_MAX*(1.+PLOT_MARGIN)
    )
    turtle.hideturtle()
    turtle.penup()
                                                                  
    
def plot(x, y, h, p, robot_x=None, robot_y=None, robot_h=None):
    if not PLOT: return
    tl = []
    s = turtle.Screen()
    turtle.clearscreen()
    s.tracer(0,1)
    
    
    # Draw table frame
    f = turtle.Turtle()
    f.setposition(0.,0.)
    f.setheading(90.)
    f.forward(TABLE_Y_MAX)
    f.setheading(0.)
    f.forward(TABLE_X_MAX)
    f.setheading(-90.)
    f.forward(TABLE_Y_MAX)
    f.setheading(180.)
    f.forward(TABLE_X_MAX)
    f.hideturtle()

    # Draw estimated_pos (from particle filter)
    if PLOT_SHOW_ESTIMATE:
        r = turtle.Turtle()
        r.shape("triangle")
        r.shapesize(0.6, 2) # (0.3, 1)
        r.penup()
        r.setposition(x, y)
        r.settiltangle(h * 180 / pi )
        r.pencolor("blue")
        r.fillcolor("blue")
        r.showturtle()
        
    # Draw particle swarm
    for i in range(len(p)):
        t = turtle.Turtle()
        tl.append(t)
        t.penup()
        t.shape("circle")
        t.shapesize(0.3,0.3) # (0.2,0.2)
        t.setposition(p[i].x,p[i].y)
        
    # Draw true robot (if available, simulation only)
    if robot_x is not None:
        r = turtle.Turtle()
        r.shape("triangle")
        r.shapesize(0.6, 2) # (0.3, 1)
        r.penup()
        r.setposition(robot_x, robot_y)
        r.settiltangle(robot_h * 180 / pi )
        r.pencolor("red")
        r.fillcolor("red")
        r.showturtle()
        
    s.update()
    sleep(PLOT_WAIT)
                                                                                                                      
    
    
if __name__ == "__main__":
    plot_setup()
    if SIMULATION:
        r = RobotSimulated()
    else:
        r = Robot()
    f = ParticleFilter()

    """
    STEER = pi/4. # 3./4. * pi # pi
    r.reset_distance()
    r.fwd_dist(10.)
    r.turn_angle(STEER) # Returns the induced position change 1
    r.fwd_dist(10.)
    r.turn_angle(STEER) # Returns the induced position change 2
    r.fwd_dist(10.)
    r.turn_angle(STEER) # Returns the induced position change 3
    r.fwd_dist(10.)
    r.turn_angle(STEER) # Returns the induced position change 4
    r.fwd_dist(10.)
    r.turn_angle(STEER) # Returns the induced position change 5
    r.fwd_dist(10.)
    r.turn_angle(STEER) # Returns the induced position change 6
    r.fwd_dist(10.)
    r.turn_angle(STEER) # Returns the induced position change 7
    r.fwd_dist(10.)
    r.turn_angle(STEER) # Returns the induced position change 8
    exit()
    
    r.turn_angle(pi)
    print r.get_distance()
    r.reset_distance()
    r.fwd_dist(10.)
    r.turn_angle(-pi)
    print r.get_distance()
    r.reset_distance()
    r.fwd_dist(10.)
    exit()
    """
    
    # GO!
    raw_input("Press any key to start!")
    iterations = 0
    STEER = [ pi/2., pi/4. ]  # Steering sequence
    while True:
        d = r.fwd_to_edge()
        f.sense(d)
        f.fwd_dist(d-ARM_LENGTH)
        if SIMULATION:
            plot(f.x, f.y, f.h, f.p, robot_x=r.x, robot_y=r.y, robot_h=r.h)
        else:
            plot(f.x, f.y, f.h, f.p)
            
        # Re-orient the vehicle
        steer = STEER[iterations % 2]
        r.turn_angle(steer) # Returns the induced position change
        f.turn_angle(steer)
        iterations += 1
