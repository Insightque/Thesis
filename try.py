import graphics
import math

SIZEX = 1000
SIZEY = 700
BORDER = 100
RADIUS = (BORDER / 2) + 1
WINDOW = graphics.GraphWin("Pathfind", SIZEX, SIZEY)
CIRCLES = []
pi = 3.14


class Vector:
    def __init__(self, x2=0, y2=0, x1=0, y1=0):
        self.x = x2 - x1
        self.y = y2 - y1


class Node:
    """represents the points / nodes of the map"""
    direction = 0
    parent = None
    g = 0
    h = 0
    f = 0

    def set_distance(self, reference):
        # dist2 = (self.parent.x - self.x)**2 + (self.parent.y - self.y)**2
        # self.g = math.sqrt(dist2)
        if self.parent is not None:
            self.g = self.parent.circle.r
        dist2 = (reference.x - self.x)**2 + (reference.y - self.y)**2
        self.h = math.sqrt(dist2)
        self.f = self.g + self.h

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.circle = Circle(x, y, RADIUS)



class Circle:
    """ Shape to draw"""

    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r
        CIRCLES.append(self)

    def draw_circle(self):
        pt = graphics.Point(self.x, self.y)
        pt.draw(WINDOW)
        circle = graphics.Circle(pt, self.r)
        circle.draw(WINDOW)
        return circle


class Search:
    """Structure for the algorithm"""

    S_open = []
    S_closed = []

    def expand(self, node):
        generated = []
        for i in range(0, 5):
            alpha = (3.14/2) * (i/4)
            dy = node.circle.r * math.sin(alpha)
            dx = node.circle.r * math.cos(alpha)
            
            node_new1 = Node(node.circle.x + dx, node.circle.y + dy)
            node_new1.direction = Vector(dx, dy)

            node_new2 = Node(node.circle.x - dx, node.circle.y + dy)
            node_new2.direction = Vector(-dx, dy)

            node_new3 = Node(node.circle.x + dx, node.circle.y - dy)
            node_new3.direction = Vector(dx, -dy)

            node_new4 = Node(node.circle.x - dx, node.circle.y - dy)
            node_new4.direction = Vector(-dx, -dy)

            new_nodes = [node_new1, node_new2, node_new3, node_new4]
            for n in new_nodes:
                n.parent = node
                n.set_distance(self.n_goal)
            self.S_open += new_nodes

    def __init__(self, Start, Goal):
        # define the start circle
        self.n_start = Start
        self.n_start.circle.draw_circle().setOutline("green")
        self.S_open.append(self.n_start)

        # define the goal circle
        self.n_goal = Goal
        self.n_goal.circle.draw_circle().setOutline("red")
        self.n_goal.set_distance()

        self.n_start.set_distance(self.n_goal)
        self.n_goal.set_distance(self.n_start)

    def exist(self, node):
        return False

    def space_exploration(self):
        self.expand(self.n_start)
        for node in self.S_open:
            node.circle.draw_circle().setOutline("purple")
       # here to start
        while self.S_open:
            node = self.S_open.pop()
            if self.n_goal.f < node.f:          # TODO: valahogy ellenőrizzük ha ott vagyunk a célnál
                return True

            elif not self.exist(node):
                self.expand(node)
                # TODO: folytasd....

def main():
    start = Node(BORDER, BORDER)
    goal = Node(SIZEX, SIZEY)
    test = Search(start, goal)
    test.space_exploration()
    while True:
        """Waiting for click and closing the window"""
        try:
            WINDOW.getMouse()
        except graphics.GraphicsError:
            pass


try:
    main()
except:
    raise
finally:
    WINDOW.close()
