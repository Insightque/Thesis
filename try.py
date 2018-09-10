import graphics
import math

SIZEX = 1000
SIZEY = 700
BORDER = 100
RADIUS = (BORDER / 2) + 1
WINDOW = graphics.GraphWin("Pathfind", SIZEX, SIZEY)
CIRCLES_2_DRAW = []
pi = 3.14
# SAMPLE = 5
SAMPLE = 2

def calc_distance(node1, node2):
    """calculating Eucledian distance from a node to an other"""
    distance = (node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2
    return math.sqrt(distance)

class Vector:
    """Vector to know the directions"""
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
            self.g = self.parent.g + self.parent.circle.r                               # g : distance from the start node
        self.h = calc_distance(self, reference)                                       # h : heuristic distance to the goal
        self.f = self.g + self.h                                        # f : total cost = g + h

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

    def draw_circle(self):
        pt = graphics.Point(self.x, self.y)
        circle = graphics.Circle(pt, self.r)
        pt.draw(WINDOW)
        circle.draw(WINDOW)
        return circle


class Search:
    """Structure for the algorithm"""

    S_open = []
    S_closed = []

    def expand(self, node):
        global CIRCLES_2_DRAW
        for i in range(0, SAMPLE):
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
                for index, element in enumerate(self.S_open,0):
                    if element.x == n.x and element.y == n.y and element.direction == n.direction and (calc_distance(n, element) <= n.r):              # TODO: nem tudom hogy itt kell e gyorsítani de próbának jó lesz
                        self.S_open.pop(index)
            self.S_open += new_nodes



        # drawing the circles to the window
        CIRCLES_2_DRAW += new_nodes
        for index,node in enumerate(CIRCLES_2_DRAW,0):
            node.circle.draw_circle().setOutline("purple")
            CIRCLES_2_DRAW.pop(index)

    def __init__(self, Start, Goal):
        # define the start circle
        self.n_start = Start
        self.n_start.circle.draw_circle().setOutline("green")
        self.S_open.append(self.n_start)

        # define the goal circle
        self.n_goal = Goal
        self.n_goal.circle.draw_circle().setOutline("red")

        self.n_start.set_distance(self.n_goal)
        self.n_goal.set_distance(self.n_start)                  # TODO: a cél távolságának becslésére ez jó?

    def not_exist(self, node):
        """Checking if there is an existing, redundant circle"""
        return True

    def PopNearest(self):
        """pop from the S_open the nearest node to the goal, to be evaluated"""
        # TODO: Geci lassu
        min_index = 0
        min_dist = self.S_open[0].f
        for index, node in enumerate(self.S_open,0):
            if node.f < min_dist:
                min_index = index
                min_dist = node.f
        return self.S_open.pop(min_index)

    def space_exploration(self):
        self.expand(self.n_start)


       # TODO: here to start
        while self.S_open:
            node = self.PopNearest()
            if calc_distance(self.n_goal, node) < self.n_goal.circle.r:          # TODO: valahogy ellenőrizzük ha ott vagyunk a célnál IDK if it's good
                return True

            elif self.not_exist(node):
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
    if graphics.GraphicsError:
        raise
    else:
        raise
finally:
    WINDOW.close()
