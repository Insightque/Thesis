import graphics
import math

SIZEX = 1000
SIZEY = 700
BORDER = 50
RADIUS = (BORDER / 2) + 1
WINDOW = graphics.GraphWin("Pathfind", SIZEX, SIZEY)
CIRCLES_2_DRAW = []
pi = 3.14
# SAMPLE = 5
SAMPLE = 5

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

    def calc_cost(self, reference):
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
    NODES = []

    def __init__(self, Start, Goal):
        # define the start circle
        self.n_start = Start
        self.n_start.circle.draw_circle().setOutline("green")
        self.S_open.append(self.n_start)
        self.NODES.append(self.n_start)

        # define the goal circle
        self.n_goal = Goal
        self.n_goal.circle.draw_circle().setOutline("red")

        self.n_start.calc_cost(self.n_goal)
        self.n_goal.calc_cost(self.n_start)                  # TODO: a cél távolságának becslésére ez jó?
        self.n_goal.f *= 3

    def expand(self, node):
        global CIRCLES_2_DRAW
        for i in range(0, SAMPLE):
            alpha = (3.14 / 2) * (i / (SAMPLE - 1))
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
            for index, n in reversed(list(enumerate(new_nodes))):
                n.parent = node
                n.calc_cost(self.n_goal)
                for element in self.NODES:
                    if element.x == n.x and element.y == n.y and element.direction == n.direction:  # TODO: nem tudom hogy itt kell e gyorsítani de próbának jó lesz
                        new_nodes.pop(index)
                        break
                    elif calc_distance(element, n) < n.circle.r:
                        new_nodes.pop(index)
                        break
            self.S_open += new_nodes
            CIRCLES_2_DRAW += new_nodes
            self.NODES += new_nodes

        # drawing the circles to the window

        for index, node in reversed(list(enumerate(CIRCLES_2_DRAW))):
            node.circle.draw_circle().setOutline("purple")
            CIRCLES_2_DRAW.pop(index)

    def not_exist(self, node):
        not_find = True
        for finished in self.S_closed:
            if node == finished:
                not_find = False
                break
        return not_find

    def PopNearest(self):                       # TODO: Itt mit is keresünk? f-et? az így nagyon elment?
        """pop from the S_open the nearest node to the goal, to be evaluated"""
        # TODO: Geci lassu
        min_index = 0
        min_dist = self.S_open[0].h
        for index, node in enumerate(self.S_open,0):
            if node.h < min_dist:
                min_index = index
                min_dist = node.h
        return self.S_open.pop(min_index)

    def overlap(self,node1,node2):
        if calc_distance(node1, node2) <= (node1.circle.r + node2.circle.r):
            return True
        else:
            return False

    def recurse(self):
        node = self.n_goal
        while node != self.n_start:
            parent = node.parent
            pt1 =graphics.Point(parent.x,parent.y)
            pt2 = graphics.Point(node.x,node.y)
            ln = graphics.Line(pt1, pt2)
            ln.setWidth(10)
            ln.setOutline('orange')
            ln.draw(WINDOW)
            node = parent

    def space_exploration(self):
       # TODO: here to start
        while self.S_open:
            node = self.PopNearest()
            if self.n_goal.parent is not None:          # TODO: valahogy ellenőrizzük ha ott vagyunk a célnál IDK if it's good
                self.recurse()
                return True

            elif self.not_exist(node):
                self.expand(node)
                if self.overlap(node, self.n_goal):
                    self.n_goal.g = node.f
                    self.n_goal.parent = node
                self.S_closed.append(node)
        return False

def main():
    start = Node(BORDER, BORDER)
    goal = Node(SIZEX-BORDER, SIZEY-BORDER)
    test = Search(start, goal)
    if test.space_exploration():
        print("DONE")
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
