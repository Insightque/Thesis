import graphics
import math
import time

SIZEX = 1500
SIZEY = 900
BORDER = 50
RADIUS = (BORDER / 2) + 1
WINDOW = graphics.GraphWin("Pathfind", SIZEX, SIZEY)
CIRCLES_2_DRAW = []
pi = 3.14
# SAMPLE = 5
SAMPLE = 5
OBSTACLES = []
CARSIZE = 0
VERTICAL = True


def calc_distance(node1, node2):
    """calculating Eucledian distance from a node to an other"""
    if type(node2) == Node:
        distance = (node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2
    elif type(node2)== graphics.Point:
        distance = (node1.x - node2.getX()) ** 2 + (node1.y - node2.getY()) ** 2
    return math.sqrt(distance)


def inside(point, rectangle):
    px = point.getX()
    py = point.getY()
    x2 = rectangle.getP1().getX()
    y2 = rectangle.getP1().getY()
    x1 = rectangle.getP2().getX()
    y1 = rectangle.getP2().getY()
    rx = None
    ry = None
    if x1 < px < x2 and y1 < py < y2:
        print("collision")
        return None
    elif x1 < px < x2:
        rx = px
    elif y1 < py < y2:
        ry = py
    if rx is None:
        if abs(px - x1) < abs(px - x2):
            rx = x1
        else:
            rx = x2
    if ry is None:
        if abs(py - y1) < abs(py - y2):
            ry = y1
        else:
            ry = y2
    return graphics.Point(rx, ry)                      # TODO: itt jól betekeri magát apa


def build_ring():
    # felső _
    pt2 = graphics.Point(0, 0)
    pt1 = graphics.Point(SIZEX, 0)
    rect = graphics.Rectangle(pt1, pt2)
    rect.setFill("black")
    rect.draw(WINDOW)
    OBSTACLES.append(rect)

    # bal |
    pt2 = graphics.Point(0, 0)
    pt1 = graphics.Point(0, SIZEY)
    rect1 = graphics.Rectangle(pt1, pt2)
    rect1.setFill("black")
    rect1.draw(WINDOW)
    OBSTACLES.append(rect1)

    # alsó _
    pt1 = graphics.Point(SIZEX, SIZEY)
    pt2 = graphics.Point(0, SIZEY+10)
    rect2 = graphics.Rectangle(pt1, pt2)
    rect2.setFill("black")
    rect2.draw(WINDOW)
    OBSTACLES.append(rect2)

    # jobb |
    pt2 = graphics.Point(SIZEX, 0)
    pt1 = graphics.Point(SIZEX, SIZEY)
    rect3 = graphics.Rectangle(pt1, pt2)
    rect3.setFill("black")
    rect3.draw(WINDOW)
    OBSTACLES.append(rect3)


class Data():
    pass


class Obstacle():

    def __init__(self, x, y):
        if VERTICAL:
            pt1 = graphics.Point(x+10, y+100)
            pt2 = graphics.Point(x-10, y-100)
        else:
            pt1 = graphics.Point(x + 100, y + 10)
            pt2 = graphics.Point(x - 100, y - 10)
        rect = graphics.Rectangle(pt1,pt2)
        rect.setFill("black")
        rect.draw(WINDOW)
        OBSTACLES.append(rect)


class Vector:
    """Vector to know the directions"""
    def __init__(self, x2=0, y2=0, x1=0, y1=0):
        self.x = x2 - x1
        self.y = y2 - y1


class Node:
    """represents the points / nodes of the map"""
    direction = Vector(0, 0, 0, 0)
    parent = None
    g = 0
    h = 0
    f = 0

    def calc_cost(self, reference):
        if self.parent is not None:
            self.g = self.parent.g + self.parent.circle.r                               # g : distance from the start node
        else:
            self.g = self.circle.r
        self.h = calc_distance(self, reference) - (self.circle.r + reference.circle.r)  # h : heuristic distance to the goal
        self.f = self.g + self.h                                        # f : total cost = g + h

    def __init__(self, x, y, r=RADIUS):
        if 0 < x < SIZEX and 0 < y < SIZEY and 5 < r:
            self.x = x
            self.y = y
            self.circle = Circle(x, y, r)
        else:
            raise ValueError


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


class Explore:
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

    def get_r_free(self, x, y):
        point = graphics.Point(x, y)
        min_dist = max(SIZEX, SIZEY)
        for obstacle in OBSTACLES:
            reference = inside(point, obstacle)
            if type(reference) == graphics.Point:
                distance = calc_distance(reference,point)
                distance -= CARSIZE
                if distance < min_dist:
                    min_dist = distance
            else:
                min_dist = -1
            if min_dist < 5:
                min_dist = -1
        return min_dist

    def expand(self, node):
        global CIRCLES_2_DRAW
        for i in range(0, SAMPLE):
            new_nodes = []
            alpha = (3.14 / 2) * (i / (SAMPLE - 1))
            dy = node.circle.r * math.sin(alpha)
            dx = node.circle.r * math.cos(alpha)

            x1 = node.circle.x + dx
            y1 = node.circle.y + dy
            try:
                node_new1 = Node(x1, y1, self.get_r_free(x1, y1))
                node_new1.direction = Vector(dx, dy)
                new_nodes.append(node_new1)
            except ValueError:
                pass

            x2 = node.circle.x - dx
            y2 = node.circle.y + dy
            try:
                node_new2 = Node(x2, y2, self.get_r_free(x2, y2))
                node_new2.direction = Vector(-dx, dy)
                new_nodes.append(node_new2)
            except ValueError:
                pass

            x3 = node.circle.x + dx
            y3 = node.circle.y - dy
            try:
                node_new3 = Node(x3, y3, self.get_r_free(x3, y3))
                node_new3.direction = Vector(dx, -dy)
                new_nodes.append(node_new3)
            except ValueError:
                pass

            x4 = node.circle.x - dx
            y4 = node.circle.y - dy
            try:
                node_new4 = Node(x4, y4, self.get_r_free(x4, y4))
                node_new4.direction = Vector(-dx, -dy)
                new_nodes.append(node_new4)
            except ValueError:
                pass

            for index, n in reversed(list(enumerate(new_nodes))):
                n.parent = node

                n.calc_cost(self.n_goal)
                for element in self.NODES:
                    if element.x == n.x and element.y == n.y :  # TODO: nem tudom hogy itt kell e gyorsítani de próbának jó lesz
                        new_nodes.pop(index)
                        break
                    elif calc_distance(element, n) < element.circle.r:
                        new_nodes.pop(index)
                        break
            self.S_open += new_nodes
            CIRCLES_2_DRAW += new_nodes
            self.NODES += new_nodes

        # drawing the circles to the window

        for index, node in reversed(list(enumerate(CIRCLES_2_DRAW))):
            #time.sleep(0.1)
            node.circle.draw_circle().setOutline("purple")
            CIRCLES_2_DRAW.pop(index)

    def not_exist(self, node):
        not_find = True
        for finished in self.S_closed:
            if node == finished:
                not_find = False
                break
        return not_find

    def pop_nearest(self):                       # TODO: Itt mit is keresünk? f-et? az így nagyon elment?
        """pop from the S_open the nearest node to the goal, to be evaluated"""
        # TODO: Geci lassu
        min_index = 0
        min_dist = self.S_open[0].f
        for index, node in enumerate(self.S_open,0):
            if node.f < min_dist:
                min_index = index
                min_dist = node.f
        return self.S_open.pop(min_index)

    def overlap(self,node1,node2):
        if calc_distance(node1, node2) <= (node1.circle.r + node2.circle.r):
            return True
        else:
            return False

    def recurse(self):
        circle_path = []
        node = self.n_goal
        while node != self.n_start:
            circle_path.append(node)
            parent = node.parent
            pt1 = graphics.Point(parent.x,parent.y)
            pt2 = graphics.Point(node.x,node.y)
            ln = graphics.Line(pt1, pt2)
            ln.setWidth(10)
            ln.setOutline('orange')
            ln.draw(WINDOW)
            node = parent
        return circle_path

    def space_exploration(self):
        # TODO: here to start
        circle_path = []
        while self.S_open:
            node= self.pop_nearest()
            node.circle.draw_circle().setOutline("red")
            if self.n_goal.parent is not None:          # TODO: valahogy ellenőrizzük ha ott vagyunk a célnál IDK if it's good
                circle_path = self.recurse()
                return True, circle_path

            elif self.not_exist(node):
                self.expand(node)
                if self.overlap(node, self.n_goal):
                    self.n_goal.g = node.f
                    self.n_goal.parent = node
                self.S_closed.append(node)
        return False, circle_path


def main():
    global VERTICAL
    build_ring()
    pressed = None
    while pressed !="Return":
        pressed = WINDOW.checkKey()
        cin_cin = WINDOW.checkMouse()
        if pressed == "space":
            VERTICAL = not VERTICAL
        if cin_cin!= None:
            click = WINDOW.getMouse()
            Obstacle(click.x, click.y)
        print(pressed)
    start = Node(BORDER, BORDER)
    goal = Node(SIZEX-BORDER, SIZEY-BORDER)
    test = Explore(start, goal)
    path_found, circle_path = test.space_exploration()
    if path_found:
        print("DONE:", circle_path)
    else:
        print("Path doesn't exist")
    while True:
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
