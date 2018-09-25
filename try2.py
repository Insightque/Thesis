import graphics
import math
import time

SIZEX = 1500
SIZEY = 900
BORDER = 50
RADIUS = (BORDER / 2) + 1
GOAL_RADIUS = RADIUS
WINDOW = graphics.GraphWin("Pathfind", SIZEX, SIZEY)
CIRCLES_2_DRAW = []
pi = 3.14
# SAMPLE = 5
SAMPLE = 5
OBSTACLES = []
CARSIZE = 0
VERTICAL = True
MAXIMAL_CURVE = pi/3  #degree
MINIMAL_RADIUS = 5
SPEED = 10          # m/s
MINIMAL_STEPSIZE = 100

def calc_distance(element1, element2):
    """calculating Eucledian distance from a node to an other"""
    if type(element2) == Node or type(element2 == State):
        distance = (element1.x - element2.x) ** 2 + (element1.y - element2.y) ** 2
    elif type(element2)== graphics.Point:
        distance = (element1.x - element2.getX()) ** 2 + (element1.y - element2.getY()) ** 2
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
    return graphics.Point(rx, ry)


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


def pop_top(S_open):
    """pop from S_open the point with minimal total cost, to be evaluated"""
    min_index = 0
    min_dist = S_open[0].f
    for index, element in enumerate(S_open,0):
        if element.f < min_dist:
            min_index = index
            min_dist = element.f
    return S_open.pop(min_index)


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
    next = None
    g = 0
    h = 0
    f = 0

    def calc_cost(self, reference):
        # dist2 = (self.parent.x - self.x)**2 + (self.parent.y - self.y)**2
        # self.g = math.sqrt(dist2)
        if self.parent is not None:
            self.g = self.parent.g + self.parent.circle.r                               # g : distance from the start node
        else:
            self.g = self.circle.r
        self.h = calc_distance(self, reference) - (self.circle.r + reference.circle.r)  # h : heuristic distance to the goal
        self.f = self.g + self.h                                        # f : total cost = g + h

    def __init__(self, x, y, r=RADIUS):
        if 0 < x < SIZEX and 0 < y < SIZEY and MINIMAL_RADIUS < r:
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
        self.n_goal.calc_cost(self.n_start)
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
            if min_dist < MINIMAL_RADIUS:
                min_dist = -1
        return min_dist

    def expand(self, node):
        global CIRCLES_2_DRAW
        for i in range(0, SAMPLE):
            new_nodes = []
            alpha = (pi / 2) * (i / (SAMPLE - 1))
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
                    if element.x == n.x and element.y == n.y and element.direction == n.direction:
                        new_nodes.pop(index)
                        break
                    elif calc_distance(element, n) < element.circle.r :
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
        """checking if the choosen node evaluated before"""
        not_find = True
        for finished in self.S_closed:
            if node == finished:
                not_find = False
                break
        return not_find


    def overlap(self,node1,node2):
        if calc_distance(node1, node2) <= (node1.circle.r + node2.circle.r):
            return True
        else:
            return False

    def recurse(self):
        path_from_end = []
        path = []
        node = self.n_goal
        while node != self.n_start:
            path_from_end.append(node)
            parent = node.parent
            pt1 = graphics.Point(parent.x,parent.y)
            pt2 = graphics.Point(node.x,node.y)
            ln = graphics.Line(pt1, pt2)
            ln.setWidth(10)
            ln.setOutline('orange')
            ln.draw(WINDOW)
            node = parent
        for index,x in enumerate(reversed(path_from_end)):
            if  0 < index < len(path_from_end):
                path[index-1].next = x
                if index == len(path_from_end)-1:
                    x.next = self.n_goal
            path.append(x)

        return path

    def space_exploration(self):
        circle_path = []
        while self.S_open:
            node = pop_top(self.S_open)
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


class State():
    x = 0
    y = 0
    fi = 0
    node: Node = None
    parent = None
    f = 0
    g = 0
    h = 0

    def __init__(self, x, y, fi = 0):
        self.x = x
        self.y = y
        self.fi = fi
        graphics.Point(x, y).draw(WINDOW)
        
    def calc_cost(self, reference, node):
        # the total cost for the state
        if self.parent is not None:
            self.g = self.parent.g + calc_distance(self, self.parent)         # g : distance from the start node
        else:
            pass
        self.g = 0

        self.h = calc_distance(self, node.next) + node.h
        self.f = self.g + self.h                                              # f : total cost = g + h


class Search:

    S_closed = []
    S_open = []

    def __init__(self, Start, Goal, circles):
        self.q_start: State = Start
        self.S_open.append(self.q_start)
        self.q_goal: State = Goal
        self.circle_path = circles
        self.q_start.node = self.map_nearest(self.q_start)
        # self.q_start.calc_cost(self.q_goal)


    def map_nearest(self, qi):
        """a választott q konfigurációt hozzápárosítja a legközelebbi ci körhöz""""ó"
        min_index = 0
        min_dist = calc_distance(qi, self.circle_path[0])
        for index, ci in enumerate(self.circle_path, 0):
            distance = calc_distance(qi, ci)
            if distance < min_dist:
                min_index = index
                min_dist = distance
        self.circle_path[min_index].circle.draw_circle().setOutline("green")
        return self.circle_path[min_index]

    def not_exist(self, qi: State):
        """Checking if the next state is evaluated before"""
        not_find = True
        for state in self.S_closed:
            if state.x == qi.x and state.y == qi.y and state.fi == qi.fi \
                    and state.node == qi.node and state.f == qi.f:
                not_find = False
        return not_find

    def expand(self, state):
        A = 0.33
        B = 0.33
        """
        primitív mozgásokra hajtódik végre.
        új configurácót generál q-ból s lépésközzel
        A megfelelő s lépésköz megválasztásában a kör mérete segít
        s = min(A*rc,B*dc,smin)
        """
        radius =state.node.circle.r
        distance = calc_distance(state, self.q_goal)
        step = min(A*radius, B*distance, MINIMAL_STEPSIZE)
        dfi = MAXIMAL_CURVE

        for i in range(2):
            dx = step * math.sin(state.fi)
            dy = step * math.cos(state.fi)
            new1 = State(state.x + dx, state.y + dy, state.fi)
            new2 = State(state.x - dx, state.y - dy, state.fi)

            dx = step * math.sin(state.fi + dfi)
            dy = step * math.cos(state.fi + dfi)
            new3 = State(state.x + dx, state.y + dy, state.fi + dfi)
            new4 = State(state.x - dx, state.y - dy, -state.fi + dfi)

            dx = step * math.sin(state.fi - dfi)
            dy = step * math.cos(state.fi - dfi)
            new5 = State(state.x + dx, state.y + dy, state.fi-dfi)
            new6 = State(state.x - dx, state.y - dy, -state.fi-dfi)
        #TODO: teszt és folytatás.... ---> mappelt körrel kell még valamit kezdeni
            new_states = [new1, new2, new3, new4, new5, new6]

            for index, q in reversed(list(enumerate(new_states))):
                # if calc_distance(q, state.node) > state.node.circle.r and calc_distance(q, state.node.next) > state.node.next.circle.r:
                #     new_states.pop(index)
                #     continue
                q.parent = state
                q.calc_cost(self.q_goal, state.node)


                pt1 = graphics.Point(q.x, q.y)
                pt2 = graphics.Point(state.x, state.y)
                ln = graphics.Line(pt1, pt2)
                ln.draw(WINDOW)

            self.S_open += new_states
            dfi = dfi / 2

    def goal_expand(self,state):
        print ("Reed Shepp")

    def heuristic_search(self):
        # amíg van új, bejárandó körök
        while len(self.S_open) != 0:
            # time.sleep(0.1)
            qi: State = pop_top(self.S_open)

            if calc_distance(qi, self.q_goal) < GOAL_RADIUS:
                return True

            # ha ilyen (minimal total cost) nincs a bejárt körök közt
            else:
                # minden iterációs lépésben a lekissebb összköltségű ci lesz kiválasztva
                qi.node = self.map_nearest(qi)
                if self.not_exist(qi):
    #TODO: to be contuined....
                    # gyerek állapotok (új) kerülnek hozzáadásra az S_open-hez
                    # és a szülő átkerül S_closed-ba
                    self.expand(qi)

                # ha ci átfedésben van c_goal-lal
                # total_cost update-elődik
                # amíg goal_cost < local cost of any S_open --> repeat algorithm
                if qi.h < GOAL_RADIUS and qi != self.q_start:
                    self.goal_expand(qi)
                    return True
                self.S_closed.append(qi)
        return False

    def recurse(self):
        path = []
        state = self.q_goal
        while state != self.q_start:
            path.append(state)
            parent = state.parent
            pt1 = graphics.Point(parent.x, parent.y)
            pt2 = graphics.Point(state.x, state.y)
            ln = graphics.Line(pt1, pt2)
            ln.setWidth(10)
            ln.setOutline('orange')
            ln.draw(WINDOW)
            state = parent
        return path

def main():
    global VERTICAL

    # setting up the playground
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

    # exploring the space
    start = Node(BORDER, BORDER)
    goal = Node(SIZEX-BORDER, SIZEY-BORDER, GOAL_RADIUS)
    test1 = Explore(start, goal)
    explored, circle_path = test1.space_exploration()
    if explored:
        print("DONE:", circle_path)
       # time.sleep(2)
        #WINDOW.delete("all")
        WINDOW.update()

        # starting heuristic search with the circle path
        start_state = State(BORDER, BORDER)
        goal_state = State(SIZEX-BORDER, SIZEY-BORDER, GOAL_RADIUS)
        test2 = Search(start_state, goal_state, circle_path)
        searched = test2.heuristic_search()
        if searched:
            print("Search DONE!!!")
        else:
            print("motion planning failed")
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
