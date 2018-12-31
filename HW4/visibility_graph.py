import math

def has_intersect_horiz(pt1, pt2, y, x1, x2):
    # find slope
    slope = float("inf")
    if pt1[0] != pt2[0]:
        slope = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
    # if equal to 0, no collision
    if slope == 0:
        return False
    # find intersecting x coordinate
    x_intersect = None
    if slope == float("inf"):
        x_intersect = pt1[0]
    else:
        b = pt1[1] - slope * pt1[0]
        x_intersect = (y - b)/slope
    # check that intersect is between x1 and x2 and pt1 and pt2 lie on opposite sides of line
    if x_intersect > x1 and x_intersect < x2:
        if (pt1[1] < y and pt2[1] > y) or (pt1[1] > y and pt2[1] < y):
            return True

    return False

def has_intersect_vert(pt1, pt2, x, y1, y2):
    # find slope
    slope = float("inf")
    if pt1[0] != pt2[0]:
        slope = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
    # if equal to infinity, no collision
    if slope == float("inf"):
        return False
    # find intersecting y coordinate
    y_intersect = None
    if slope == 0:
        y_intersect = pt1[1]
    else:
        b = pt1[1] - slope * pt1[0]
        y_intersect = slope * x + b
    # check that intersect is between y1 and y2 and pt1 and pt2 lie on opposite sides of line
    if y_intersect > y1 and y_intersect < y2:
        if (pt1[0] < x and pt2[0] > x) or (pt1[0] > x and pt2[0] < x):
            return True

    return False


# returns whether a corner is visible to a point
def visible(mypoint, mycorner, corner_desc):
    # calculate angle
    myangle = math.atan2(mypoint[1] - mycorner[1], mypoint[0] - mycorner[0])
    
    # allowed angles are between 0 and 270 degrees
    if corner_desc == "topleft":
        myangle = myangle % (2 * math.pi)   # force in range [0, 2 pi)
        if myangle >= 0 and myangle <= math.pi*3/2:
            return True
    
    # allowed angles are between -90 and 180 degrees
    if corner_desc == "topright":
        myangle = myangle % (2 * math.pi)   # force in range [0, 2 pi)
        if myangle > math.pi:               # move to [-pi, pi)
            myangle -= 2 * math.pi
        if myangle >= -math.pi/2 and myangle <= math.pi:
            return True

    # allowed angles are between 90 and 360 degrees
    elif corner_desc == "botleft":
        myangle = myangle % (2 * math.pi)   # force in range [0, 2 pi)
        if myangle == 0:
            myangle = 2 * math.pi           # force in range (0, 2 pi]
        if myangle >= math.pi/2 and myangle <= math.pi*2:
            return True
    
    # allowed angles are between -180 and 90 degrees
    elif corner_desc == "botright":
        myangle = myangle % (2 * math.pi)   # force in range [0, 2 pi)
        if myangle >= math.pi:               # move to (-pi, pi]
            myangle -= 2 * math.pi
        if myangle >= -math.pi and myangle <= math.pi/2:
            return True

    return False

class node():
    def __init__(self, xcoord, ycoord):
        self.x = xcoord
        self.y = ycoord
        self.edges = []
    
    def get_coords(self):
        return [self.x, self.y]

    def get_dist(self, mynode):
        pt1 = self.get_coords()
        pt2 = mynode.get_coords()
        return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)
    
    def add_edge(self, mynode):
        self.edges.append(mynode)

class map():
    def __init__(self, topleft, topright, botleft, botright, padding):
        # pad the rectangle
        self.topleft = node(topleft[0] - padding, topleft[1] + padding)
        self.topright = node(topright[0] + padding, topright[1] + padding)
        self.botleft = node(botleft[0] - padding, botleft[1] - padding)
        self.botright = node(botright[0] + padding, botright[1] - padding)
    
    def add_start(self, xcoord, ycoord):
        self.start = node(xcoord, ycoord)
    
        if visible(self.start.get_coords(), self.topleft.get_coords(), "topleft"):
            self.start.add_edge(self.topleft)
            self.topleft.add_edge(self.start)
        if visible(self.start.get_coords(), self.topright.get_coords(), "topright"):
            self.start.add_edge(self.topright)
            self.topright.add_edge(self.start)
        if visible(self.start.get_coords(), self.botleft.get_coords(), "botleft"):
            self.start.add_edge(self.botleft)
            self.botleft.add_edge(self.start)
        if visible(self.start.get_coords(), self.botright.get_coords(), "botright"):
            self.start.add_edge(self.botright)
            self.botright.add_edge(self.start)
    
    def add_end(self, xcoord, ycoord):
        self.end = node(xcoord, ycoord)
    
        if visible(self.end.get_coords(), self.topleft.get_coords(), "topleft"):
            self.end.add_edge(self.topleft)
            self.topleft.add_edge(self.end)
        if visible(self.end.get_coords(), self.topright.get_coords(), "topright"):
            self.end.add_edge(self.topright)
            self.topright.add_edge(self.end)
        if visible(self.end.get_coords(), self.botleft.get_coords(), "botleft"):
            self.end.add_edge(self.botleft)
            self.botleft.add_edge(self.end)
        if visible(self.end.get_coords(), self.botright.get_coords(), "botright"):
            self.end.add_edge(self.botright)
            self.botright.add_edge(self.end)

        # check if end is visible from start
        topl = self.topleft.get_coords()
        topr = self.topright.get_coords()
        botl = self.botleft.get_coords()
        botr = self.botright.get_coords()
        # check collision with top line
        if has_intersect_horiz(self.start.get_coords(), self.end.get_coords(), topl[1], topl[0], topr[0]):
            return
        # check collision with bottom line
        if has_intersect_horiz(self.start.get_coords(), self.end.get_coords(), botl[1], botl[0], botl[0]):
            return
        # check collision with left line
        if has_intersect_vert(self.start.get_coords(), self.end.get_coords(), botl[0], botl[1], topl[1]):
            return
        # check collision with right line
        if has_intersect_vert(self.start.get_coords(), self.end.get_coords(), botr[0], botr[1], topr[1]):
            return
        self.end.add_edge(self.start)
        self.start.add_edge(self.end)
    
    # runs Dijkstra's algorithm on nodes
    def find_shortest_path(self):
        all_nodes = []
        all_nodes.append(self.start)
        all_nodes.append(self.end)
        all_nodes.append(self.topleft)
        all_nodes.append(self.topright)
        all_nodes.append(self.botleft)
        all_nodes.append(self.botright)

        dist = {}
        prev = {}
        unvisited = {}
        
        for mynode in all_nodes:
            dist[mynode] = float("inf")
            prev[mynode] = None
            unvisited[mynode] = 0
                
        dist[self.start] = 0
        
        while len(unvisited) > 0:
            # find node with smallest distance in unvisited
            smallest_dist = float("inf")
            curr_node = None
            for mynode in unvisited:
                if dist[mynode] < smallest_dist:
                    smallest_dist = dist[mynode]
                    curr_node = mynode
        
            # check if you found the end node
            if curr_node == self.end:
                myreturn = []
                while prev[curr_node] != None:
                    myreturn.append(curr_node.get_coords())
                    curr_node = prev[curr_node]
                return myreturn[::-1]
        
            # remove node from unvisited
            del unvisited[curr_node]
            # visit all of its neighbors
            for mynode in curr_node.edges:
                if mynode in unvisited: # check mynode is in unvisited
                    alt = dist[curr_node] + curr_node.get_dist(mynode)
                    if alt < dist[mynode]:
                        dist[mynode] = alt
                        prev[mynode] = curr_node

def main():
    rect_corners = [[0.39,0.51], [0.7,0.51], [0.39,0.2], [0.7,0.2]]
    car_width = 0.13 # measured 13 cm for width of PiCar
    
    # my_map1 for going from start to first stop
    my_map1 = map(rect_corners[0], rect_corners[1], rect_corners[2], rect_corners[3], car_width)
    my_map1.add_start(0.0,0.0)
    my_map1.add_end(1.05,0.38)
    print ("Path from start to first stop:")
    print (my_map1.find_shortest_path())

    # my_map2 for going from first stop to second stop
    my_map2 = map(rect_corners[0], rect_corners[1], rect_corners[2], rect_corners[3], car_width)
    my_map2.add_start(1.05,0.38)
    my_map2.add_end(0.5,0.8)
    print ("Path from first stop to second stop:")
    print (my_map2.find_shortest_path())

if __name__ == "__main__":
    main()
