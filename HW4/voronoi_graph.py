import math

# calculates where the voronoi curves would meet
def voronoi_intersect(room_corner, object_corner, corner_position):
    distance_x = abs(room_corner[0] - object_corner[0])
    distance_y = abs(room_corner[1] - object_corner[1])
    xoffset = math.sqrt(2*distance_y*distance_x) - distance_y
    yoffset = math.sqrt(2*distance_y*distance_x) - distance_x
    if corner_position == "topleft":
        return [object_corner[0] - xoffset, object_corner[1] + yoffset]
    elif corner_position == "topright":
        return [object_corner[0] + xoffset, object_corner[1] + yoffset]
    elif corner_position == "botleft":
        return [object_corner[0] - xoffset, object_corner[1] - yoffset]
    elif corner_position == "botright":
        return [object_corner[0] + xoffset, object_corner[1] - yoffset]

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
    # the variables with a 2 at the end indicate room corners as opposed to obstacle corners
    def __init__(self, topleft, topright, botleft, botright, topleft2, topright2, botleft2, botright2):
        self.all_nodes = []
        # generate all nodes in voronoi
        # starting from top left node and going clockwise
        temp = voronoi_intersect(topleft2, topleft, "topleft")
        self.all_nodes.append(node(temp[0], temp[1]))
        # top nodes
        self.all_nodes.append(node(topleft[0], (topleft[1] + topleft2[1])/2))
        self.all_nodes.append(node(topright[0], (topright[1] + topright2[1])/2))
        # top right node
        temp = voronoi_intersect(topright2, topright, "topright")
        self.all_nodes.append(node(temp[0], temp[1]))
        # right nodes
        self.all_nodes.append(node((topright[0] + topright2[0])/2, topright[1]))
        self.all_nodes.append(node((botright[0] + botright2[0])/2, botright[1]))
        # bottom right node
        temp = voronoi_intersect(botright2, botright, "botright")
        self.all_nodes.append(node(temp[0], temp[1]))
        # bottom nodes
        self.all_nodes.append(node(botright[0], (botright[1] + botright2[1])/2))
        self.all_nodes.append(node(botleft[0], (botleft[1] + botleft2[1])/2))
        # bottom left node
        temp = voronoi_intersect(botleft2, botleft, "botleft")
        self.all_nodes.append(node(temp[0], temp[1]))
        # left nodes
        self.all_nodes.append(node((botleft[0] + botleft2[0])/2, botleft[1]))
        self.all_nodes.append(node((topleft[0] + topleft2[0])/2, topleft[1]))
    
        # add in all the edges
        for i in range(len(self.all_nodes) - 1):
            self.all_nodes[i].add_edge(self.all_nodes[i+1])
            self.all_nodes[i+1].add_edge(self.all_nodes[i])
        # add in edges for first and last element
        self.all_nodes[0].add_edge(self.all_nodes[-1])
        self.all_nodes[-1].add_edge(self.all_nodes[0])
    
    def find_shortest_voronoi_path(self, start, end):
        temp_start = node(start[0], start[1])
        temp_end = node(end[0], end[1])
        
        start_dist = []
        end_dist = []
        for mynode in self.all_nodes:
            start_dist.append(temp_start.get_dist(mynode))
            end_dist.append(temp_end.get_dist(mynode))
        # find 2 closest nodes to start
        temp_start_dist = [start_dist.index(x) for x in sorted(start_dist)]
        start_close1 = self.all_nodes[temp_start_dist[0]]
        start_close2 = self.all_nodes[temp_start_dist[1]]
        # find 2 closest nodes to end
        temp_end_dist = [end_dist.index(x) for x in sorted(end_dist)]
        end_close1 = self.all_nodes[temp_end_dist[0]]
        end_close2 = self.all_nodes[temp_end_dist[1]]
        
        # try all 4 possibilities for shortest path
        # default is start_close1 and end_close1
        temp = self.find_shortest_path_length(start_close1, end_close1)
        smallest_dist = temp[1] + start_dist[temp_start_dist[0]] + end_dist[temp_end_dist[0]]
        myreturn = temp[0]
        # try start_close1 and end_close2
        temp = self.find_shortest_path_length(start_close1, end_close2)
        temp_dist = temp[1] + start_dist[temp_start_dist[0]] + end_dist[temp_end_dist[1]]
        if temp_dist < smallest_dist:
            smallest_dist = temp_dist
            myreturn = temp[0]
        # try start_close2 and end_close1
        temp = self.find_shortest_path_length(start_close2, end_close1)
        temp_dist = temp[1] + start_dist[temp_start_dist[1]] + end_dist[temp_end_dist[0]]
        if temp_dist < smallest_dist:
            smallest_dist = temp_dist
            myreturn = temp[0]
        # try start_close2 and end_close2
        temp = self.find_shortest_path_length(start_close2, end_close2)
        temp_dist = temp[1] + start_dist[temp_start_dist[1]] + end_dist[temp_end_dist[1]]
        if temp_dist < smallest_dist:
            smallest_dist = temp_dist
            myreturn = temp[0]

        myreturn.append(end)
        return myreturn
    
    # runs Dijkstra's algorithm on nodes
    def find_shortest_path_length(self, startnode, endnode):
        dist = {}
        prev = {}
        unvisited = {}
        
        for mynode in self.all_nodes:
            dist[mynode] = float("inf")
            prev[mynode] = None
            unvisited[mynode] = 0
                
        dist[startnode] = 0
        
        while len(unvisited) > 0:
            # find node with smallest distance in unvisited
            smallest_dist = float("inf")
            curr_node = None
            for mynode in unvisited:
                if dist[mynode] < smallest_dist:
                    smallest_dist = dist[mynode]
                    curr_node = mynode
        
            # check if you found the end node
            if curr_node == endnode:
                myreturn = []
                while prev[curr_node] != None:
                    myreturn.append(curr_node.get_coords())
                    curr_node = prev[curr_node]
                myreturn.append(curr_node.get_coords())
                return [myreturn[::-1], smallest_dist]
        
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
    room_corners = [[-0.44,1.32],[1.48,1.32],[-0.44,-0.585],[1.48,-0.585]]
    
    # my_map1 for going from start to first stop
    my_map1 = map(rect_corners[0], rect_corners[1], rect_corners[2], rect_corners[3], room_corners[0], room_corners[1], room_corners[2], room_corners[3])
    print ("Path from start to first stop:")
    mypath1 = my_map1.find_shortest_voronoi_path([0,0],[1.05,0.38])
    for point in mypath1:
        point[0] = round(point[0],4)
        point[1] = round(point[1],4)
    print (mypath1)

    # my_map2 for going from first stop to second stop
    my_map2 = map(rect_corners[0], rect_corners[1], rect_corners[2], rect_corners[3], room_corners[0], room_corners[1], room_corners[2], room_corners[3])
    print ("Path from first stop to second stop:")
    mypath2 = my_map2.find_shortest_voronoi_path([1.05,0.38],[0.5,0.8])
    for point in mypath2:
        point[0] = round(point[0],4)
        point[1] = round(point[1],4)
    print (mypath2)

if __name__ == "__main__":
    main()
