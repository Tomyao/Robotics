import matplotlib.pyplot as plt

def main():
    rect_corners = [[0.39,0.51], [0.7,0.51], [0.7,0.2], [0.39,0.2], [0.39,0.51]]
    rect_corners_padded = [[39-13,51+13], [39-13,20-13], [70+13,20-13], [70+13,51+13], [39-13, 51+13]]
    for a in rect_corners_padded:
        a[0] = a[0]/100
        a[1] = a[1]/100
    room_corners = [[-0.44,1.32],[1.48,1.32],[1.48,-0.585], [-0.44,-0.585], [-0.44,1.32]]
    start = [0.0,0.0]
    endpoint1 = [1.05,0.38]
    endpoint2 = [0.5,0.8]
    voronoi_path = [[0.0335, -0.1115], [0.39, -0.1925], [0.7, -0.1925], [1.0216, -0.1266], [1.09, 0.2], [1.05, 0.38], [1.09, 0.51], [1.0141, 0.8541], [0.7, 0.915], [0.5, 0.8]]
    visibility_path = [[0.83,0.07], [1.05,0.38], [0.83,0.64], [0.5,0.8]]
    voronoi_graph = [[0.0404, 0.8396], [0.39, 0.915], [0.7, 0.915], [1.0141, 0.8541], [1.09, 0.51], [1.09, 0.2], [1.0216, -0.1266], [0.7, -0.1925], [0.39, -0.1925], [0.0335, -0.1115], [-0.025, 0.2], [-0.025, 0.51], [0.0404, 0.8396]]
    
    # draw obstacle and boundaries
    xplot = []
    yplot = []
    for a in rect_corners:
        xplot.append(a[0])
        yplot.append(a[1])
    plt.plot(xplot, yplot, color ='r')

    xplot = []
    yplot = []
    for a in room_corners:
        xplot.append(a[0])
        yplot.append(a[1])
    plt.plot(xplot, yplot, color ='b')
    '''
    # draw padded obstacle
    xplot = []
    yplot = []
    for a in rect_corners_padded:
        xplot.append(a[0])
        yplot.append(a[1])
    plt.plot(xplot, yplot, color ='orange')
    '''
    # draw start, endpoint 1 and 2
    xplot = [start[0], endpoint1[0], endpoint2[0]]
    yplot = [start[1], endpoint1[1], endpoint2[1]]
    wordplot = ["start", "endpoint 1", "endpoint 2"]
    plt.scatter(xplot, yplot, color = 'black')
    plt.text(xplot[0]-0.2, yplot[0]-0.02, wordplot[0], fontsize=8)
    plt.text(xplot[1]+0.05, yplot[1]-0.02, wordplot[1], fontsize=8)
    plt.text(xplot[2]-0.4, yplot[2]-0.02, wordplot[2], fontsize=8)
    '''
    # draw voronoi path
    xplot = [0.0]
    yplot = [0.0]
    for a in voronoi_path:
        xplot.append(a[0])
        yplot.append(a[1])
    plt.plot(xplot, yplot, color ='black')
    plt.scatter(xplot, yplot, s = 20, color = 'black')
    
    # draw visibility path
    xplot = [0.0]
    yplot = [0.0]
    for a in visibility_path:
        xplot.append(a[0])
        yplot.append(a[1])
    plt.plot(xplot, yplot, color ='black')
    plt.scatter(xplot, yplot, s = 20, color = 'black')
    
    # draw voronoi graph
    xplot = []
    yplot = []
    for a in voronoi_graph:
        xplot.append(a[0])
        yplot.append(a[1])
    plt.plot(xplot, yplot, color ='black')
    plt.scatter(xplot, yplot, s = 20, color = 'black')
    xplot = []
    yplot = []
    xplot.append(voronoi_graph[0][0])
    yplot.append(voronoi_graph[0][1])
    xplot.append(room_corners[0][0])
    yplot.append(room_corners[0][1])
    plt.plot(xplot, yplot, color ='black')
    xplot = []
    yplot = []
    xplot.append(voronoi_graph[3][0])
    yplot.append(voronoi_graph[3][1])
    xplot.append(room_corners[1][0])
    yplot.append(room_corners[1][1])
    plt.plot(xplot, yplot, color ='black')
    xplot = []
    yplot = []
    xplot.append(voronoi_graph[6][0])
    yplot.append(voronoi_graph[6][1])
    xplot.append(room_corners[2][0])
    yplot.append(room_corners[2][1])
    plt.plot(xplot, yplot, color ='black')
    xplot = []
    yplot = []
    xplot.append(voronoi_graph[9][0])
    yplot.append(voronoi_graph[9][1])
    xplot.append(room_corners[3][0])
    yplot.append(room_corners[3][1])
    plt.plot(xplot, yplot, color ='black')
    '''

    # draw visibility graph
    xplot = []
    yplot = []
    for a in rect_corners_padded:
        xplot.append(a[0])
        yplot.append(a[1])
    plt.plot(xplot, yplot, color ='black')
    plt.scatter(xplot, yplot, s = 20, color = 'black')
    xplot = []
    yplot = []
    xplot.append(start[0])
    yplot.append(start[1])
    xplot.append(rect_corners_padded[0][0])
    yplot.append(rect_corners_padded[0][1])
    plt.plot(xplot, yplot, color ='black')
    xplot = []
    yplot = []
    xplot.append(start[0])
    yplot.append(start[1])
    xplot.append(rect_corners_padded[1][0])
    yplot.append(rect_corners_padded[1][1])
    plt.plot(xplot, yplot, color ='black')
    xplot = []
    yplot = []
    xplot.append(start[0])
    yplot.append(start[1])
    xplot.append(rect_corners_padded[2][0])
    yplot.append(rect_corners_padded[2][1])
    plt.plot(xplot, yplot, color ='black')
    xplot = []
    yplot = []
    xplot.append(endpoint1[0])
    yplot.append(endpoint1[1])
    xplot.append(rect_corners_padded[2][0])
    yplot.append(rect_corners_padded[2][1])
    plt.plot(xplot, yplot, color ='black')
    xplot = []
    yplot = []
    xplot.append(endpoint1[0])
    yplot.append(endpoint1[1])
    xplot.append(rect_corners_padded[3][0])
    yplot.append(rect_corners_padded[3][1])
    plt.plot(xplot, yplot, color ='black')
    xplot = []
    yplot = []
    xplot.append(endpoint2[0])
    yplot.append(endpoint2[1])
    xplot.append(rect_corners_padded[0][0])
    yplot.append(rect_corners_padded[0][1])
    plt.plot(xplot, yplot, color ='black')
    xplot = []
    yplot = []
    xplot.append(endpoint2[0])
    yplot.append(endpoint2[1])
    xplot.append(rect_corners_padded[3][0])
    yplot.append(rect_corners_padded[3][1])
    plt.plot(xplot, yplot, color ='black')

    # show plot!
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

if __name__ == "__main__":
    main()
