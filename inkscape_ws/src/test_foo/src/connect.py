"""
For a provided image, it determines where the connected edges are.

@author Shashank Swaminathan
"""

import numpy as np

class Connect:
    """
    Class to encapsulate connected edge finder
    """
    def __init__(self, img):
        """
        Grabs initial data about the image, and initializes attributes.

        :param img: Image to find connected edges in. Assumes that edges are where pixels are 255.
        """
        self.img = img
        self.n_rows=self.img.shape[0]
        self.n_cols=self.img.shape[1]
        self.center=[self.n_rows/2,self.n_cols/2]
        self.connect_list = np.zeros(self.img.shape)

    def grab_edges(self):
        """
        Performs DFS on the image to find connected edges.

        :returns: A matrix of the edge points in the image, organized by which edge they belong to, and radially within the edge (so that the points are arranged counter-clockwise around the edge).
        """
        self._find_connects() # Do DFS
        self.connected_edge_list = [] # Init list of points on edge
        for i in range(self.n_rows):
            for j in range(self.n_cols):
                if (self.img[i][j] == 255):
                    # If on edge, append important info about point to list
                    self.connected_edge_list.append(self._get_info_ro(i,j))

        # Sort list first based on which edge group it belongs to
        # Then sort based on where it is radially
        self.connected_edge_list = np.array(self.connected_edge_list)
        self.connected_edge_list.view('i8,i8,i8,i8,i8').sort(order=['f4','f3'], axis=0)
        return self.connected_edge_list[:,[0,1,4]]

    def _find_connects(self):
        """
        Helper function that handles the DFS to identify which edges exist. Updates every point in the image to belong to an edge group of connected points.
        """
        dx8 = [-1,0,1,1,1,0,-1,-1]
        dy8 = [1,1,1,0,-1,-1,-1,0]
        dx4 = [0,1,0,-1]
        dy4 = [1,0,-1,0]
        conn_num = 1

        for i in range(self.n_rows):
            for j in range(self.n_cols):
                if (self.img[i][j]==255 and self.connect_list[i][j] == 0):
                    self._recurse_connect(dx8,dy8,i,j,conn_num)
                    conn_num = conn_num + 1

    def _get_info_ro(self,i,j):
        """
        Helper function that gets the polar coordinate of the given point in the matrix. (Because the matrix of img data directly correlates to a (x,y) coordinate frame).

        :param i: Row index of point
        :param j: Column index of point

        :returns: Tuple of following: i, j, r, theta, edge group of point
        """
        x=i-self.center[0]
        y=j-self.center[1]
        r=np.sqrt(x**2+y**2)
        o=np.arctan2(y,x)
        if o < 0:
            o=o+np.pi*2
        return i,j,r,o,self.connect_list[i][j]

    def _recurse_connect(self,dx,dy,i,j,c):
        """
        Helper functon to recursively find the edges connected to the given point in the image. Update the edge group every connected point belongs to.

        :param dx: Vector of the relative row-indices of all connected points
        :param dy: Vector of the relative column-indices of all connected points
        :param i: Row index of point
        :param j: Column index of point
        :param c: The current edge group that the function is recursing over
        """
        self.connect_list[i][j]=c
        for k in range(len(dx)):
            ni=i+dx[k]
            nj=j+dy[k]
            if (self.img[ni][nj] == 255 and self.connect_list[ni][nj] == 0):
                self._recurse_connect(dx,dy,ni,nj,c)

# Test main method
if __name__ == "__main__":
    img = np.array([[0,  0,  0,  0,  0,  0,0],
                    [0,255,255,255,255,255,0],
                    [0,  0,  0,  0,  0,  0,0],
                    [0,255,  0,  0,  0,255,0],
                    [0,255,  0,  0,  0,255,0],
                    [0,  0,255,255,255,  0,0],
                    [0,  0,  0,  0,  0,  0,0]])
    c = Connect(img=img)
    print(c.connect_list)
    c.grab_edges()
    print(c.connect_list)
    print(c.connected_edge_list)
