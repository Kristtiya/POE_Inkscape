#!/usr/bin/env python

import numpy as np
# import cv2
import matplotlib.pyplot as plt

class Connect:
    def __init__(self, img):
        self.img = img
        self.n_rows=self.img.shape[0]
        self.n_cols=self.img.shape[1]
        self.center=[self.n_rows/2,self.n_cols/2]
        self.connect_list = np.zeros(self.img.shape)

    def grab_edges(self):
        self._find_connects()
        self.connected_edge_list = []
        for i in range(self.n_rows):
            for j in range(self.n_cols):
                if (self.img[i][j] == 255):
                    self.connected_edge_list.append(self._get_info_ro(i,j))

        self.connected_edge_list = np.array(self.connected_edge_list)
        self.connected_edge_list.view('i8,i8,i8,i8,i8').sort(order=['f4','f3'], axis=0)
        return self.connected_edge_list[:,[0,1,4]]

    def _find_connects(self):
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
        x=i-self.center[0]
        y=j-self.center[1]
        r=np.sqrt(x**2+y**2)
        o=np.arctan2(y,x)
        if o < 0:
            o=o+np.pi*2
        return i,j,r,o,self.connect_list[i][j]

    def _recurse_connect(self,dx,dy,i,j,c):
        self.connect_list[i][j]=c
        for k in range(len(dx)):
            ni=i+dx[k]
            nj=j+dy[k]
            if (self.img[ni][nj] == 255 and self.connect_list[ni][nj] == 0):
                self._recurse_connect(dx,dy,ni,nj,c)

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
