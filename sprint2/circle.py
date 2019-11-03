import cv2
import numpy as np
from matplotlib import pyplot as plt

def get_edges(img):
    edges = cv2.Canny(img, 100, 200)

    return edges

def display(edges, img):
    plt.subplot(121)
    plt.imshow(img, cmap='gray')
    plt.title("Original Image")
    plt.xticks([])
    plt.yticks([])

    plt.subplot(122)
    plt.imshow(edges, cmap='gray')
    plt.title("Edge Image")
    plt.xticks([])
    plt.yticks([])

    plt.show()

def get_white(edges):
    white = []

    for i in range(0, len(edges)):
        for j in range(0, len(edges[i])):
            if edges[i][j] == 255:
                white.append([i, j])
    
    return np.array(white)

def distance(point):
    return np.sqrt(point[0]**2+point[1]**2)

def sort_points(white):
    distances = np.apply_along_axis(distance, axis=1, arr=white)
    indices = np.arange(0, white.shape[0])
    combined = np.transpose(np.vstack((distances, indices)))
    sorted_pts = combined[combined[:, 0].argsort()]
    sorted_indeces = sorted_pts[:, 1].tolist()
    points = [white[int(idx)] for idx in sorted_indeces]

    return points

def img_to_points(filename):
    img = cv2.imread(filename, 0)
    img = cv2.bitwise_not(img)
    edges = get_edges(img)
    white_edges = get_white(img)
    pts = sort_points(white_edges)

    return pts

if __name__ == "__main__":
    points = img_to_points("circle.png")
    print(points)