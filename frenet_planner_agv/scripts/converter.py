import rospy
from sensor_msgs.msg import PointCloud
import cv2
import numpy as np
import math
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull

viz = True

def draw_polygon(img, polygon):
    polygon_to_draw = np.zeros((polygon.shape))
    polygon_to_draw[:, 0] = (10*polygon[:, 0] + 300)
    polygon_to_draw[:, 1] = (300 - 10*polygon[:, 1])
    polygon_to_draw = polygon_to_draw.reshape((-1,1,2)).astype(np.int32)
    cv2.polylines(img,[polygon_to_draw],True,(255,0,0))

def callback(data):
    points = np.empty((len(data.ranges), 2), np.float64)

    if viz:
        img = 255*np.ones((600, 600, 3), np.uint8)

    for i, r in enumerate(data.ranges):
        theta = data.angle_min + i*data.angle_increment
        points[i][0] = r*math.sin(theta)
        points[i][1] = r*math.cos(theta)

        if viz:
            cv2.circle(img, (int(300+10*r*math.sin(theta)), int(300-10*r*math.cos(theta))), 2, (128, 128, 128), -1)

    clustering = DBSCAN(eps=1, min_samples=1).fit(points)
    n_clusters = max(clustering.labels_) + 1
    obstacles = []

    for cluster_idx in range(n_clusters+1):
        cluster_points = points[clustering.labels_==cluster_idx, :]

        if cluster_points.shape[0] < 3:
            continue
            
        hull = ConvexHull(cluster_points)
        hull_points = cluster_points[hull.vertices, :]
        print(hull_points)
        obstacles.append(hull_points)

        if viz:
            draw_polygon(img, hull_points)

    if viz:
        cv2.imshow("obstacle map", img)
        cv2.waitKey(50)

    
def converter():
    rospy.init_node('converter', anonymous=True)
    rospy.Subscriber("/scan", PointCloud, callback)
    rospy.spin()

if __name__ == '__main__':
    converter()