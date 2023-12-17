import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

def main():
    pointlist = [[0.0, 0.1, 0.2]]

    pointcloud = pc2.create_cloud_xyz32(Header(frame_id='frame'), pointlist)

    for point in pc2.read_points(pointcloud):
        print(point)
        

if __name__ == '__main__':
    main()