import numpy as np
import matplotlib.pyplot as plt
import rosbag
import tf

bag = rosbag.Bag('bag_output_ukf_polban0807_02.bag')

rpy = []
for topic, msg, t in bag.read_messages(topics=['/odometry/filtered_map']):
    q = msg.pose.pose.orientation
    euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    rpy.append(euler)
rpy = np.array(rpy)

plt.figure()
plt.plot(rpy[:,-1])
plt.show()
