import rospy
import torch
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
 
model = torch.jit.load('convmodel.pt')
model.eval()

result = None
counter = 0
min_counts = 0

def laser_callback(data):
    global model, result, counter, min_counts
    with torch.no_grad():
        ranges = torch.tensor(data.ranges)[None, :]
        mresult = model(ranges)
        mresult = torch.sigmoid(mresult)
        if mresult != result and counter > min_counts:
             result = mresult
             counter = 0
        counter += 1
        print(mresult, result, counter)

def main():
    global result
    pub = rospy.Publisher('/track_param_predictor', Float64, queue_size=10)
    rospy.init_node('track_predictor_listener', anonymous=True)

    rospy.Subscriber('/scan', LaserScan, laser_callback)

    rate = rospy.Rate(10)  

    print('starting')

    while not rospy.is_shutdown():
        if result is not None:
            pub.publish(Float64(result))
        rate.sleep()


if __name__ == '__main__':
    main()
