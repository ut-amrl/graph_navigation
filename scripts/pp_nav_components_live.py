
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray

def callback(data):
  nav_components = data.data
  print('--------------------------------------------------------------------------------------------------')
  for idx in range(0, int(len(nav_components)), 7):
    print("Curvature: {}, Length: {}, Progress: {}, FPL: {}, Clearance: {}, COST: {} total {}\n".format(nav_components[idx], nav_components[idx+1], nav_components[idx+2], nav_components[idx+3], nav_components[idx+4], nav_components[idx+5], nav_components[idx+6]))
  print('--------------------------------------------------------------------------------------------------')
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("nav_cost_components", Float32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


listener()