import rospy
from geometry_msgs.msg import PoseStamped

class WaypointManager:
    def __init__(self):
        rospy.init_node('waypoint_manager', anonymous=True)
        self.waypoint_pub = rospy.Publisher('/waypoints', PoseStamped, queue_size=10)
        self.waypoint_sub = rospy.Subscriber('/received_waypoints', PoseStamped, self.waypoint_callback)
        self.rate = rospy.Rate(10)
    
    def publish_waypoint(self, x, y, z):
        waypoint = PoseStamped()
        waypoint.header.stamp = rospy.Time.now()
        waypoint.header.frame_id = "map"
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.position.z = z
        waypoint.pose.orientation.w = 1.0
        
        self.waypoint_pub.publish(waypoint)
        rospy.loginfo(f"Published waypoint: x={x}, y={y}, z={z}")
    
    def waypoint_callback(self, msg):
        rospy.loginfo(f"Received waypoint: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}")
    
    def run(self):
        while not rospy.is_shutdown():
            x, y, z = 5.0, 5.0, 4.0 
            self.publish_waypoint(x, y, z)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        manager = WaypointManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass
