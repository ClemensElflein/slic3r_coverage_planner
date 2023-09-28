#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from geometry_msgs.msg import Polygon, Point32
from slic3r_coverage_planner.srv import PlanPath
from nav_msgs.msg import Path
from std_msgs.msg import Header

# Example of how to call slic3r_coverage_planner PlanPath service.
# It takes a polyon as outline and an arary of polygones for inner holes (isles)
# It returns a list of Slic3r pathes. Each Slic3r path object consists of a nav_msgs/Path object
#
# This program calls the Slic3r path planner for a simple polygon and publishes the first
# path segment as nav_msgs/Path


def client():
    rospy.wait_for_service("slic3r_coverage_planner/plan_path")
    pub = rospy.Publisher("slic3r_path", Path, queue_size=10)
    rospy.init_node("talker", anonymous=True)
    try:
        slic3r_proxy = rospy.ServiceProxy("slic3r_coverage_planner/plan_path", PlanPath)
        p = Polygon()  # outline
        holes = [] 
        hole = Polygon()

        hole.points = [
            Point32(x=2, y=1),
            Point32(x=-2, y=1),
            Point32(x=-2, y=0),
            Point32(x=-1, y=1),
            Point32(x=2, y=2),]
        holes.append(hole)  
        p.points = [
            Point32(x=0, y=5),
            Point32(x=4, y=4),
            Point32(x=4, y=1),
            Point32(x=3, y=-2),
            Point32(x=1, y=-3),
            Point32(x=-2, y=0),
            Point32(x=-3, y=-1.3),
            Point32(x=-4, y=-4),
            Point32(x=-3, y=-2),
            Point32(x=-2, y=-1),
            Point32(x=-1, y=3),
            Point32(x=0, y=5)]
        pathSrv = PlanPath()
        pathSrv.angle = 20
        pathSrv.outline_count = 2
        pathSrv.distance = 0.2
        pathSrv.fill_type = 0
        pathSrv.outline = p
        pathSrv.holes = holes

        # ['fill_type', 'angle', 'distance', 'outer_offset', 'outline_count', 'outline_overlap count', 'outline', 'holes']
        resp1 = slic3r_proxy(0, 20.0, 0.2, 0, 2, 0, p, holes)
        #resp1 = slic3r_proxy(pathSrv)
        nav_path = Path()
        header = Header()
        rate = rospy.Rate(1)  # 1hz
        while not rospy.is_shutdown():
            # wait for subscribers
            connections = pub.get_num_connections()
            # at least one subscriber? send the generated path
            if connections > 0:
                header.frame_id = "map"
                header.stamp = rospy.Time.now()

                nav_path.header = header
                for path in resp1.paths:
                    
                    nav_path.poses = path.path.poses
                    pub.publish(nav_path)
                    time.sleep(5)
                rospy.spin()  # infinite loop ensures to send only once
            rate.sleep()

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    try:
        client()
    except rospy.ROSInterruptException:
        pass
