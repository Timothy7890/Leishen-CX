#!/usr/bin/env python3
"""以 10 Hz 记录 FAST-LIO 位姿 (camera_init -> body) 到 CSV。"""
import csv

import rospy
import tf2_ros


def main():
    rospy.init_node("trajectory_recorder", anonymous=False)
    out = rospy.get_param("~output_file", "trajectory.csv")
    src, dst = "camera_init", "body"
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)

    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
        rate = rospy.Rate(10)
        last = None
        rospy.loginfo("recording %s -> %s to %s", src, dst, out)
        while not rospy.is_shutdown():
            try:
                t = buf.lookup_transform(src, dst, rospy.Time(0), rospy.Duration(1.0))
                stamp = t.header.stamp.to_sec()
                if stamp != last:  # 位姿未更新时不重复写入
                    p, q = t.transform.translation, t.transform.rotation
                    w.writerow([f"{stamp:.6f}", p.x, p.y, p.z, q.x, q.y, q.z, q.w])
                    f.flush()
                    last = stamp
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break


if __name__ == "__main__":
    main()
