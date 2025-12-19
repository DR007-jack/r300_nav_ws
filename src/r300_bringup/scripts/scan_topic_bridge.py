#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Simple topic normalizer/bridge for LaserScan topics.

This node subscribes to a list of candidate LaserScan topics and republishes
the messages (unchanged) to a single canonical topic (default: /scan).

Why: this repository contains multiple launch files that expect different
laser topic names (e.g. `/scan`, `unilidar/laserscan`). A small, robust
bridge lets the navigation stack always consume `/scan` without editing
drivers or many launch files.

Compatibility: ROS1 (Melodic), Python 2.7 (use shebang and rospy). Keep code
simple to avoid encoding/format issues.
"""

import rospy
from sensor_msgs.msg import LaserScan


class ScanBridge(object):
    def __init__(self):
        # Accept either a YAML list or a space-separated string for backward compatibility
        raw = rospy.get_param('~input_topics', ['/scan', 'unilidar/laserscan', '/unilidar/laserscan'])
        if isinstance(raw, basestring):
            input_topics = [t for t in raw.split() if t]
        else:
            input_topics = list(raw)

        self.output_topic = rospy.get_param('~output_topic', '/scan')
        queue_size = rospy.get_param('~queue_size', 10)

        rospy.loginfo('scan_topic_bridge: publishing to %s, subscribing to %s', self.output_topic, input_topics)

        self.pub = rospy.Publisher(self.output_topic, LaserScan, queue_size=queue_size)

        # create subscribers for all candidate topics; callbacks will publish to output
        self.subs = []
        for t in input_topics:
            try:
                sub = rospy.Subscriber(t, LaserScan, self._cb, callback_args=t, queue_size=queue_size)
                self.subs.append(sub)
            except Exception as e:
                rospy.logwarn('scan_topic_bridge: could not subscribe to %s: %s', t, e)

    def _cb(self, msg, src_topic):
        # Forward the message unchanged. We preserve the incoming header/frame_id.
        # Optionally we could override header.frame_id here if needed.
        try:
            self.pub.publish(msg)
        except Exception as e:
            rospy.logerr('scan_topic_bridge: publish failed from %s: %s', src_topic, e)


def main():
    rospy.init_node('scan_topic_bridge')
    bridge = ScanBridge()
    rospy.spin()


if __name__ == '__main__':
    main()
