import rospy
import sys


class NodeBase:

    def __init__(self, name):
        rospy.init_node(name, sys.argv)
        # log methods
        self.DEBUG = rospy.logdebug
        self.INFO = rospy.loginfo
        self.WARN = rospy.logwarn
        self.ERROR = rospy.logerr
        self.FATAL = rospy.logfatal

    def __bool__(self):
        return not rospy.is_shutdown()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type: return False
        del self


if __name__ == '__main__':
    with NodeBase("demo") as node:
        try:
            while node:
                node.INFO("pending...")
                rospy.sleep(1)
        except rospy.ROSInterruptException:
            pass
