#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from math import pi
import message_filters
import sys


class ArmController:
    def __init__(self):
        rospy.init_node("talker", anonymous=True)
        rospy.loginfo("Initializing the Investigator Arm")
        self.rate = rospy.Rate(10)  # 10hz
        if len(sys.argv) == 1:
            self._required_pose = "relax arm"  # "relax arm", "tall_arm", "default arm"
        else:
            self._required_pose = sys.argv[1]
        self.joint_state_checked = [False, False, False, False, False, False, False]
        self.relax_arm_pose = [(0.0), (-(pi / 2)), (0.0), (pi / 2), (0.0), (0.0), (0.0)]
        self.tall_arm_pose = [(0.0), (-(pi / 2)), (pi / 2), (0.0), (0.0), (0.0), (0.0)]
        self.default_arm_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.stability_error = 0.1
        self._joint_1_pub = rospy.Publisher(
            "/cluerosity/joint1_position_controller/command", Float64, queue_size=10
        )
        self._joint_2_pub = rospy.Publisher(
            "/cluerosity/joint2_position_controller/command", Float64, queue_size=10
        )
        self._joint_3_pub = rospy.Publisher(
            "/cluerosity/joint3_position_controller/command", Float64, queue_size=10
        )
        self._joint_4_pub = rospy.Publisher(
            "/cluerosity/joint4_position_controller/command", Float64, queue_size=10
        )
        self._joint_5_pub = rospy.Publisher(
            "/cluerosity/joint5_position_controller/command", Float64, queue_size=10
        )
        self._joint_6_pub = rospy.Publisher(
            "/cluerosity/joint6_position_controller/command", Float64, queue_size=10
        )
        self._joint_7_pub = rospy.Publisher(
            "/cluerosity/joint7_position_controller/command", Float64, queue_size=10
        )
        rospy.Subscriber(
            "/cluerosity/joint1_position_controller/state",
            JointControllerState,
            self.joint_1_state_clbk,
        )
        rospy.Subscriber(
            "/cluerosity/joint2_position_controller/state",
            JointControllerState,
            self.joint_2_state_clbk,
        )
        rospy.Subscriber(
            "/cluerosity/joint3_position_controller/state",
            JointControllerState,
            self.joint_3_state_clbk,
        )
        rospy.Subscriber(
            "/cluerosity/joint4_position_controller/state",
            JointControllerState,
            self.joint_4_state_clbk,
        )
        rospy.Subscriber(
            "/cluerosity/joint5_position_controller/state",
            JointControllerState,
            self.joint_5_state_clbk,
        )
        rospy.Subscriber(
            "/cluerosity/joint6_position_controller/state",
            JointControllerState,
            self.joint_6_state_clbk,
        )
        rospy.Subscriber(
            "/cluerosity/joint7_position_controller/state",
            JointControllerState,
            self.joint_7_state_clbk,
        )
        # self._joint_1_sub = message_filters.Subscriber('/cluerosity/joint1_position_controller/state', JointControllerState)
        # self._joint_2_sub = message_filters.Subscriber('/cluerosity/joint2_position_controller/state', JointControllerState)
        # self._joint_3_sub = message_filters.Subscriber('/cluerosity/joint3_position_controller/state', JointControllerState)
        # self._joint_4_sub = message_filters.Subscriber('/cluerosity/joint4_position_controller/state', JointControllerState)
        # self._joint_5_sub = message_filters.Subscriber('/cluerosity/joint5_position_controller/state', JointControllerState)
        # self._joint_6_sub = message_filters.Subscriber('/cluerosity/joint6_position_controller/state', JointControllerState)
        # self._joint_7_sub = message_filters.Subscriber('/cluerosity/joint7_position_controller/state', JointControllerState)
        self._set_required_pose()
        # ts = message_filters.TimeSynchronizer([self._joint_1_sub, self._joint_2_sub, self._joint_3_sub, self._joint_4_sub, self._joint_5_sub, self._joint_6_sub, self._joint_7_sub], 10)
        # ts.registerCallback(self.joint_state_callback)

    # def joint_state_callback(self, joint_state1,
    #     joint_state2,
    #     joint_state3,
    #     joint_state4,
    #     joint_state5,
    #     joint_state6,
    #     joint_state7 ):

    #     if joint_state1.process_value < self.stability_error and joint_state1.process_value > -self.stability_error and self.joint_1_state_checked == False:
    #         rospy.loginfo("Joint 1 initialized")
    #         self.joint_1_state_checked = True
    #     if joint_state2.process_value < (-(pi/2) + self.stability_error) and joint_state2.process_value > (-(pi/2) - self.stability_error) and self.joint_2_state_checked == False:
    #         rospy.loginfo("Joint 2 initialized")
    #         rospy.loginfo("I'm in joint 2")
    #         self.joint_2_state_checked = True
    #     if joint_state3.process_value < self.stability_error and joint_state3.process_value > -self.stability_error and self.joint_3_state_checked == False:
    #         rospy.loginfo("Joint 3 initialized")
    #         self.joint_3_state_checked = True
    #     if joint_state4.process_value < ((pi/2) + self.stability_error) and joint_state4.process_value > ((pi/2) - self.stability_error) and self.joint_4_state_checked == False:
    #         rospy.loginfo("Joint 4 initialized")
    #         self.joint_4_state_checked = True
    #     if joint_state5.process_value < self.stability_error and joint_state5.process_value > -self.stability_error and self.joint_5_state_checked == False:
    #         rospy.loginfo("Joint 5 initialized")
    #         self.joint_5_state_checked = True
    #     if joint_state6.process_value < self.stability_error and joint_state6.process_value > -self.stability_error and self.joint_6_state_checked == False:
    #         rospy.loginfo("Joint 6 initialized")
    #         self.joint_6_state_checked = True
    #     if joint_state7.process_value < self.stability_error and joint_state7.process_value > -self.stability_error and self.joint_7_state_checked == False:
    #         rospy.loginfo("Joint 7 initialized")
    #         self.joint_7_state_checked = True
    #     rospy.loginfo("I'm here")

    def joint_1_state_clbk(self, joint_state):
        if (
            joint_state.process_value < (self.relax_arm_pose[0] + self.stability_error)
            and joint_state.process_value
            > (self.relax_arm_pose[0] - self.stability_error)
            and self.joint_state_checked[0] == False
            and self._required_pose == "relax arm"
        ):
            rospy.loginfo("Joint 1 initialized")
            self.joint_state_checked[0] = True

        if (
            joint_state.process_value < (self.tall_arm_pose[0] + self.stability_error)
            and joint_state.process_value
            > (self.tall_arm_pose[0] - self.stability_error)
            and self.joint_state_checked[0] == False
            and self._required_pose == "tall arm"
        ):
            rospy.loginfo("Joint 1 initialized")
            self.joint_state_checked[0] = True

        if (
            joint_state.process_value
            < (self.default_arm_pose[0] + self.stability_error)
            and joint_state.process_value
            > (self.default_arm_pose[0] - self.stability_error)
            and self.joint_state_checked[0] == False
            and self._required_pose == "default arm"
        ):
            rospy.loginfo("Joint 1 initialized")
            self.joint_state_checked[0] = True

    def joint_2_state_clbk(self, joint_state):
        if (
            joint_state.process_value < (self.relax_arm_pose[1] + self.stability_error)
            and joint_state.process_value
            > (self.relax_arm_pose[1] - self.stability_error)
            and self.joint_state_checked[1] == False
            and self._required_pose == "relax arm"
        ):
            rospy.loginfo("Joint 2 initialized")
            self.joint_state_checked[1] = True

        if (
            joint_state.process_value < (self.tall_arm_pose[1] + self.stability_error)
            and joint_state.process_value
            > (self.tall_arm_pose[1] - self.stability_error)
            and self.joint_state_checked[1] == False
            and self._required_pose == "tall arm"
        ):
            rospy.loginfo("Joint 2 initialized")
            self.joint_state_checked[1] = True

        if (
            joint_state.process_value
            < (self.default_arm_pose[1] + self.stability_error)
            and joint_state.process_value
            > (self.default_arm_pose[1] - self.stability_error)
            and self.joint_state_checked[1] == False
            and self._required_pose == "default arm"
        ):
            rospy.loginfo("Joint 2 initialized")
            self.joint_state_checked[1] = True

    def joint_3_state_clbk(self, joint_state):
        if (
            joint_state.process_value < (self.relax_arm_pose[2] + self.stability_error)
            and joint_state.process_value
            > (self.relax_arm_pose[2] - self.stability_error)
            and self.joint_state_checked[2] == False
            and self._required_pose == "relax arm"
        ):
            rospy.loginfo("Joint 3 initialized")
            self.joint_state_checked[2] = True

        if (
            joint_state.process_value < (self.tall_arm_pose[2] + self.stability_error)
            and joint_state.process_value
            > (self.tall_arm_pose[2] - self.stability_error)
            and self.joint_state_checked[2] == False
            and self._required_pose == "tall arm"
        ):
            rospy.loginfo("Joint 3 initialized")
            self.joint_state_checked[2] = True

        if (
            joint_state.process_value
            < (self.default_arm_pose[2] + self.stability_error)
            and joint_state.process_value
            > (self.default_arm_pose[2] - self.stability_error)
            and self.joint_state_checked[2] == False
            and self._required_pose == "default arm"
        ):
            rospy.loginfo("Joint 3 initialized")
            self.joint_state_checked[2] = True

    def joint_4_state_clbk(self, joint_state):
        if (
            joint_state.process_value < (self.relax_arm_pose[3] + self.stability_error)
            and joint_state.process_value
            > (self.relax_arm_pose[3] - self.stability_error)
            and self.joint_state_checked[3] == False
            and self._required_pose == "relax arm"
        ):
            rospy.loginfo("Joint 4 initialized")
            self.joint_state_checked[3] = True

        if (
            joint_state.process_value < (self.tall_arm_pose[3] + self.stability_error)
            and joint_state.process_value
            > (self.tall_arm_pose[3] - self.stability_error)
            and self.joint_state_checked[3] == False
            and self._required_pose == "tall arm"
        ):
            rospy.loginfo("Joint 4 initialized")
            self.joint_state_checked[3] = True

        if (
            joint_state.process_value
            < (self.default_arm_pose[3] + self.stability_error)
            and joint_state.process_value
            > (self.default_arm_pose[3] - self.stability_error)
            and self.joint_state_checked[3] == False
            and self._required_pose == "default arm"
        ):
            rospy.loginfo("Joint 4 initialized")
            self.joint_state_checked[3] = True

    def joint_5_state_clbk(self, joint_state):
        if (
            joint_state.process_value < (self.relax_arm_pose[4] + self.stability_error)
            and joint_state.process_value
            > (self.relax_arm_pose[4] - self.stability_error)
            and self.joint_state_checked[4] == False
            and self._required_pose == "relax arm"
        ):
            rospy.loginfo("Joint 5 initialized")
            self.joint_state_checked[4] = True

        if (
            joint_state.process_value < (self.tall_arm_pose[4] + self.stability_error)
            and joint_state.process_value
            > (self.tall_arm_pose[4] - self.stability_error)
            and self.joint_state_checked[4] == False
            and self._required_pose == "tall arm"
        ):
            rospy.loginfo("Joint 5 initialized")
            self.joint_state_checked[4] = True

        if (
            joint_state.process_value
            < (self.default_arm_pose[4] + self.stability_error)
            and joint_state.process_value
            > (self.default_arm_pose[4] - self.stability_error)
            and self.joint_state_checked[4] == False
            and self._required_pose == "default arm"
        ):
            rospy.loginfo("Joint 5 initialized")
            self.joint_state_checked[4] = True

    def joint_6_state_clbk(self, joint_state):
        if (
            joint_state.process_value < (self.relax_arm_pose[5] + self.stability_error)
            and joint_state.process_value
            > (self.relax_arm_pose[5] - self.stability_error)
            and self.joint_state_checked[5] == False
            and self._required_pose == "relax arm"
        ):
            rospy.loginfo("Joint 6 initialized")
            self.joint_state_checked[5] = True

        if (
            joint_state.process_value < (self.tall_arm_pose[5] + self.stability_error)
            and joint_state.process_value
            > (self.tall_arm_pose[5] - self.stability_error)
            and self.joint_state_checked[5] == False
            and self._required_pose == "tall arm"
        ):
            rospy.loginfo("Joint 6 initialized")
            self.joint_state_checked[5] = True

        if (
            joint_state.process_value
            < (self.default_arm_pose[5] + self.stability_error)
            and joint_state.process_value
            > (self.default_arm_pose[5] - self.stability_error)
            and self.joint_state_checked[5] == False
            and self._required_pose == "default arm"
        ):
            rospy.loginfo("Joint 6 initialized")
            self.joint_state_checked[5] = True

    def joint_7_state_clbk(self, joint_state):
        if (
            joint_state.process_value < (self.relax_arm_pose[6] + self.stability_error)
            and joint_state.process_value
            > (self.relax_arm_pose[6] - self.stability_error)
            and self.joint_state_checked[6] == False
            and self._required_pose == "relax arm"
        ):
            rospy.loginfo("Joint 7 initialized")
            self.joint_state_checked[6] = True

        if (
            joint_state.process_value < (self.tall_arm_pose[6] + self.stability_error)
            and joint_state.process_value
            > (self.tall_arm_pose[6] - self.stability_error)
            and self.joint_state_checked[6] == False
            and self._required_pose == "tall arm"
        ):
            rospy.loginfo("Joint 7 initialized")
            self.joint_state_checked[6] = True

        if (
            joint_state.process_value
            < (self.default_arm_pose[6] + self.stability_error)
            and joint_state.process_value
            > (self.default_arm_pose[6] - self.stability_error)
            and self.joint_state_checked[6] == False
            and self._required_pose == "default arm"
        ):
            rospy.loginfo("Joint 7 initialized")
            self.joint_state_checked[6] = True

    def _check_pose_state(self):
        if False not in self.joint_state_checked:
            rospy.loginfo(
                "\033[92m"
                + "\n\nThe Investigator Manipulator has been initialized\n"
                + "\033[0m"
            )
            rospy.signal_shutdown("End of Program")

    def _set_required_pose(self):
        while not rospy.is_shutdown():
            if self._required_pose == "relax arm":
                self._joint_1_pub.publish(self.relax_arm_pose[0])
                self._joint_2_pub.publish(self.relax_arm_pose[1])
                self._joint_3_pub.publish(self.relax_arm_pose[2])
                self._joint_4_pub.publish(self.relax_arm_pose[3])
                self._joint_5_pub.publish(self.relax_arm_pose[4])
                self._joint_6_pub.publish(self.relax_arm_pose[5])
                self._joint_7_pub.publish(self.relax_arm_pose[6])
            elif self._required_pose == "tall arm":
                self._joint_1_pub.publish(self.tall_arm_pose[0])
                self._joint_2_pub.publish(self.tall_arm_pose[1])
                self._joint_3_pub.publish(self.tall_arm_pose[2])
                self._joint_4_pub.publish(self.tall_arm_pose[3])
                self._joint_5_pub.publish(self.tall_arm_pose[4])
                self._joint_6_pub.publish(self.tall_arm_pose[5])
                self._joint_7_pub.publish(self.tall_arm_pose[6])
            elif self._required_pose == "default arm":
                self._joint_1_pub.publish(self.default_arm_pose[0])
                self._joint_2_pub.publish(self.default_arm_pose[1])
                self._joint_3_pub.publish(self.default_arm_pose[2])
                self._joint_4_pub.publish(self.default_arm_pose[3])
                self._joint_5_pub.publish(self.default_arm_pose[4])
                self._joint_6_pub.publish(self.default_arm_pose[5])
                self._joint_7_pub.publish(self.default_arm_pose[6])
            self.rate.sleep()
            self._check_pose_state()


if __name__ == "__main__":
    try:
        ArmController()
    except rospy.ROSInterruptException:
        pass
