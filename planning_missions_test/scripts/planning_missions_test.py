#! /usr/bin/env python
# -*- coding: utf-8 -*-
from time import sleep
import threading
import rospy
import config
import json
from std_msgs.msg import Bool
from autoware_planning_msgs.msg import Mission, Missions, MissionAchieved
from geometry_msgs.msg import TwistStamped
from collections import deque

class Missions_publisher:
    def __init__(self, planningFlie_path):
        # print(planningFlie_path)
        with open(planningFlie_path, 'r') as f:
            self.json_load =json.load(f)

        self.actionqueue = deque(self.json_load["actionLists"] )
    
        self.missions_msg = Missions()

        _pub_missionsPlanning_topic = rospy.get_param("~pub_missionsPlanning_topic", "/planning/mission_planner/missoins")
        self.pub_missions =  rospy.Publisher(_pub_missionsPlanning_topic, Missions, queue_size=1)

        _pub_mission_Planning_topic = rospy.get_param("~pub_mission_Planning_topic", "/planning/mission_planner/current_mission")
        self.pub_current_mission =  rospy.Publisher(_pub_mission_Planning_topic, Mission, queue_size=1)
        
        self.__pub_autoware_engage_msg = Bool()
        _pub_autopilotSwitch = rospy.get_param("~autoware_engage", "/autoware/engage")
        self.pub_autopilotSwitch =  rospy.Publisher(_pub_autopilotSwitch, Bool, queue_size=1)

        _sub_missionAchieved_topic = rospy.get_param("~sub_missionAchieved_topic", "/planning/mission_planner/mission_achieved")
        self.sub_missionAchieved = rospy.Subscriber(_sub_missionAchieved_topic, MissionAchieved, self.missionAchieved_callback)
        
        self.chassisTwistX = 0
        _sub_chassisTwistSpeed_topic = rospy.get_param("~sub_chassisSpeed_topic", "/localization/twist")
        self.sub_chassisTwistSpeed = rospy.Subscriber(_sub_chassisTwistSpeed_topic, TwistStamped, self.chassisTwistSpeed_callback)           

        self.Tiemer  = rospy.Timer(rospy.Duration(1), self.timer_callback)
        self.count = 0
        sleep(3)
        self.pub_mission_missions()


    def timer_callback(self, event):
        if(self.count>=10):
            sleep(1)
            self.count = self.count - 1
            
    
    def chassisTwistSpeed_callback(self, msg):
        self.chassisTwistX = msg.twist.linear.x

    def missionAchieved_callback(self, msg):
        if msg.mission_achieved and msg.mission_order==self.missions_msg.current_mission.mission_order  and len(self.actionqueue)>0:
            rospy.loginfo("%d号--执行完毕--开始执行%d号任务",self.missions_msg.current_mission.mission_order, self.actionqueue[1])
            self.pub_mission_missions()



    def create_Missions_msg(self):
            
        current_mission = self.actionqueue[0]
        self.missions_msg.current_mission = self.__creat_mission(current_mission["mission"]["goal"],current_mission["mission"]["mission_order"],\
                                                              current_mission["mission"]["lane_driving_sweeping_mode"],\
                                                              current_mission["mission"]["free_space_sweeping_mode"])
        for i in self.actionqueue:
            self.missions_msg.all_missions.append(self.__creat_mission(i["mission"]["goal"],current_mission["mission"]["mission_order"],\
                                                                    i["mission"]["lane_driving_sweeping_mode"],\
                                                                    i["mission"]["free_space_sweeping_mode"])) 
            

    def pub_autopilotSwitch_func(self, autopilotSwitch, time_sec):
        self.__pub_autoware_engage_msg.data = autopilotSwitch
        count=0  
        while(-0.5 <= self.chassisTwistX <= 0.5 ):
            rospy.loginfo("速度为零，自动驾驶没有启动")
            if(count>time_sec): break
            self.pub_autopilotSwitch.publish(self.__pub_autoware_engage_msg)
            count = count + 1
            sleep(1)
        

    def pub_mission_missions(self):
        self.create_Missions_msg()

        self.pub_current_mission.publish(self.missions_msg.current_mission)
        self.pub_missions.publish(self.missions_msg)
        
        self.missions_msg.all_missions = list()
        self.actionqueue.popleft()

        rospy.loginfo("%d号任务--执行开始",self.missions_msg.current_mission.mission_order)
        self.pub_autopilotSwitch_func(True, 1)


    @staticmethod
    def __creat_mission(goal, mission_order, lane_driving_sweeping_mode, free_space_sweeping_mode):
        mission_msg  = Mission()
        mission_msg.goal.position.x = goal["x"]
        mission_msg.goal.position.y = goal["y"]
        mission_msg.goal.position.z = goal["z"]

        mission_msg.goal.orientation.x = goal["qx"]
        mission_msg.goal.orientation.y = goal["qy"]
        mission_msg.goal.orientation.z = goal["qz"]
        mission_msg.goal.orientation.w = goal["qw"]

        mission_msg.lane_driving_sweeping_mode = lane_driving_sweeping_mode
        mission_msg.free_space_sweeping_mode = free_space_sweeping_mode
        mission_msg.mission_order = mission_order


        return mission_msg
    

def main():
    rospy.init_node("Hello")
    rospy.loginfo("Hello World!!!!")
    test = Missions_publisher(config.planningFlie_path)

    rospy.spin()


if __name__ == "__main__":
    main()