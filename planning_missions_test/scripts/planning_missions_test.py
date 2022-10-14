#! /usr/bin/env python
# -*- coding: utf-8 -*-
from time import sleep
import rospy
import config
import json
from autoware_planning_msgs.msg import Mission, Missions, MissionAchieved

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

        _sub_missionAchieved_topic = rospy.get_param("~sub_missionAchieved_topic", "/planning/mission_planner/mission_achieved")
        self.sub_missionAchieved = rospy.Subscriber(_sub_missionAchieved_topic, MissionAchieved, self.missionAchieved_callback)
        sleep(3)
        self.pub_Missions_msg()
        self.pub_missions.publish(self.missions_msg)
        self.pub_current_mission(self.missions_msg.current_mission)
        self.missions_msg.all_missions = list()
        self.actionqueue.popleft()




    def missionAchieved_callback(self, msg):
        if msg.mission_order and msg.mission_achieved and len(self.actionqueue)>0:
            self.pub_Missions_msg()
            self.pub_missions.publish(self.missions_msg)
            self.pub_current_mission(self.missions_msg.current_mission)
            self.missions_msg.all_missions = list()
            self.actionqueue.popleft()



    def pub_Missions_msg(self):
            
        current_mission = self.actionqueue[0]
        self.missions_msg.current_mission = self.__creat_mission(current_mission["mission"]["goal"],\
                                                              current_mission["mission"]["lane_driving_sweeping_mode"],\
                                                              current_mission["mission"]["free_space_sweeping_mode"])
        for i in self.actionqueue:
            self.missions_msg.all_missions.append(self.__creat_mission(i["mission"]["goal"],\
                                                                    i["mission"]["lane_driving_sweeping_mode"],\
                                                                    i["mission"]["free_space_sweeping_mode"])) 
            

    #     # 
    #     if self.mission_order and self.mission_achieved:



    @staticmethod
    def __creat_mission(goal, lane_driving_sweeping_mode, free_space_sweeping_mode):
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

        return mission_msg
    

def main():
    rospy.init_node("Hello")
    rospy.loginfo("Hello World!!!!")
    test = Missions_publisher(config.planningFlie_path)

    rospy.spin()


if __name__ == "__main__":
    main()