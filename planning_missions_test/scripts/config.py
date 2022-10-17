#! /usr/bin/env python
# -*- coding: utf-8 -*-
import os
import rospy

script_path = os.path.dirname(os.path.abspath(__file__)) 

__planningFlienName = "config/planning2.json"
planningFlie_path = script_path + "/../" + __planningFlienName

planning_topic = ""