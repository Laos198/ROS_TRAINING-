#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import GetWorldProperties, GetModelState
from geometry_msgs.msg import Point
from tsr1.srv import GetClosest, GetDistance, GetClosestResponse
from nav_msgs.msg import Odometry
import math


class GazeboUtils(object):
    def __init__(self):
        pass

    def getWorldProperties(self):
        try:
            get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            wp = get_world_properties()
            if wp.success:
                return wp
            else:
                rospy.logwarn(f"al invocar el servicio se recibio el estatus: {wp.success}")
                return None
        except rospy.rospy.ServiceException as se:
            rospy.logerr(f"Error al llamar '/gazebo/get_world_properties': {se}")    

    def getModelState(self, model_name, relative_entity_name='world'):
        try:
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            ms = get_model_state(model_name, relative_entity_name)
            if ms.success:
                return ms
            else:    
                rospy.logwarn(f"al invocar el servicio se recibio el estatus: {ms.success}")
                return None
        except rospy.rospy.ServiceException as se:
            rospy.logerr(f"Error al llamar '/gazebo/get_model_state': {se}")   


class DistanceMonitor(): 
    def __init__(self): 
        self.landmarks = {}
        self.exclude_objs = ['ground_plane', 'turtlebot3_waffle']
        self._gazeebo_utils = GazeboUtils()
        self.position = Point()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self._on_odom_callback)
        self._getClosestSrv : rospy.Service('/get_closest',GetClosest, self.get_closest_srv)
        self._getDistanceSrv : rospy.Service('/get_distance',GetDistance, self.get_distance_srv)
    
    def _on_odom_callback(self, msg): 
        self.position = msg.pose.pose.position

    def get_closest_srv(self, req):
        closest_landmark = ''
        closest_distance = -1
        ##como saber cuales son los objetos de la simulacion
        for model_name, (x,y) in self.landmarks.items():
            dx = x - self._position.x
            dy = y - self._position.y
            sqr_dist = (dx*dx) + (dy*dy)
            if closest_distance == -1 or sqr_dist <closest_distance: 
                closest_landmark = model_name

        response = GetClosestResponse()
        response.object_name = closest_landmark
        response.succes = True 
        response.status_message = "Todo OKA"

        return response

        
    def get_distance_srv(self, req): 
        response = GetClosestResponse()
        if req.object_name not in self.landmarks: 
            response.object_distance = 0.0
            response.succes = False
            response.status_message = f"el objeto '{req.object_name}'no fue encontrado"
            return response
        x , y = self._landmarks[req.object_name]
        dx = x - self._position.x
        dy = y - self._position.y
        response.object_distance = math.hypot(dx,dy)
        response.succes = True
        response.status_message = "Todo ok"

        return response

        


    def _ini(self): 
        wp = self._gazeebo_utils.getWorldProperties()
        if wp: 
            for model in wp.model_names: 
                if model not in self._excluded_objs: 
                    ms = self._gazeebo_utils.getModelState(model)
                    position = (ms.pose.position.x, ms.pose.position.y)
                    self.landmarks.update({model: position})

        

def test_service():
    gazebo_utils = GazeboUtils()
    wp = gazebo_utils.getWorldProperties()
    if wp:
        for model in wp.model_names:
            ms = gazebo_utils.getModelState(model)
            position = (ms.pose.position.x, ms.pose.position.y) 
            print(f"model_name: {model} (x:{position[0]}, y:{position[1]})")

def main(): 
    rospy.init_node('distance_monitor_server')
    monitor = DistanceMonitor()
    rospy.spin()

if __name__ == '__main__':
    main()