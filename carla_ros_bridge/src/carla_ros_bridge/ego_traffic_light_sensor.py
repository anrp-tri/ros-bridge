#!/usr/bin/env python

from collections import defaultdict
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

import carla

from carla_msgs.msg import CarlaEgoTrafficLightInfo, CarlaTrafficLightStatus
from carla_ros_bridge.ego_vehicle import EgoVehicle
from carla_ros_bridge.pseudo_actor import PseudoActor
from carla_ros_bridge.traffic import TrafficLight


# Publish info no less frequently than this.
_PUBLISH_INTERVAL_SECONDS = 5.0

def get_stop_line_location(carla_traffic_light):
        """Take average of all affected waypoints to get the stop location.
        Note: carla has a get_stop_waypoints() but it doesn't seem to return correct location.
        """
        stop_location = carla.Location()
        points = carla_traffic_light.get_affected_lane_waypoints()
        for point in points:
            stop_location += point.transform.location
        return stop_location / len(points)

def group_traffic_lights(node, actor_list, previous_road_search_distance=2.0):
    results = defaultdict(dict)
    for actor in actor_list.values():
        if isinstance(actor, TrafficLight):
            carla_actor = actor.carla_actor
            affected_points = carla_actor.get_affected_lane_waypoints()
            if len(affected_points) == 0:
                node.logerr("Unable to find any affected points at")
                continue

            for point in affected_points:
                if not point.is_junction:
                    node.logerr(f"The affected waypoint from traffic light is not in a junction. traffic_light_id: {carla_actor.id}, road_id: {point.road_id}, lane_id: {point.lane_id}")
                    continue
                stop_location = point.transform.location
                previous_points = point.previous(previous_road_search_distance)
                if len(previous_points) == 0:
                    node.logerr(f"Unable to find previous points for traffic_light_id: {carla_actor.id}, road_id: {point.road_id}, lane_id: {point.lane_id}")
                    continue
                p = previous_points[0]
                results[p.road_id][p.lane_id] = StoplineInfo(actor, p.road_id, p.lane_id, stop_location)
    return results

def get_stop_line_info(stop_line_info, road_id, lane_id):
    if stop_line_info is None:
        return None
    if road_id in stop_line_info and lane_id in stop_line_info[road_id]:
        return stop_line_info[road_id][lane_id]
    return None


class StoplineInfo:
    def __init__(self, traffic_light_actor, road_id, lane_id, stop_location):
        self.traffic_light_actor = traffic_light_actor
        self.road_id = road_id
        self.lane_id = lane_id
        self.stop_location = stop_location


class EgoTrafficLightSensor(PseudoActor):

    def __init__(self, uid, name, parent, node, actor_list, world):
        super(EgoTrafficLightSensor, self).__init__(uid=uid, name=name, parent=parent, node=node)

        self.pub = node.new_publisher(
            CarlaEgoTrafficLightInfo,
            self.get_topic_prefix() + "/info",
            qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        self._info_published_at = None

        self.map = world.get_map()
        self.map_name = self.map.name
        self.ego = self.get_ego(actor_list)
        self.actor_list = actor_list
        self.stop_line_info_map = None
        self.cur_sli = None

        self.msg = CarlaEgoTrafficLightInfo()
        self.msg.traffic_light_status = CarlaTrafficLightStatus()
        self.msg.inside_intersection = False
        if self.ego:
            self.msg.ego_id = self.ego.id

    def destroy(self):
        super(EgoTrafficLightSensor, self).destroy()
        self.node.destroy_publisher(self.pub)
        self.actor_list = None
        self.stop_line_info_map = None
        self.node.loginfo("Destroy EgoTrafficLightSensor")

    @staticmethod
    def get_blueprint_name():
        return "sensor.pseudo.ego_traffic_light"

    def update(self, frame, timestamp):
        if self.ego is None or self.map is None:
            return

        if self.stop_line_info_map is None or self.map_name != self.map.name:
            self.stop_line_info_map = group_traffic_lights(self.node, self.actor_list)
            self.map_name = self.map.name

        ego_location = self.ego.get_location()
        wp = self.map.get_waypoint(ego_location)
        sli = get_stop_line_info(self.stop_line_info_map, wp.road_id, wp.lane_id)

        if self.has_change(self.cur_sli, sli) \
            or (self._info_published_at is None or timestamp - self._info_published_at > _PUBLISH_INTERVAL_SECONDS):
            self.calculate_and_publish_data(ego_location, sli)

    def calculate_and_publish_data(self, ego_location, stop_line_info):
        self.cur_sli = stop_line_info
        try:
            if self.cur_sli:
                self.msg.distance_to_stopline = ego_location.distance(self.cur_sli.stop_location)
                self.msg.traffic_light_status = self.cur_sli.traffic_light_actor.get_status()
            else:
                self.msg.distance_to_stopline = -1.
                self.msg.traffic_light_status = None
        except Exception as e:
            self.node.loginfo("Error: {}".format(e))

        self.msg.inside_intersection = self.is_inside_intersection(self.ego, self.map)
        self.pub.publish(self.msg)

    def get_ego(self, actor_list):
        """Return a CarlaActor representing the ego car."""
        for actor in actor_list.values():
            if isinstance(actor, EgoVehicle):
                return actor.carla_actor
        return None

    def is_inside_intersection(self, carla_ego, carla_map):
        if carla_ego is None:
            return False
        waypoint = carla_map.get_waypoint(carla_ego.get_location())
        if waypoint is None:
            return False
        return waypoint.is_junction

    def has_change(self, cur_sli, new_sli):
        if cur_sli is None and new_sli is None:
            return False

        if cur_sli != new_sli:
            return True

        same_status = cur_sli.traffic_light_actor.get_status() == new_sli.traffic_light_actor.get_status()
        return not same_status

