from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import rospy
from geometry_msgs.msg import Point, Vector3, Pose
from .util import point_distance
from .data import ClusterPoint
import states as states
import json
from .trader import Trader


class Clusterer():

    def __init__(self, min_center_detections=15):
        self.centers = []
        self.markers = MarkerArray()
        self.centroid_treshold = 0.6
        self.min_center_detections = min_center_detections
        self.jobs = []
        self.state = states.DEFAULT
        self.did_calculate_profits = False
        self.trader = Trader()

        self.visualization_colors = [ColorRGBA(255, 0, 0, 1), ColorRGBA(
            255, 255, 0, 1), ColorRGBA(0, 0, 255, 1)]

        self.markers_pub = rospy.Publisher(
            'markers', MarkerArray, queue_size=1000)

        _ = rospy.Subscriber("cluster/point", Marker, self.point_callback)

    def change_state(self,state):
        self.state = state

    def has_pending_jobs(self):
        return len(self.jobs) > 0

    def is_ready_for_cylinders(self):
        has_data = self.get_num_data_confirmed_circles() >= 7
        return has_data and self.did_calculate_profits

    def get_next_job(self):
        job = self.jobs[0]
        self.jobs = self.jobs[1:]
        return job

    def get_circle_clusters(self):
        return [center for center in self.centers if center.is_circle]

    def get_num_data_confirmed_circles(self):
        return len([center for center in self.centers if center.is_circle and center.data and center.is_visited])

    def get_cylinder_clusters(self):
        return [center for center in self.centers if not center.is_circle]

    def get_num_cylinders_detected(self):
        return len(self.get_cylinder_clusters())


    def get_cluster(self, cluster_ix, type=Marker.CYLINDER):
        centers = self.get_cylinder_clusters() if type == Marker.CYLINDER else self.get_circle_clusters()
        cluster = centers[cluster_ix]
        return cluster


    def reset_cluster(self, cluster_ix, type=Marker.CYLINDER):
        centers = self.get_cylinder_clusters() if type == Marker.CYLINDER else self.get_circle_clusters()
        cluster = centers[cluster_ix]
        print("Reseting cluster: ", cluster)
        reseted_cluster = ClusterPoint(cluster.x, cluster.y, 1, True)
        centers[cluster_ix] = reseted_cluster


    def find_nearest_cluster(self, p, type=Marker.CYLINDER):
        closest_center = None
        min_dist = 999999999
        min_ix = 0



        centers = self.get_cylinder_clusters() if type == Marker.CYLINDER else self.get_circle_clusters()

        for center_ix, center in enumerate(centers):
            dist = point_distance(p, center)
            if dist < min_dist:
                if closest_center:
                    print("Detected in multiple centers")

                closest_center = center
                min_dist = dist
                min_ix = center_ix

        is_circle = type == Marker.SPHERE
        return closest_center, min_ix, centers, is_circle


    def get_type_centers(self, type):
        is_circle = False
        centers = None
        if type == Marker.SPHERE:
            print("Got Circle!")
            is_circle = True
            centers = self.get_circle_clusters()
        elif type == Marker.CYLINDER:
            print("Got cylinder")
            centers = self.get_cylinder_clusters()
        else:
            print("Wrong marker type!!")
        return centers, is_circle


    def point_callback(self, marker):
        p = marker.pose.position

        if self.state != states.OBSERVING:
            return

        color = marker.color  ## TODO Extract actual color
        data = json.loads(marker.text) if marker.text else None
        closest_center, min_ix, centers, is_circle = self.find_nearest_cluster(p, marker.type)

        if is_circle:
            print("Got circle!")
        else:
            print("Got cylinder")

        print(closest_center, min_ix, centers)


        if closest_center:
            print("[Cluster] updating existing cluster")
            new_center = closest_center.move_center(p, data) ## Update reference to self.centers
            centers[min_ix] = new_center

            if new_center.n >= self.min_center_detections and not new_center.is_visited:
                new_center.is_visited = True
                centers[min_ix] = new_center
                if is_circle:
                    print("Circle with enough detections: ", new_center)
                else:
                    print("Cylindner with enough detections: ", new_center)
                    self.jobs.append(new_center)
        else:
            self.centers.append(ClusterPoint(p.x, p.y, 1, is_circle, color, data)) ## Append to self.centers
            print("[Cluster] Adding new center")

        self.publish_markers()

    def publish_markers(self):
        by_n = sorted(self.centers, key=lambda center: center.n, reverse=True)
        markers = [self.point_2_marker(p, ix)
                   for (ix, p) in enumerate(by_n[:3])]
        print("[Cluster] Publishing {} markers.".format(len(markers)))
        print(by_n)
        self.markers_pub.publish(markers)

    def point_2_marker(self, p, ix):
        pose = Pose()
        pose.position.x = p.x
        pose.position.y = p.y
        pose.position.z = 0.5
        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = "map"
        marker.pose = pose
        marker.type = p.type
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(30)
        marker.id = ix
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = p.color
        return marker
