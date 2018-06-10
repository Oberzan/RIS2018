from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import rospy
from geometry_msgs.msg import Point, Vector3, Pose
from util import point_distance
from data import ClusterPoint
import states as states
import json
import colorsys


class Clusterer():

    def __init__(self, cluster_topic, min_center_detections=15):
        self.centers = []
        self.markers = MarkerArray()
        self.centroid_treshold = 0.6
        self.min_center_distance = min_center_detections
        self.jobs = []
        self.jobs_created = 0
        self.state = states.DEFAULT
        self.jobs_calculated = False
        self.topic = cluster_topic
        self.num_jobs_handled = 0
        self.finished_jobs = []


        self.visualization_colors = [ColorRGBA(255, 0, 0, 1), ColorRGBA(
            255, 255, 0, 1), ColorRGBA(0, 0, 255, 1)]

        self.markers_pub = rospy.Publisher(
            'markers', MarkerArray, queue_size=1000)

        _ = rospy.Subscriber(cluster_topic, Marker, self.point_callback)

    def is_circle_cluster(self):
        return self.topic == "cluster/point"

    def change_state(self,state):
        self.state = state

    def has_pending_jobs(self):
        return len(self.jobs) > 0

    def get_next_job(self):
        self.num_jobs_handled += 1
        job = self.jobs[0]
        self.jobs = self.jobs[1:]
        self.finished_jobs.append(job)

        if job.data:
            return self.get_next_job()
        else:
            return job

    def get_num_jobs(self):
        return len(self.jobs)

    def calculate_color(self, color_rgb):
        hsv_color = colorsys.rgb_to_hsv(color_rgb.r, color_rgb.g, color_rgb.b)
        angle = hsv_color[0] * 360
        if angle > 330 or angle < 15:
            return 'red'
        elif angle < 70:
            return 'yellow'
        elif angle < 160:
            return 'green'
        else:
            return 'blue'

    def sort_jobs(self, gains):
        with_gains = [(job, gains.get(job.color)) for job in self.jobs]
        s = sorted(with_gains, key=lambda x: x[1], reverse=True)
        sorted_jobs = [job[0] for job in s]

        self.jobs = sorted_jobs
        self.jobs_calculated = True

    def reset_cluster(self, cluster_ix):
        cluster = self.centers[cluster_ix]
        reseted_cluster = ClusterPoint(cluster.x, cluster.y, 1, True)
        self.centers[cluster_ix] = reseted_cluster

    def find_nearest_cluster(self, p):
        closest_center = None
        min_dist = 999999999
        min_ix = 0


        for center_ix, center in enumerate(self.centers):
            dist = point_distance(p, center)
            if dist < min_dist:
                if closest_center:
                    print("Detected in multiple centers")

                closest_center = center
                min_dist = dist
                min_ix = center_ix

        return closest_center, min_ix

    def point_callback(self, marker):
        p = marker.pose.position
        closest_center, min_ix = self.find_nearest_cluster(p)
        color = marker.color
        if color.r == 0 and color.g == 0 and color.b == 0:
            color, discrete_color = None, None
        else:
            discrete_color = self.calculate_color(color)
        data = json.loads(marker.text) if marker.text else None

        if closest_center:
            print("[Cluster] updating existing cluster")
            new_center = closest_center.move_center(p, data, color, discrete_color)
            self.centers[min_ix] = new_center

            if new_center.n >= self.min_center_distance and not new_center.is_visited:
                new_center.is_visited = True
                self.centers[min_ix] = new_center
                self.jobs.append(new_center)
        else:
            discrete_colors = {discrete_color: 1} if discrete_color else {}
            self.centers.append(ClusterPoint(p.x, p.y, 1,False, color, discrete_colors, data))
            print("[Cluster] Adding new center")

        print(self.centers)
        self.publish_markers()

    def publish_markers(self):
        by_n = sorted(self.centers, key=lambda center: center.n, reverse=True)
        markers = [self.point_2_marker(p, ix)
                   for (ix, p) in enumerate(by_n[:3])]
        self.markers_pub.publish(markers)

    def point_2_marker(self, data_point, ix):
        pose = Pose()
        pose.position.x = data_point.x
        pose.position.y = data_point.y
        pose.position.z = 0.5
        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = "map"
        marker.pose = pose
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(30)
        marker.id = ix
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = data_point.color
        return marker