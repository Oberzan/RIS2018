from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import rospy
from geometry_msgs.msg import Vector3, Pose
from util import point_distance
from data import ClusterPoint
import states as states
import colorsys
from itertools import groupby
from constants import NUM_CIRCLES_TO_DETECT


class Clusterer():

    def __init__(self, cluster_topic, min_center_detections=15, expected_clusters_count=NUM_CIRCLES_TO_DETECT):
        self.centers = []
        self.markers = MarkerArray()
        self.centroid_treshold = 0.475
        self.min_center_detections = min_center_detections
        self.jobs = []
        self.jobs_created = 0
        self.state = states.DEFAULT
        self.jobs_calculated = False
        self.topic = cluster_topic
        self.num_jobs_handled = 0
        self.data_detected = False
        self.expected_clusters_count = expected_clusters_count

        self.visualization_colors = [ColorRGBA(255, 0, 0, 1), ColorRGBA(
            255, 255, 0, 1), ColorRGBA(0, 0, 255, 1)]

        self.markers_pub = rospy.Publisher(
            'markers', MarkerArray, queue_size=1000)

        _ = rospy.Subscriber(cluster_topic, Marker, self.point_callback)

    def get_best_finished_jobs(self):
        with_data = [center for center in self.centers if center.data != None]
        best_jobs = sorted(with_data, key=lambda item: sum(item.data.values()), reverse=True)[:7]
        print("BEST JOBS:")
        print(best_jobs)
        return best_jobs

    def is_circle_cluster(self):
        return self.topic == "cluster/point"

    def reset_is_data_detected(self):
        self.data_detected = False

    def get_num_jobs_with_data(self):
        return len([center for center in self.centers if center.data != None])

    def create_next_job(self):
        without_data = self.get_jobs_with_no_data()
        most_ns = sorted(without_data, key=lambda center: center.n, reverse=True)
        if len(most_ns) > 0:
            self.jobs.append(most_ns[0])
            print("Creating next job!!")
        else:
            print("Create next job failed!!!!")

    def get_jobs_with_no_data(self):
        return [center for center in self.centers if center.data == None]

    def change_state(self, state):
        self.state = state

    def has_pending_jobs(self):
        return len(self.jobs) > 0

    def get_next_job(self):
        self.num_jobs_handled += 1
        job = self.jobs[0]
        self.jobs = self.jobs[1:]
        print("Num jobs handled: ", self.num_jobs_handled)
        if job.data:
            if sum(job.data.values()) < 10:
                print("Less than 10 warped images job!!")
                return job

            print("JOB HAS DATA - GETTING NEXT job")
            print(job)
            if self.has_pending_jobs():
                return self.get_next_job()
            else:
                return None
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

    def get_best_cylinders(self, data_points):
        data_points = sorted(data_points, key=lambda item: item.get_discrete_color())
        groups = groupby(data_points, lambda data_point: (data_point.get_discrete_color()))

        l = []
        for color, group in groups:
            max_n = max(group, key=lambda item: item.n)
            l.append(max_n)
        return l

    def sort_jobs(self, gains):
        print("---------SORTING JOBS----------")
        best_candidates = self.get_best_cylinders(self.centers)

        print("BEST CANDIDATES: ", best_candidates)

        with_gains = [(job, gains.get(job.get_discrete_color())) for job in best_candidates]
        print("WITH GAINS: ", with_gains)
        s = sorted(with_gains, key=lambda x: x[1], reverse=True)
        sorted_jobs = [job[0] for job in s]

        print("Sorted jobs: ", sorted_jobs)

        self.jobs = sorted_jobs
        self.jobs_calculated = True

    def reset_cluster(self, cluster_ix):
        cluster = self.centers[cluster_ix]
        reseted_cluster = cluster.reset_cluster_point()
        self.centers[cluster_ix] = reseted_cluster

    def find_nearest_cluster(self, p, centroid_threshold=None):
        closest_center = None
        min_dist = 999999999
        min_ix = 0

        for center_ix, center in enumerate(self.centers):
            dist = point_distance(p, center)
            if dist < min_dist:
                if not centroid_threshold or (centroid_threshold and dist < centroid_threshold):
                    if closest_center:
                        print("Detected in multiple centers")

                    closest_center = center
                    min_dist = dist
                    min_ix = center_ix

        return closest_center, min_ix

    def point_callback(self, marker):
        ## if self.state != states.OBSERVING:
        ##    return
        p = marker.pose.position
        closest_center = None
        min_dist = 999999999
        min_ix = 0

        for center_ix, center in enumerate(self.centers):
            dist = point_distance(p, center)
            if dist < min_dist and dist < self.centroid_treshold:
                if closest_center:
                    print("Detected in multiple centers")

                closest_center = center
                min_dist = dist
                min_ix = center_ix

        color = marker.color
        if color.r == 0 and color.g == 0 and color.b == 0:
            color, discrete_color = None, None
        else:
            discrete_color = self.calculate_color(color)

        if marker.text:
            self.data_detected = True

        if closest_center:
            print("[Cluster] updating existing cluster")
            new_center = closest_center.move_center(p, marker.text, color, discrete_color)

            self.centers[min_ix] = new_center

            if new_center.n >= self.min_center_detections and not new_center.is_visited:
                new_center.is_visited = True
                self.centers[min_ix] = new_center
                if self.is_circle_cluster():
                    self.jobs.append(new_center)
        else:
            data = {marker.text: 1} if marker.text else None
            discrete_colors = {discrete_color: 1} if discrete_color else {}
            self.centers.append(ClusterPoint(p.x, p.y, 1, False, color, discrete_colors, data))
            print("[Cluster] Adding new center")

        if self.is_circle_cluster():
            print("--------CIRCLES----------")
        else:
            print("--------CYLINDERS----------")

        for center in self.centers:
            print(center)
        print("\n\n")
        self.publish_markers()

    def publish_markers(self):
        candidates = [center for center in self.centers if center.get_discrete_color() != None]
        by_n = sorted(candidates, key=lambda center: center.n, reverse=True)[:self.expected_clusters_count]
        markers = [self.point_2_marker(p, ix) for (ix, p) in enumerate(by_n)]
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
        if self.is_circle_cluster():
            marker.type = Marker.SPHERE
            marker.id = 1000 + ix
        else:
            marker.type = Marker.CYLINDER
            marker.id = ix
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(30)
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = self.get_marker_color(data_point)
        return marker


    def get_marker_color(self, data_point):
        if data_point.get_discrete_color() == 'red':
            return ColorRGBA(1, 0, 0, 1)
        elif data_point.get_discrete_color() == 'yellow':
            return ColorRGBA(1, 1, 0, 1)
        elif data_point.get_discrete_color() == 'green':
            return ColorRGBA(0, 1, 0, 1)
        elif data_point.get_discrete_color() == 'blue':
            return ColorRGBA(0, 0, 1, 1)
        else:
            print("SHOULT NOT BE THERE!")

        return ColorRGBA(0, 0, 0, 1)