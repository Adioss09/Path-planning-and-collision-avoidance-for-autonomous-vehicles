import numpy as np
from scipy.spatial import distance

class Tracker:
    def __init__(self, max_disappeared=10, max_distance=50):
        self.next_object_id = 0
        self.objects = {} # dict of id -> list of [cx, cy, timestamp]
        self.disappeared = {}
        self.max_disappeared = max_disappeared
        self.max_distance = max_distance

    def update(self, detections, timestamp):
        """
        Updates the tracker with new detections.
        detections: list of dicts with 'center'
        Returns a list of IDs corresponding to the input detections.
        """
        input_centroids = np.zeros((len(detections), 2))
        for i, det in enumerate(detections):
            input_centroids[i] = det['center']

        if len(input_centroids) == 0:
            for obj_id in list(self.disappeared.keys()):
                self.disappeared[obj_id] += 1
                if self.disappeared[obj_id] > self.max_disappeared:
                    self.deregister(obj_id)
            return []

        if len(self.objects) == 0:
            ret_ids = []
            for i in range(len(input_centroids)):
                ret_ids.append(self.register(input_centroids[i], timestamp))
            return ret_ids

        object_ids = list(self.objects.keys())
        object_centroids = [self.objects[obj_id][-1][:2] for obj_id in object_ids]

        D = distance.cdist(np.array(object_centroids), input_centroids)

        rows = D.min(axis=1).argsort()
        cols = D.argmin(axis=1)[rows]

        used_rows = set()
        used_cols = set()

        ret_ids = [None] * len(input_centroids)

        for row, col in zip(rows, cols):
            if row in used_rows or col in used_cols:
                continue

            if D[row, col] > self.max_distance:
                continue

            obj_id = object_ids[row]
            self.objects[obj_id].append([input_centroids[col][0], input_centroids[col][1], timestamp])
            
            # Keep history short (e.g. last 10 points)
            if len(self.objects[obj_id]) > 10:
                self.objects[obj_id] = self.objects[obj_id][-10:]
                
            self.disappeared[obj_id] = 0
            ret_ids[col] = obj_id

            used_rows.add(row)
            used_cols.add(col)

        unused_rows = set(range(D.shape[0])) - used_rows
        unused_cols = set(range(D.shape[1])) - used_cols

        for row in unused_rows:
            obj_id = object_ids[row]
            self.disappeared[obj_id] += 1
            if self.disappeared[obj_id] > self.max_disappeared:
                self.deregister(obj_id)

        for col in unused_cols:
            ret_ids[col] = self.register(input_centroids[col], timestamp)

        return ret_ids

    def register(self, centroid, timestamp):
        self.objects[self.next_object_id] = [[centroid[0], centroid[1], timestamp]]
        self.disappeared[self.next_object_id] = 0
        self.next_object_id += 1
        return self.next_object_id - 1

    def deregister(self, object_id):
        del self.objects[object_id]
        del self.disappeared[object_id]
