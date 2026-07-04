import numpy as np
from dr_spaam.detector import Detector

ckpt = 'path_to_checkpoint'
detector = Detector(
    ckpt,
    model="DROW3",          # Or DR-SPAAM
    gpu=True,               # Use GPU
    stride=1,               # Optionally downsample scan for faster inference
    panoramic_scan=True     # Set to True if the scan covers 360 degree
)

# tell the detector field of view of the LiDAR
laser_fov_deg = 360
detector.set_laser_fov(laser_fov_deg)

# detection
num_pts = 1091
while True:
    # create a random scan
    scan = np.random.rand(num_pts)  # (N,)

    # detect person
    dets_xy, dets_cls, instance_mask = detector(scan)  # (M, 2), (M,), (N,)

    # confidence threshold
    cls_thresh = 0.5
    cls_mask = dets_cls > cls_thresh
    dets_xy = dets_xy[cls_mask]
    dets_cls = dets_cls[cls_mask]