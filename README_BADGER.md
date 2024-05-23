# Braindump on Badger's Evaluation of the LIO-Livox Algorithm

  *by Charles Cochran*

## TL;DR

This library works well, but it's CPU-expensive, and we don't need it right
now.

## Algorithm Overview

This algorithm subscribes to a lidar pointcloud topic (and, optionally, an IMU
topic), does math, and publishes odometry messages (in several formats)
describing the motion of the lidar.

To clarify, this algorithm does *not* build a 3D map based on the input
pointcloud data, and it does *not* do loop-closure. It simply compares each
pointcloud to the previous one, identifies notable "features"
(corners/edges/surfaces), and runs a motion-tracking solver to calculate an
odometry estimate based on the difference. Thus, like both wheel odometry and
IMU-based odometry, it is prone to drifting. But its drift-rate appears to be
very slow, making it a promising means for odometry.

We could presumably use this algorithm as an additional/primary source in our
odometry EKF (though it's CPU-usage may make this impractical). Doing so would
help us solve the following problems:
- *Translational* wheel slippages.
- Wheel calibration on bots lacking a tilt lidar (if this hasn't yet been
  solved).

The algorithm's primary failure mode would involve dynamic activity in the
lidar's FOV being interpreted as sensor motion. I have so far been unable to
cause this failure.

## Running w/ a Mid-360

As can be seen in this branch, we must first fix several compilation errors
and SEGVs (the library is not very well maintained). I submitted these fixes
in [an upstream PR](https://github.com/Livox-SDK/LIO-Livox/pull/78), but the
maintainer isn't very active so I doubt they'll be accepted.

I then tested the library with a Mid-360 (which requires getting the driver in
our build; see the `livox-wip` branch of `bar_truck`) using the persistent
pointcloud test described below. I found it to be quite promising, so I
decided to try to get it running on our Hesai.

## Running w/ a Hesai QT128

### APPC Ring Filter

The algorithm expects data from the Mid-360's 4 lasers, but the Hesai has 128
lasers. Thus, I wrote a custom `PCL` filter in APPC to filter out specified
rings (see the `lio-livox` branches of `ant_perceive_point_cloud` and
`bar_launchers`). The filter takes which rings to *keep* as a ROS parameter.

For initial testing, I gave it 16 spread-apart rings with a 0.4-deg horizontal
resolution. See below for further discussion of ring selection.

### Driver

Next we need a driver (`lidarCallBackHesai()` in `ScanRegistration.cpp`) to
run the algorithm with the Hesai data.

The first thing the driver needs to do is transform the input pointcloud from
the tilted `top_front` TF frame to the `base_footprint` frame. This will allow
the algorithm to estimate the motion of `base_footprint` relative to a flat
"world" frame instead of tracking the motion of the top-front lidar through a
tilted "world" frame.

The algorithm will bin the points by ring to prepare for "feature extraction"
(identification of corners/edges/surfaces within each ring). The Mid-360 code
assumes a particular order for its input point data (ring 0, ring 1, ring 2,
ring 3, ring 0, etc.) and sets the `XYZINormal` pointcloud's `normal_y` field
to the ring number (which will be used as a bin index). My driver replicates
this by assigning each unique ring number a particular `normal_y` value
between 0 and N and setting each point's `normal_y` based on its ring number.
This avoids an assumption about the ring values or the order of the input
points.

### Config

The Hesai config/launch files mirror the Mid-360 files. The `Used_Line`
parameter in the config file indicates the number of scan lines being used; it
must match the number of kept rings in aforementioned APPC filter. The launch
file has an `IMU_Mode` parameter; setting it to `0` will disable the IMU (see
below for more information).

Now we should be able to run the algorithm with the Hesai QT128, allowing us
to test its effectiveness.

## Testing

The algorithm publishes a `nav_msgs/Odometry` topic (`/livox_odometry_mapped`)
attempting to track the motion of the sensor producing the input pointcloud
through a fixed "world" frame, resetting to a "zero" position upon startup. We
can test the odometry accuracy in a few different ways.

### Persistent Pointcloud Test

Mirroring the `base_footprint` -> `odom_wheels` reverse-transform, let's
invert the algorithm's calculated odometry transform and publish a
reverse-transform from `base_footprint` to a new frame, `odom_livox` (recall
that we previously transformed the input pointcloud into the `base_footprint`
frame). Then, in Rviz, we can set the fixed frame to `odom_livox` and
visualize a persistent (`Decay Time = 9999`), throttled pointcloud from one of
the lidars as we drive the bot around. The sharpness/coherency of the
resulting cloud is a measure of how well the odometry algorithm is performing.

Doing the above, I found very promising results. Driving the robot all the way
around the lab resulted in an essentially perfect cloud alignment, indicating
minimal drift. For comparison with our wheel odometry, repeating the test with
`odom_wheels` as the fixed frame produced a drastic alignment error.

### Odometry Arrow Divergence Test (WIP)

Visualizing the unmodified `Odometry` message published by the algorithm isn't
particularly useful, as it resets to the origin upon startup (unlike our
`/odom` message). Upon startup, however, we can store the transform between
`map` and `odom`. Then we can add this to each odometry message before
publishing it (on `/livox_odometry_mapped2`). If we visualize this topic (with
the `/odom` frame fixed in Rviz), we'll find that its arrows line up with the
`/odom` arrows upon startup. As the robot drives around, the divergence
between the two arrows should indicate the difference between the algorithm
and our current odometry.

Warning: This test was never finished and is probably broken, so YMMV. But I'm
leaving it in because the idea is probably useful.

### imu-test

The [imu-test2](https://github.com/BadgerTechnologies/imu-test) tool could
easily be extended to compare the angular odometry of this algorithm with the
localizer, gyroscope, and wheel odometry.

## Running w/ the IMU (WIP)

See the [upstream
README](https://github.com/Livox-SDK/LIO-Livox/blob/master/README.md) for a
vague description of the possible `IMU_Mode`s.

Modes `0` and `1` have to do with correcting lidar rotation distortion--the
result of the lidar itself rotating (as the robot turns, for example) while
the lidar is spinning and taking measurements. Mode `0` doesn't use the gyro
at all; it corrects rotation distortion using a simple approach that assumes
the lidar is spinning at a constant velocity. Mode `1` uses the gyro to assist
in distortion correction. I haven't observed a significant difference betwen
the two modes.

Note that we could also just comment out the `RemoveLidarDistortion()` call in
`PoseEstimation.cpp` and make the APPC filter use a pointcloud we have already
motion-corrected instead of the `_raw` pointcloud topic.

Mode `2` makes heavy use of the IMU in the algorithm, first attempting to
"initialize" the IMU via a bias calibration (which we have already done by
using the `_cal` IMU topic). With the Mid-360 (and its built-in IMU), I did
not observe an algorithm accuracy difference between the modes. With the
Hesai, I have been unable to get mode `2` to successfully "initialize" an
external IMU, but it should be possible. It's difficult to see how mode `2`
could improve the algorithm's accuracy, though it could make it more resilient
against dynamic objects in the lidar's FOV. It could also increase its
effiency (by decreasing the work that the solver has to do; see below).

Note that the launch files have a `Extrinsic_Tlb` table describing a transform
between the input pointcloud frame and the input IMU frame. Since we have
translated the pointcloud to `base_footprint`, I suspect that an identity
matrix is appropriate in our case, as the `base_imu` frame (used by the
`/imu/safetbox_cal` topic) differs from the `base_footprint` only in X/Y/Z
translation.

## Optimization/Reducing CPU Usage

The algorithm is very CPU-heavy, so significant optimization work would
probably be needed to use it for real.

### Disabling Lidar Queueing

In fact, the algorithm often falls behind due to being CPU-bound. For example,
using 16 Hesai 0.4-deg rings @ 10 Hz (14400 pts/cloud), the algorithm can just
about keep-up with real-time when the sensor isn't moving (and thus the solver
isn't having to do much). It quickly "falls behind" when the sensor is moving,
however, and it handles this by buffering lidar messages in a FIFO, which
doesn't seem like a good idea! Thus, I made a quick-n-dirty hack to
`PoseEstimation.cpp`'s `process()` function to disable this queueing:

```
    if(!_lidarMsgQueue.empty()){
-      // get new lidar msg
-      time_curr_lidar = _lidarMsgQueue.front()->header.stamp.toSec();
-      pcl::fromROSMsg(*_lidarMsgQueue.front(), *laserCloudFullRes);
-      _lidarMsgQueue.pop();
+      // instead of queuing, just use the latest scan and throw away the old ones!
+      time_curr_lidar = _lidarMsgQueue.back()->header.stamp.toSec();
+      pcl::fromROSMsg(*_lidarMsgQueue.back(), *laserCloudFullRes);
+      _lidarMsgQueue = {};
       newfullCloud = true;
     }
```

With this change, the algorithm is able to "keep up" (the odometry output
frequency still drops when the sensor is moving, but it is always "current").

Note that a proper solution would involve removing the queue entirely and just
storing a reference to the latest (and previous) message.

### Feature Extraction Latency

The algorithm parallelizes the feature extraction step by creating (and
destroying) a new thread every frame to process each ring. At the moment, I'm
still doing this. But now we're using 16 rings, so this is probably
responsible for a non-trivial amount of latency. My experimentation reveals
that, even though timestamps are handled properly througout the algorithm, a
small latency in the feature extraction step can cause major delays in the
later odometry solver. Thus, rewriting this to use a threadpool and avoid
thread creation/deletion overhead may be a helpful means of overall
optimization.

### Solver Optimization

Early profiling revealed that the algorithm spends a lot of time in the Ceres
solver (the final step), which is probably why it performs worse when the
sensor is moving than when it is still (see above). Experimentation would
likely reveal many ways to increase the solver's efficiency via improved
configuration.

### Ring Configuration

Including more points in the input cloud will (potentially) increase the
algorithm's accuracy and (certainly) increase its CPU usage. Using the APPC
filter's `rings` parameter to measure the algorithm's accuracy and efficiency
with various ring configurations should help identify an optimal input size
and layout.
