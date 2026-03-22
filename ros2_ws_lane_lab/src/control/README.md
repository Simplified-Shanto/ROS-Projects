# control (ROS 2)

Centerline extraction and steering computation from a lane mask.

## What It Does

This package provides two nodes:

- `center_line`:
  - Subscribes to a lane mask image (mono8/mono16/32FC1).
  - Extracts the lane centerline using the same logic and defaults as `video_test.py`.
  - Publishes the centerline as a `nav_msgs/Path`.
  - Optionally publishes a visualization image.

- `pure_pursuit`:
  - Subscribes to the centerline and (optionally) the lane mask.
  - Computes steering angle using the same logic and defaults as `video_test.py`.
  - Publishes steering angle as `std_msgs/Float32` (degrees by default).

The control pipeline is:

```
/lane/mask  ->  center_line  -> /lane/centerline  ->  pure_pursuit  -> /steering_angle
```

## Topics

### Subscribed

- `/lane/mask` (`sensor_msgs/Image`): lane mask (mono8 recommended)
- `/lane/centerline` (`nav_msgs/Path`): centerline points from `center_line`

### Published

- `/lane/centerline` (`nav_msgs/Path`): output from `center_line`
- `/lane/centerline_image` (`sensor_msgs/Image`): optional overlay image
- `/steering_angle` (`std_msgs/Float32`): steering angle (deg by default)

## Parameters

### `center_line` node

Video-test compatible names (preferred):

- `centerline_threshold` (float, default `0.5`): mask threshold
- `centerline_step` (int, default `4`): row step in pixels
- `centerline_min_width` (int, default `6`): minimum contiguous mask width
- `centerline_min_pixels` (int, default `20`): minimum pixels per row
- `centerline_smooth` (int, default `7`): moving average window
- `centerline_color` (string, default `"0,255,255"`): overlay line color (R,G,B)
- `centerline_thickness` (int, default `2`): overlay line thickness

Other params:

- `mask_topic` (string, default `/lane/mask`)
- `centerline_topic` (string, default `/lane/centerline`)
- `overlay_topic` (string, default `/lane/centerline_image`)
- `publish_overlay` (bool, default `false`)
- `frame_id` (string, default `""`)

Legacy aliases are also supported: `threshold`, `step`, `min_width`, `min_pixels`, `smooth`.

### `pure_pursuit` node

Video-test compatible names (preferred):

- `steer_method` (string, default `center_offset`)
- `steer_lookahead` (float, default `0.3`)
- `steer_wheelbase` (float, default `0.25`)  
  Wheelbase as a fraction of image height (used in pure pursuit).
- `steer_wheelbase_cm` (float, default `26.7`)
- `steer_max_angle` (float, default `25.0`)
- `steer_bias_deg` (float, default `0.0`)
- `steer_deadband` (float, default `0.0`)
- `steer_smooth` (float, default `0.0`)  
  EMA smoothing factor; 0 disables.
- `lane_width_cm` (float, default `35.0`)
- `centerline_threshold` (float, default `0.5`)
- `centerline_min_width` (int, default `6`)
- `centerline_min_pixels` (int, default `20`)

Other params:

- `centerline_topic` (string, default `/lane/centerline`)
- `mask_topic` (string, default `/lane/mask`)
- `angle_topic` (string, default `/steering_angle`)
- `angle_in_degrees` (bool, default `true`)
- `image_width` / `image_height` (int, default `0`)  
  Used as a fallback if the mask has not been received yet.
- `use_mask` (bool, default `true`)

Legacy aliases are also supported: `method`, `lookahead`, `wheelbase`, `wheelbase_cm`, `mask_threshold`, `mask_min_width`, `mask_min_pixels`, `max_angle_deg`.

## Launch Files

- `center_line.launch.py`
- `pure_pursuit.launch.py`
- `control.launch.py`  
  Starts `center_line` immediately and `pure_pursuit` 1 second later.

## Build

```bash
source /opt/ros/humble/setup.bash
cd /home/pi/autodrive
colcon build --merge-install --packages-select control
source /home/pi/autodrive/install/setup.bash
```

## Run

Single nodes:

```bash
ros2 run control center_line
ros2 run control pure_pursuit
```

`center_line` executable was replaced by `path_planning/centerline` in the current workspace layout.

Launch both:

```bash
ros2 launch control control.launch.py
```

## Notes

- The steering math and centerline logic match `video_test.py` defaults.
- If you change camera/video resolution, update `image_width` and `image_height`.
