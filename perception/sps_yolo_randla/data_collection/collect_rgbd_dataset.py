      
import cv2
import numpy as np
import os
import argparse
import time
import pyrealsense2 as rs  # For Intel RealSense
import sys

sys.path.append('D:/software/pyorbbecsdk/install/lib')
import pyorbbecsdk as ob  # For Orbbec cameras
from pyorbbecsdk import OBFormat

# from utils import frame_to_rgb_image

MIN_DEPTH = 20  # 20mm
MAX_DEPTH = 1000  # 10000mm


class TemporalFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.previous_frame = None

    def process(self, frame):
        if self.previous_frame is None:
            result = frame
        else:
            result = cv2.addWeighted(frame, self.alpha, self.previous_frame, 1 - self.alpha, 0)
        self.previous_frame = result
        return result


temporal_filter = TemporalFilter(alpha=0.6)


class DepthCamera:
    def __init__(self, camera_type='orbbec'):
        self.camera_type = camera_type
        self.is_running = False

        if camera_type == 'realsense':
            self.init_realsense()
        elif camera_type == 'orbbec':
            self.init_orbbec()
        else:
            raise ValueError(f"Unsupported camera type: {camera_type}")

    def init_realsense(self):
        """Initialize Intel RealSense camera"""
        print("Initializing Intel RealSense camera...")
        self.pipeline = rs.pipeline()
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

        # Start streaming
        rs_profile = self.pipeline.start(config)
        depth_profile = next(s for s in rs_profile.get_streams() if s.stream_type() == rs.stream.depth)

        # Read depth stream intrinsics.
        depth_intrinsics = depth_profile.as_video_stream_profile().get_intrinsics()

        # Print intrinsics for dataset records.
        print("Depth camera intrinsics:")
        print(f"width: {depth_intrinsics.width}")
        print(f"height: {depth_intrinsics.height}")
        print(f"principal point (cx, cy): ({depth_intrinsics.ppx}, {depth_intrinsics.ppy})")
        print(f"focal length (fx, fy): ({depth_intrinsics.fx}, {depth_intrinsics.fy})")
        print(f"distortion model: {depth_intrinsics.model}")
        print(f"distortion coefficients: {depth_intrinsics.coeffs}")
        self.is_running = True
        print("RealSense camera ready.")

    def init_orbbec(self):
        """Initialize Orbbec camera"""
        print("Initializing Orbbec camera...")
        # Initialize the context
        context = ob.Context()

        # Get device list
        device_list = context.query_devices()
        if device_list.get_count() == 0:
            raise RuntimeError("No Orbbec devices found!")

        # Get device and create pipeline
        device = device_list.get_device_by_index(0)
        device_info = device.get_device_info()
        print("Device Info:", device_info)
        # print_device_info(device_info)

        self.pipeline = ob.Pipeline(device)

        # Configure streams
        config = ob.Config()
        profile_list = self.pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
        print("Profile List:", profile_list)
        color_profile = profile_list.get_video_stream_profile(640, 480, ob.OBFormat.RGB, 30)
        config.enable_stream(color_profile)

        depth_profile_list = self.pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
        depth_profile = depth_profile_list.get_video_stream_profile(640, 480, ob.OBFormat.Y16, 30)
        depth_intrinsics = depth_profile.get_intrinsic()
        print("depth_intrinsics  {}".format(depth_intrinsics))
        config.enable_stream(depth_profile)

        # Start pipeline
        self.pipeline.enable_frame_sync()
        self.pipeline.start(config)
        self.is_running = True
        self.align_filter = ob.AlignFilter(align_to_stream=ob.OBStreamType.COLOR_STREAM)
        # self.point_cloud_filter = ob.PointCloudFilter()
        print("Orbbec camera ready.")

    def get_frames(self):
        """Get color and depth frames"""
        if not self.is_running:
            raise RuntimeError("Camera is not running!")

        if self.camera_type == 'realsense':
            return self.get_realsense_frames()
        else:
            return self.get_orbbec_frames()

    def get_realsense_frames(self):
        """Get frames from RealSense camera"""
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return None, None

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        # color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
        depth_image = np.asanyarray(depth_frame.get_data())

        return color_image, depth_image

    def get_orbbec_frames(self):
        """Get frames from Orbbec camera"""
        frames = self.pipeline.wait_for_frames(100)
        if frames is None:
            return None, None
        # color_frame = frames.get_color_frame()
        # depth_frame = frames.get_depth_frame()
        # if not color_frame or not depth_frame:
        #     return None, None

        aligned_frames = self.align_filter.process(frames)
        if not aligned_frames:
            return None, None
        aligned_frames = frames.as_frame_set()
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if color_frame is None or depth_frame is None:
            return None, None

        # point_format = ob.OBFormat.RGB_POINT if color_frame is not None else OBFormat.POINT
        # self.point_cloud_filter.set_create_point_format(point_format)

        # point_cloud_frame = self.point_cloud_filter.process(frame)
        # ob.save_point_cloud_to_ply("point_cloud.ply", point_cloud_frame)
        # print(point_cloud_frame.get_height(), point_cloud_frame.get_width())

        # Convert to numpy arrays
        color_image = np.frombuffer(color_frame.get_data(), dtype=np.uint8)
        color_image = color_image.reshape((color_frame.get_height(), color_frame.get_width(), 3))
        # color_image = frame_to_rgb_image(color_frame)
        # print("color_image shape: ", color_image.shape)
        try:
            depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape(
                (depth_frame.get_height(), depth_frame.get_width()))
        except ValueError:
            print("Failed to reshape depth data")
        depth_data = depth_data.astype(np.float32) * depth_frame.get_depth_scale()
        depth_data = np.where((depth_data > MIN_DEPTH) & (depth_data < MAX_DEPTH), depth_data, 0)
        depth_data = depth_data.astype(np.uint16)
        # print("depth_data shape: ", depth_data.shape)
        # depth_data = temporal_filter.process(depth_data)
        # vis_depth_image = cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX)
        # vis_depth_image = cv2.applyColorMap(depth_image.astype(np.uint8), cv2.COLORMAP_JET)
        # vis_depth_image = cv2.addWeighted(color_image, 0.5, depth_image, 0.5, 0)

        return color_image, depth_data

    def stop(self):
        """Stop the camera"""
        if self.is_running:
            self.pipeline.stop()
            self.is_running = False
            print(f"{self.camera_type.capitalize()} camera stopped.")


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Depth Camera Data Collection')
    parser.add_argument('--camera', type=str, default='realsense',
                        choices=['realsense', 'orbbec'],
                        help='Camera type: realsense or orbbec')
    parser.add_argument('--output', type=str, default='data',
                        help='Output directory for collected data')
    args = parser.parse_args()

    # Create output directories
    rgb_dir = os.path.join(args.output, 'rgb')
    depth_dir = os.path.join(args.output, 'depth')
    os.makedirs(rgb_dir, exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)

    # Initialize camera
    print(f"Starting {args.camera} camera...")
    camera = DepthCamera(args.camera)

    # Create display window
    # cv2.namedWindow('Camera Feed')
    cv2.namedWindow('Camera Feed', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('Depth Visualization', cv2.WINDOW_AUTOSIZE)

    print("Press SPACE to capture image, ESC to exit")
    count = 0

    try:
        while True:
            color_frame, depth_frame = camera.get_frames()

            if color_frame is None or depth_frame is None:
                continue

            # Display color image
            if args.camera == 'orbbec':
                cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB, color_frame)
            cv2.imshow('Camera Feed', color_frame)

            # Normalize depth for visualization
            depth_vis = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            cv2.imshow('Depth Visualization', depth_vis)

            # Check for key press
            key = cv2.waitKey(1)

            if key == 27:  # ESC key
                break
            elif key == 32:  # SPACE key
                # Generate timestamp-based filename
                timestamp = int(time.time() * 1000)

                # Add camera type to the filename to distinguish between realsense and orbbec
                camera_prefix = f"{args.camera}_"

                rgb_filename = os.path.join(rgb_dir, f'{camera_prefix}{timestamp}_rgb.png')
                depth_filename = os.path.join(depth_dir, f'{camera_prefix}{timestamp}_depth.npy')

                # Save images
                cv2.imwrite(rgb_filename, color_frame)
                np.save(depth_filename, depth_frame)

                print(f"Captured frame {count}: {rgb_filename}, {depth_filename}")
                count += 1
    finally:
        # Clean up
        camera.stop()
        cv2.destroyAllWindows()
        print(f"Data collection complete. Saved {count} frames to {args.output}")


if __name__ == "__main__":
    main()

    
