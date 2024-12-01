# need image 
# need point cloud
# divide it to the segment and give the results
import matplotlib.pyplot as plt 
import matplotlib.image as img 
import numpy as np
import open3d as o3d
import cv2
# import mcap
# import time
# from mcap.writer import McapWriter
# from mcap.records import Message

def get_intrinsics(H, W, fov=47.69):
    """
    Intrinsics for a pinhole camera model.
    Assume FOV of 55 degrees and central principal point.
    
    H: Height of the image
    W: Width of the image
    fov: Field of view in degrees (default is 55)
    
    Returns:
    - 3x3 intrinsic matrix for the camera.
    """
    # Calculate the focal length using the FOV and image width
    f = 0.5 * W / np.tan(0.5 * fov * np.pi / 180.0)
    
    # Calculate the principal point (center of the image)
    cx = 0.5 * W
    cy = 0.5 * H
    
    # Return the 3x3 intrinsic matrix
    return np.array([
        [f, 0, cx],
        [0, f, cy],
        [0, 0, 1]
    ])



def pixel_to_point(depth_image, camera_intrinsics=None):
    # depth_image = depth_image[:, :, 0]
    # depth_image = np.squeeze(depth_image)
    height, width = depth_image.shape
    
    if camera_intrinsics is None:
        camera_intrinsics = get_intrinsics(height, width, fov=55.0)
    
    fx, fy = camera_intrinsics[0, 0], camera_intrinsics[1, 1]
    cx, cy = camera_intrinsics[0, 2], camera_intrinsics[1, 2]
    
    # Create u, v meshgrid and precompute projection triangle ratios
    u = np.linspace(0, width - 1, width)
    v = np.linspace(0, height - 1, height)
    u, v = np.meshgrid(u, v)
    
    # Compute x_over_z and y_over_z and ensure they are 2D
    x_over_z = (u - cx) / fx  # 2D
    y_over_z = (v - cy) / fy  # 2D
    
    # Ensure no extra dimensions
    x_over_z = np.squeeze(x_over_z)
    y_over_z = np.squeeze(y_over_z)
    
    # Calculate z using 3D Pythagoras theorem (now shapes should match)
    z = depth_image / np.sqrt(1 + x_over_z**2 + y_over_z**2)
    
    x = x_over_z * z
    y = y_over_z * z
    
    return x, y, z


def create_point_cloud(depth_image, color_image=None, camera_intrinsics=None, scale_ratio=100.0):
    """
    Create a 3D point cloud from a depth image and a color image.
    
    depth_image: 2D array containing depth values.
    color_image: 2D color image aligned with the depth image.
    camera_intrinsics: Optional 3x3 camera intrinsic matrix.
    scale_ratio: Scaling factor for the depth image (default: 100).
    
    Returns:
    - cloud: A point cloud object (Open3D format) with points and colors.
    """
    
    height, width, _ = depth_image.shape
    # print(depth_image.shape)
    
    # Get the camera intrinsics if not provided
    if camera_intrinsics is None:
        camera_intrinsics = get_intrinsics(height, width, fov=55.0)
    
    # Resize the color image to match the dimensions of the depth image
    # color_image = cv2.resize(color_image, (width, height))
    
    # Ensure that the depth image does not contain zeros (avoid invalid points)
    depth_image = np.maximum(depth_image, 1e-5)
    
    # Apply the scaling ratio to the depth image
    depth_image = scale_ratio / depth_image
    
    # Convert depth image to 3D points
    if depth_image.shape[-1] == 3:
        depth_image = depth_image[:, :, 0]

    # If it's (H, W, 1), squeeze or slice to make it (H, W)
    if len(depth_image.shape) == 3 and depth_image.shape[-1] == 1:
        depth_image = depth_image.squeeze()
    x, y, z = pixel_to_point(depth_image, camera_intrinsics)
    point_image = np.stack((x, y, z), axis=-1)
    
    # Concatenate color and depth information
    # xyzrgb_image = np.concatenate([point_image, color_image], axis=-1)
    
    # Create a point cloud using Open3D
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(point_image.reshape(-1, 3))
    # cloud.colors = o3d.utility.Vector3dVector(color_image.reshape(-1, 3) / 255.0)
    
    # Masking for outdoor sky points (optional step)
    mask = point_image[..., 2] < 2  # Filter out points that are too far away
    cloud.points = o3d.utility.Vector3dVector(point_image[mask].reshape(-1, 3))
    # cloud.colors = o3d.utility.Vector3dVector(color_image[mask].reshape(-1, 3) / 255.0)
    
    return cloud


def mcap_file_conversion(cloud):
# Open the MCAP file
    with open("output.mcap", "wb") as f:
        writer = McapWriter(f)

        # Create MCAP schema and channel for point cloud data
        writer.start()

        channel_id = writer.register_channel(
            topic="/point_cloud_topic",
            message_encoding="json",
            schema_id=None,
            metadata={"type": "sensor_msgs/PointCloud2"},
        )

        # Loop through your frames (example: point clouds) and write them
        # for frame in frames:
            # Example: Convert frame (point cloud) to the desired format (e.g., JSON or binary)
            # Here frame_data should be your point cloud in the form of bytes or JSON
        frame_data = cloud.to_json()  # Example conversion
        writer.write_message(
            Message(
                channel_id=channel_id,
                log_time=int(time.time() * 1e9),  # Time in nanoseconds
                data=frame_data.encode("utf-8")
            )
        )

        writer.finish()  # Finalize the MCAP file


def main():
    # get the image and open in matplotlib

    # testImage = img.imread('test_img.png')
    testImage = cv2.imread('test_img.png', cv2.IMREAD_UNCHANGED)
    cloud = create_point_cloud(testImage)
    o3d.visualization.draw_geometries([cloud], window_name="3D Point Cloud", width=800, height=600)
    o3d.io.write_point_cloud("my_point_cloud.pcd", cloud)
    # mcap_file_conversion(cloud)
    # plt.imshow(testImage) 
    # plt.show()
    # plt.pause(1)

if __name__ == "__main__":
    main()