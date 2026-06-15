import os
import shutil

camera_list = ["realsense", "orbbec"]

org_data_root = "data_org"
processed_data_root = "data"

os.makedirs(os.path.join(processed_data_root, "Images"), exist_ok=True)
os.makedirs(os.path.join(processed_data_root, "Depth"), exist_ok=True)

for camera in camera_list:
    org_data_path = os.path.join(org_data_root, camera)
    if not os.path.exists(org_data_path):
        print("Camera data path does not exist:", org_data_path)
        continue
    else:
        print("Camera data path exists:", org_data_path)
        # Proceed with organizing data for this camera
        for root, dirs, files in os.walk(org_data_path):
            for file in files:
                if file.endswith(".jpg") or file.endswith(".png"):
                    # Copy image files to the Images directory
                    shutil.copy(os.path.join(root, file), os.path.join(processed_data_root, "Images", file))
                elif file.endswith(".npy"):
                    # Copy depth files to the Depth directory
                    shutil.copy(os.path.join(root, file), os.path.join(processed_data_root, "Depth", file))
# Print a message indicating completion of data organization
print("Data organization completed successfully.")
print("Length of Images directory:", len(os.listdir(os.path.join(processed_data_root, "Images"))))
print("Length of Depth directory:", len(os.listdir(os.path.join(processed_data_root, "Depth"))))
