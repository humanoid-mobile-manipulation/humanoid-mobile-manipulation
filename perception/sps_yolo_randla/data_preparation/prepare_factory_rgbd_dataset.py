import os
import shutil

dirs = os.listdir("./data_from_hefei")
part_names = ["cap", "plastic", "trapezoid", "curved", "human_shape"]
source_dir_keyword = "rack"
os.makedirs("./data_from_hefei/dealed_data_roboflow_biaozhu/rgb", exist_ok=True)
os.makedirs("./data_from_hefei/dealed_data_roboflow_biaozhu/depth", exist_ok=True)
for dir in dirs:
    if source_dir_keyword not in dir:
        continue
    all_files = os.listdir(os.path.join("./data_from_hefei", dir))
    for file in all_files:
        imgs = os.listdir(os.path.join("./data_from_hefei", dir, file, "rgb"))
        depths = os.listdir(os.path.join("./data_from_hefei", dir, file, "depth"))
        for component in part_names:
            if component in file:
                component_dir = os.path.join("./data_from_hefei/dealed_data_roboflow_biaozhu", component, "rgb")
                os.makedirs(component_dir, exist_ok=True)
                component_depth_dir = os.path.join("./data_from_hefei/dealed_data_roboflow_biaozhu", component, "depth")
                os.makedirs(component_depth_dir, exist_ok=True)
                for img in imgs:
                    shutil.copy(os.path.join("./data_from_hefei", dir, file, "rgb", img), 
                                os.path.join("./data_from_hefei/dealed_data_roboflow_biaozhu", component, "rgb", img))
                for depth in depths:
                    shutil.copy(os.path.join("./data_from_hefei", dir, file, "depth", depth), 
                                os.path.join("./data_from_hefei/dealed_data_roboflow_biaozhu", component, "depth", depth))
        print(f"Processed {file} in {dir}")
    print(f"Processed directory: {dir}")
print("Data organization completed successfully.")
