import torch
from torch.utils.data import Dataset
import pickle as pkl
import numpy as np
import os


def standardize_depth(depth):
    """
    Standardize valid depth values to zero mean and unit variance.

    Args:
        depth (HxWx1): input depth map

    Returns:
        depth_std (HxWx1): standardized depth map
    """
    valid_depth = depth[depth > 0]  # Only use valid points.
    mean = np.mean(valid_depth)
    std = np.std(valid_depth)
    depth_std = (depth - mean) / std
    depth_std[depth == 0] = 0  # Keep invalid points at zero.
    return depth_std

def rgbd_mask_to_pointcloud(rgb, depth, mask, camera_type='orbbec', downsample=False, max_points=100000, scale=1000):
    """
    Args:
        rgb (HxWx3): 0-255 or 0-1
        depth (HxW): in meters
        mask (HxW): binary or class labels
        intrinsics (3x3): camera intrinsics
        fx, fy = 374.805786, 374.805786 # orbbec
        cx, cy = 323.000031, 234.500000
        fx, fy = 385.2234802246094, 385.2234802246094
        cx, cy = 320.3030090332031, 237.64939880371094  # realsense

    Returns:
        points (Nx7): [R,G,B,X,Y,Z,Mask]
    """
    if camera_type == 'orbbec':
        intrinsics = np.array([[374.805786, 0, 323.000031],
                               [0, 374.805786, 234.500000],
                               [0, 0, 1]], dtype=np.float32)
    elif camera_type == 'realsense':
        intrinsics = np.array([[385.2234802246094, 0, 320.3030090332031],
                               [0, 385.2234802246094, 237.64939880371094],
                               [0, 0, 1]], dtype=np.float32)
    # Normalize RGB to [0, 1]
    if rgb.max() > 1.0:
        rgb = rgb.astype(np.float32) / 255.0

    # Flatten all arrays
    H, W = depth.shape[0], depth.shape[1]
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    u = u.reshape(-1)
    v = v.reshape(-1)
    # depth = np.normalize(depth, axis=None)
    # depth = standardize_depth(depth)  # Standardize depth, Normalize depth to [0, 1] if needed
    depth_flat = depth.reshape(-1)
    depth_flat = depth_flat / scale  # Convert to meters if needed
    rgb_flat = rgb.reshape(-1, 3)
    mask_flat = mask.reshape(-1)

    # Compute XYZ
    fx, fy = intrinsics[0, 0], intrinsics[1, 1]
    cx, cy = intrinsics[0, 2], intrinsics[1, 2]
    z = depth_flat
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    # Stack [R,G,B,X,Y,Z,Mask]
    # points = np.column_stack([rgb_flat, x, y, z, mask_flat])
    points = np.column_stack([x, y, z, rgb_flat, mask_flat])
    # Remove invalid points (depth=0 or NaN)
    # valid_mask = (z > 0) & ~np.isnan(z)
    valid_mask = np.ones_like(z, dtype=bool)
    points = points[valid_mask]

    # Downsample if too many points
    if downsample and len(points) > max_points:
        idx = np.random.choice(len(points), max_points, replace=False)
        points = points[idx]

    return points


    
class RGBDDataset(Dataset):
    def __init__(self, data_files, transform=None):
        """
        Dataset for RGBD data.

        Parameters
        ----------
        data_files: list of str
            List of file paths to the dataset files.
        transform: callable, optional
            A function/transform to apply to the data.
        """
        self.data_files = data_files
        self.transform = transform
        self.data = {}
        for file in self.data_files:
            data = pkl.load(open(file, 'rb'))  # Ensure files are accessible
            for key, value in data.items():
                if key not in self.data:
                    self.data[key] = []
                # print(value.shape if isinstance(value, np.ndarray) else type(value))
                self.data[key].append(value)
        for key in self.data:
            # print(len(self.data["score"]))
            # if isinstance(self.data[key][0], float):
                # print("key:", key, "length:", len(self.data[key]), "type:", type(self.data[key][0]))
            # else:
                # print("key:", key, "length:", len(self.data[key]), "shape:", self.data[key][0].shape, "type:", type(self.data[key][0]))
            self.data[key] = np.concatenate([np.array(self.data[key][i])[np.newaxis, ...] for i in range(len(self.data[key]))], axis=0).astype(np.float32)

    def __len__(self):
        # return self.data.shape[0]  # Assuming the first dimension is the batch size
        # If data is a dictionary, return the length of one of the keys
        return len(self.data[next(iter(self.data))])  

    def __getitem__(self, idx):
        data = {key: value[idx] for key, value in self.data.items()}
        # for key in data:
            # print(f"Key: {key}, Shape: {data[key].shape}, Type: {type(data[key])}")
        # raise NotImplementedError("This method should be implemented in subclasses.")
        # keys = ['head_img', 'head_depth', 'head_seg', 'score']
        # return data/
        if self.transform:
            data = self.transform(data)
        input_points = rgbd_mask_to_pointcloud(data['head_img'], data['head_depth'], data['head_seg'], downsample=True, max_points=10000)
        gt_score = data['score']
        # print(f"Input points shape: {input_points.shape}, GT score shape: {gt_score.shape}")
        input_points = torch.tensor(input_points, dtype=torch.float32)
        gt_score = torch.tensor([gt_score], dtype=torch.float32)
        return input_points, gt_score
        # return data
        # return data['head_img'], data['head_depth'], data['head_seg'], data['score']
    

# Example usage:
if __name__ == '__main__':
    # data_dict = {"train": ["/home/yangzhou/code/dp_sps1/scorer_datasets/scorer_seat_belt/dp_dataset_seg-187-9/train.pkl",
    #                             "/home/yangzhou/code/dp_sps1/scorer_datasets/scorer_striker/dp_dataset_seg-95-4/train.pkl"],
    #             "test": ["/home/yangzhou/code/dp_sps1/scorer_datasets/scorer_seat_belt/dp_dataset_seg-187-9/test.pkl",
    #                     "/home/yangzhou/code/dp_sps1/scorer_datasets/scorer_striker/dp_dataset_seg-95-4/test.pkl"],}
    data_dict = {"train":[], "test":[]}
    files_train = os.listdir("mask_score/scoredds_pklfiles/train")
    files_test = os.listdir("mask_score/scoredds_pklfiles/test")
    for file in files_train:
        data_dict['train'].append(os.path.join("mask_score/scoredds_pklfiles/train", file))
    for file in files_test:
        data_dict['test'].append(os.path.join("mask_score/scoredds_pklfiles/test", file))
    dataset = RGBDDataset(data_files=data_dict['train'])

    print(f"Dataset length: {len(dataset)}")
    for i in range(5):
        input_points, gt_score = dataset[i]
        print("value max:", input_points[:,2].max(), "min:", input_points[:,2].min())
        print(f"Input points shape: {input_points.shape}, GT score shape: {gt_score.shape}")
        print(f"Input points: {input_points[:5]}, GT score: {gt_score[:5]}")
