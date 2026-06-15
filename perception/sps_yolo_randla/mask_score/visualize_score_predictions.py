import os
import torch
import cv2
import numpy as np
import pickle as pkl
import matplotlib.pyplot as plt
from randla_score_model import RandLANet
from torch.utils.data import Dataset
from rgbd_mask_score_dataset import rgbd_mask_to_pointcloud
# RGBD dataset variant that returns images for visualization.

def hex_list_to_bgr(hex_colors):
    """Convert list of HEX colors (#RRGGBB) to list of OpenCV BGR tuples"""
    bgr_list = []
    for hex_color in hex_colors:
        hex_color = hex_color.lstrip('#')
        rgb = tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
        bgr_list.append((rgb[2], rgb[1], rgb[0]))  # BGR
    return bgr_list


# Example colors.
colors = ["#91CAE8", "#F48892"]
bgr_colors = hex_list_to_bgr(colors)
print(bgr_colors)



class RGBDDataset(Dataset):
    def __init__(self, data_files, transform=None, return_image=False):
        """
        Dataset for RGBD data.

        Parameters
        ----------
        data_files: list of str
            List of file paths to the dataset files.
        transform: callable, optional
            A function/transform to apply to the data.
        return_image: bool, optional
            Whether to return the original image for visualization.
        """
        self.data_files = data_files
        self.transform = transform
        self.return_image = return_image
        self.data = {}
        for file in self.data_files:
            data = pkl.load(open(file, 'rb'))
            for key, value in data.items():
                if key not in self.data:
                    self.data[key] = []
                self.data[key].append(value)
        for key in self.data:
            self.data[key] = np.concatenate([np.array(self.data[key][i])[np.newaxis, ...] for i in range(len(self.data[key]))], axis=0).astype(np.float32)

    def __len__(self):
        return len(self.data[next(iter(self.data))])

    def __getitem__(self, idx):
        data = {key: value[idx] for key, value in self.data.items()}
        
        if self.transform:
            data = self.transform(data)
            
        # Keep original image for visualization.
        original_img = data['head_img'].copy()
        
        # Convert to point cloud.
        input_points = rgbd_mask_to_pointcloud(
            data['head_img'], 
            data['head_depth'], 
            data['head_seg']
        )
        
        gt_score = data['score']
        input_points = torch.tensor(input_points, dtype=torch.float32)
        gt_score = torch.tensor([gt_score], dtype=torch.float32)
        
        if self.return_image:
            # Return point cloud, score, original image, and mask.
            return input_points, gt_score, original_img, data['head_seg']
        else:
            return input_points, gt_score

def visualize_results(model, dataset, device, save_dir=None):
    """
    Visualize score-model predictions.
    
    Args:
        model: trained score model
        dataset: evaluation dataset
        device: compute device (cpu or cuda)
        save_dir: optional output directory
    """
    model.eval()
    idx = 0
    
    while idx < len(dataset):
        # Load data.
        points, gt_score, image, mask = dataset[idx]
        
        # Prepare model input.
        points = points.unsqueeze(0).to(device)  # Add batch dimension.
        # points = points.transpose(2, 1)  # Convert (B, N, C) -> (B, C, N).
        print(points.shape)
        
        # Model inference.
        with torch.no_grad():
            pred_score = model(points)
        pred_score = torch.sigmoid(pred_score)
        
        # Convert to scalar values.
        gt_score = gt_score.item()
        pred_score = pred_score.item()
        
        # Prepare visualization image.
        # Ensure image values are in the 0-255 range.
        if image.max() <= 1.0:
            vis_img = (image * 255).astype(np.uint8)
        else:
            vis_img = image.astype(np.uint8)
        
        # Overlay mask on the image in red.
        vis_img = cv2.cvtColor(vis_img,cv2.COLOR_RGB2BGR)
        mask = mask.astype(np.uint8)
        mask = mask.squeeze()
        print(vis_img.shape,mask.shape)
        vis_img[mask == 1] = vis_img[mask == 1] * 0.7 + np.array([0, 0, 255]) * 0.3
        
        # Add score text.
        text_gt = f"GT Score: {gt_score:.4f}"
        text_pred = f"Pred Score: {pred_score:.4f}"
        
        cv2.putText(vis_img, text_gt, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(vis_img, text_pred, (10, 70), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(vis_img, f"Image: {idx+1}/{len(dataset)}", (10, 110), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # Show image.
        cv2.imshow("Result", cv2.cvtColor(vis_img, cv2.COLOR_RGB2BGR))
        
        # Save result.
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
            cv2.imwrite(os.path.join(save_dir, f"result_{idx:04d}.png"), 
                       cv2.cvtColor(vis_img, cv2.COLOR_RGB2BGR))
        
        # Wait for keyboard input.
        key = cv2.waitKey(0)
        
        # Key handling.
        if key == ord('s'):  # Next image.
            idx += 1
        elif key == ord('p'):  # Previous image.
            idx = max(0, idx - 1)
        elif key == 27:  # Exit with ESC.
            break
        else:
            continue
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    data_dict = {"train": ["/home/yangzhou/code/dp_sps1/scorer_datasets/scorer_seat_belt/dp_dataset_seg-187-9/train.pkl",
                                "/home/yangzhou/code/dp_sps1/scorer_datasets/scorer_striker/dp_dataset_seg-95-4/train.pkl"],
                "test": ["/home/yangzhou/code/dp_sps1/scorer_datasets/scorer_seat_belt/dp_dataset_seg-187-9/test.pkl",
                        "/home/yangzhou/code/dp_sps1/scorer_datasets/scorer_striker/dp_dataset_seg-95-4/test.pkl"],}
    data_dict = {"train":[], "test":[]}
    files_train = os.listdir("mask_score/scoredds_pklfiles/train")
    files_test = os.listdir("mask_score/scoredds_pklfiles/test")
    for file in files_train:
        data_dict['train'].append(os.path.join("mask_score/scoredds_pklfiles/train", file))
    for file in files_test:
        data_dict['test'].append(os.path.join("mask_score/scoredds_pklfiles/test", file))
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")
    
    # Load model.
    log_dir = "run_maskscore/rgbd_score_training"
    d_in = 7  # RGBXYZ + Mask
    model = RandLANet(d_in, 10, 16, 4, device)
    model_path = os.path.join(log_dir, 'checkpoints', 'best_model.pth')
    model.load_state_dict(torch.load(model_path, map_location=device)["model"])
    model.to(device)
    
    # Create evaluation dataset and return original images.
    test_dataset = RGBDDataset(data_files=data_dict['test'], return_image=True)
    
    # Visualize results.
    visualize_results(
        model=model,
        dataset=test_dataset,
        device=device,
        save_dir=os.path.join(log_dir, "visualizations")
    )
