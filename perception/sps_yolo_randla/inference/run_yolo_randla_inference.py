import os
import cv2
import numpy as np
import torch
from ultralytics import YOLO
from mask_score.randla_score_model import RandLANet
from mask_score.rgbd_mask_score_dataset import rgbd_mask_to_pointcloud, RGBDDataset
import matplotlib.pyplot as plt

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



# Initialize YOLO segmentation model.
seg_model = YOLO("runs/segment/train3/weights/best.pt")
seg_model.eval()

# Initialize RandLA-Net score model.
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
score_model = RandLANet(d_in=7, num_classes=10, num_neighbors=16, decimation=4, device=device)
score_model.load_state_dict(torch.load("run_maskscore/rgbd_score_training/checkpoints/last_model.pth")["model"])
score_model.to(device)
score_model.eval()

def save_masks(image, masks, scores, save_dir):
    """
    Save all mask visualization results.
    
    Args:
        image: original image
        masks: mask list
        scores: score list
        save_dir: output directory
    """
    os.makedirs(save_dir, exist_ok=True)
    
    # Save original image.
    cv2.imwrite(os.path.join(save_dir, "original.jpg"), image)
    
    # Save each mask.
    for i, (mask, score) in enumerate(zip(masks, scores)):
        # Save mask image.
        mask_img = (mask * 255).astype(np.uint8)
        cv2.imwrite(os.path.join(save_dir, f"mask_{i}.png"), mask_img)
        
        # Create visualization image with mask.
        vis_img = image.copy()
        
        # Draw contour.
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        color = (0, 255, 0)  # Green.
        cv2.drawContours(vis_img, contours, -1, color, 2)
        
        # Add score.
        if contours:
            cnt = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.putText(vis_img, f"Score: {score:.4f}", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # Save visualization result.
        cv2.imwrite(os.path.join(save_dir, f"mask_vis_{i}.jpg"), vis_img)

def process_image(image, depth_map=None, camera_type='realsense', save_dir=None):
    """
    Process one image: segment, score, and save results.
    
    Args:
        image: input RGB image, shape (H, W, 3)
        depth_map: aligned depth map, shape (H, W), or None
        camera_type: camera type, 'orbbec' or 'realsense'
        save_dir: output directory
    
    Returns:
        best_vis_img: image showing only the highest-score mask
        all_vis_img: image showing all masks
        masks: segmentation mask list
        scores: corresponding score list
        best_score: highest score
    """
    # Convert to uint8 format.
    if isinstance(image, torch.Tensor):
        image = image.cpu().numpy()
    if isinstance(depth_map, torch.Tensor):
        depth_map = depth_map.cpu().numpy()
    
    # Ensure image is uint8.
    if image.dtype == np.float32:
        image_uint8 = (image * 255).astype(np.uint8)
    else:
        image_uint8 = image.copy()
    
    # Convert grayscale image to RGB.
    if len(image_uint8.shape) == 2:
        image_uint8 = cv2.cvtColor(image_uint8, cv2.COLOR_GRAY2RGB)
    elif len(image_uint8.shape) == 3 and image_uint8.shape[2] == 1:
        image_uint8 = cv2.cvtColor(image_uint8, cv2.COLOR_GRAY2RGB)
    
    # YOLO segmentation prediction.
    results = seg_model.predict(image_uint8)
    
    # Return original image if no target is detected.
    if results[0].masks is None:
        print("No masks detected.")
        return image_uint8, image_uint8, [], [], 0.0
    
    # Get segmentation results.
    masks = results[0].masks.data.cpu().numpy()
    print(f"Detected {len(masks)} masks.")
    cls_idx = results[0].boxes.cls.data.cpu().numpy()
    scores = []
    best_score = 0.0
    best_mask_idx = -1
    
    # Process each mask.
    for i, mask in enumerate(masks):
        # Use a synthetic depth map if no depth input is available.
        if depth_map is None:
            fake_depth = np.ones_like(mask, dtype=np.float32) * 0.5
        else:
            fake_depth = depth_map.copy()
        
        # Generate point cloud.
        points = rgbd_mask_to_pointcloud(
            image_uint8, 
            fake_depth, 
            mask,
            camera_type=camera_type,
            downsample=True,
            max_points=50000
        )
        
        # Convert to tensor.
        points_tensor = torch.tensor(points, dtype=torch.float32).unsqueeze(0).to(device)
        
        # Predict score.
        with torch.no_grad():
            pred_score = score_model(points_tensor)
            pred_score = torch.sigmoid(pred_score).item()
        
        scores.append(pred_score)
        
        # Update highest score.
        if pred_score > best_score:
            best_score = pred_score
            best_mask_idx = i
    
    # Save all mask results.
    if save_dir and len(masks) > 0:
        save_masks(image_uint8, masks, scores, os.path.join(save_dir, "all_masks"))
    
    # Visualize results.
    best_vis_img = visualize_best_mask(image_uint8, masks, scores, best_mask_idx)
    all_vis_img = visualize_all_masks(image_uint8, masks, scores, best_mask_idx, cls_idx)
    
    return best_vis_img, all_vis_img, masks, scores, best_score

def visualize_best_mask(image, masks, scores, best_idx):
    """
    Visualize only the highest-score mask.
    
    Args:
        image: original RGB image
        masks: mask list
        scores: score list
        best_idx: index of the highest-score mask
    
    Returns:
        vis_img: visualization image
    """
    # Copy original image.
    vis_img = image.copy()
    
    # Return original image if no mask exists.
    if best_idx == -1:
        return vis_img
    
    # Get highest-score mask.
    best_mask = masks[best_idx]
    best_score = scores[best_idx]
    
    # Draw mask contour.
    contours, _ = cv2.findContours(best_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Use magenta for the highest-score mask.
    color = (255, 0, 255)  # Magenta (BGR).
    thickness = 3
    
    # Draw contour.
    cv2.drawContours(vis_img, contours, -1, color, thickness)
    
    # Add score label.
    if contours:
        # Find bounding box of the largest contour.
        cnt = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(cnt)
        
        # Add score above bounding box.
        text = f"Best Score: {best_score:.4f}"
        cv2.putText(vis_img, text, (x, y-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    
    # Add summary text.
    cv2.putText(vis_img, f"Highest Score: {best_score:.4f}", (10, 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    
    return vis_img

def visualize_all_masks(image, masks, scores, best_idx, cls_idx):
    """
    Visualize all masks and highlight the highest-score mask.
    
    Args:
        image: original RGB image
        masks: mask list
        scores: score list
        best_idx: index of the highest-score mask
    
    Returns:
        vis_img: visualization image
    """
    # Copy original image.
    vis_img = image.copy()
    
    # Return original image if no mask exists.
    if len(masks) == 0:
        return vis_img
    
    # Draw contour for each mask.
    for i, mask in enumerate(masks):
        # Find contours.
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Use magenta for the best mask and green for other masks.
        if i == best_idx:
            # color = (255, 0, 255)  # Magenta (BGR).
            color = bgr_colors[1]
            thickness = 3
        else:
            # color = (0, 255, 0)  # Green (BGR).
            color = bgr_colors[0]
            thickness = 2
        
        # Draw contour.
        cv2.drawContours(vis_img, contours, -1, color, thickness)
        
        # Add score label.
        if contours:
            # Find bounding box of the largest contour.
            cnt = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(cnt)
            
            # Add score label.
            text = f"{int(cls_idx[i])}, {scores[i]:.2f}"
            cv2.putText(vis_img, text, (x, y+h//2), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    
    # Add summary text.
    # if best_idx != -1:
    #     cv2.putText(vis_img, f"Highest Score: {scores[best_idx]:.2f}", (10, 30), 
    #                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
    
    return vis_img

def process_dataset_sample(sample, camera_type='realsense', save_dir=None):
    """
    Process one dataset sample.
    
    Args:
        sample: dataset sample dictionary
        camera_type: camera type
        save_dir: output directory
    
    Returns:
        best_vis_img: image showing only the highest-score mask
        all_vis_img: image showing all masks
        masks: segmentation mask list
        scores: corresponding score list
        best_score: highest score
    """
    # Get image and depth map.
    image = sample['head_img']
    depth_map = sample['head_depth']
    
    # Process image.
    return process_image(image, depth_map, camera_type, save_dir)

def plot_scores(scores, save_path=None):
    """
    Plot score histogram.
    
    Args:
        scores: score list
        save_path: optional output path
    """
    if not scores:
        print("No scores to plot.")
        return
    
    plt.figure(figsize=(10, 6))
    plt.hist(scores, bins=20, color='skyblue', edgecolor='black')
    plt.title('Mask Scores Distribution')
    plt.xlabel('Score')
    plt.ylabel('Frequency')
    plt.grid(axis='y', alpha=0.75)
    
    # Mark highest score.
    max_score = max(scores)
    plt.axvline(max_score, color='red', linestyle='dashed', linewidth=1)
    plt.text(max_score+0.01, plt.ylim()[1]*0.9, f'Max: {max_score:.4f}', color='red')
    
    if save_path:
        plt.savefig(save_path)
        print(f"Saved score distribution plot to {save_path}")
    else:
        plt.show()


def find_png_files(base_dirs):
    png_files = []
    for base_dir in base_dirs:
        for root, _, files in os.walk(base_dir):
            for file in files:
                if file.lower().endswith(".png"):
                    png_files.append(os.path.join(root, file))
    return png_files

# Example usage.
if __name__ == "__main__":
    # Create output directory.
    base_save_dir = "results"
    os.makedirs(base_save_dir, exist_ok=True)
    
    # Example 1: process image files.
    dirs = [
        "data/examples/rack_a",
        "data/examples/rack_b"
    ]

    image_list = find_png_files(dirs)
    for image_path in image_list:
        image = cv2.imread(image_path)
        
        # Create a save directory for the current image.
        image_name = os.path.basename(image_path).split('.')[0]
        # save_dir = os.path.join(base_save_dir, f"single_image_{image_name}")
        save_dir = os.path.join(base_save_dir, f"outputs")
        
        # Process image without a depth map.
        best_vis, all_vis, masks, scores, best_score  = process_image(
            image, 
            depth_map=None, 
            camera_type='realsense',
            save_dir=save_dir
        )
        
        # Save visualization results.
        # cv2.imwrite(os.path.join(save_dir, "best_mask_result.jpg"), best_vis)
        cv2.imwrite(os.path.join(save_dir, "all_masks_%s.png"%image_name), all_vis)
        print(f"Results saved to: {save_dir}, {image_name}")

    # Plot score distribution.
    # if scores:
    #     plot_save_path = os.path.join(save_dir, "score_distribution.png")
    #     plot_scores(scores, plot_save_path)
    
    print(f"Results saved to: {save_dir}")
    
    # Show results.
    # cv2.imshow("Best Mask Result", best_vis)
    # cv2.waitKey(0)
    # cv2.imshow("All Masks Result", all_vis)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    
    # Example 2: process dataset samples.
    # Create evaluation dataset.

    # data_dict = {"train": [], "test": []}
    # files_train = os.listdir("mask_score/scoredds_pklfiles/train")
    # files_test = os.listdir("mask_score/scoredds_pklfiles/test")
    # for file in files_train:
    #     data_dict['train'].append(os.path.join("mask_score/scoredds_pklfiles/train", file))
    # for file in files_test:
    #     data_dict['test'].append(os.path.join("mask_score/scoredds_pklfiles/test", file))
    
    # dataset = RGBDDataset(data_files=data_dict['test'])
    
    # # Process multiple samples.
    # num_samples = min(5, len(dataset))
    # for i in range(num_samples):
    #     # Create a save directory for the current sample.
    #     sample_save_dir = os.path.join(base_save_dir, f"sample_{i}")
        
    #     # Process sample.
    #     sample = dataset[i]
    #     best_vis, all_vis, masks, scores, best_score = process_dataset_sample(
    #         sample, 
    #         save_dir=sample_save_dir
    #     )
        
    #     # Save visualization results.
    #     cv2.imwrite(os.path.join(sample_save_dir, "best_mask_result.jpg"), best_vis)
    #     cv2.imwrite(os.path.join(sample_save_dir, "all_masks_result.jpg"), all_vis)
        
    #     # Plot score distribution.
    #     if scores:
    #         plot_save_path = os.path.join(sample_save_dir, "score_distribution.png")
    #         plot_scores(scores, plot_save_path)
        
    #     print(f"Sample {i} results saved to: {sample_save_dir}")
        
    #     # Show results.
    #     cv2.imshow(f"Sample {i} - Best Mask", best_vis)
    #     cv2.waitKey(500)
    #     cv2.imshow(f"Sample {i} - All Masks", all_vis)
    #     cv2.waitKey(500)
    
    # cv2.destroyAllWindows()
