import os
import pickle
import numpy as np
import shutil
import random
from tqdm import tqdm
import argparse


def process_data_splits(data_root, output_root, train_ratio=0.7):
    """Process all data_split folders and generate PKL samples."""
    # Create output directories.
    train_dir = os.path.join(output_root, "train")
    test_dir = os.path.join(output_root, "test")
    os.makedirs(train_dir, exist_ok=True)
    os.makedirs(test_dir, exist_ok=True)

    # Collect raw PKL and depth files.
    all_pkl_files = []
    all_depth_files = []

    print("Scanning data folders...")
    # Iterate over all data_split folders.
    for split_idx in range(1, 6):
        split_dir = os.path.join(data_root, f"data_split_{split_idx}")
        if not os.path.exists(split_dir):
            print(f"Warning: folder {split_dir} does not exist; skipping")
            continue

        # Read PKL files.
        pkl_dir = os.path.join(split_dir, "data", "pkl")
        if os.path.exists(pkl_dir):
            for file in os.listdir(pkl_dir):
                if file.endswith(".pkl") and "rgb_scored" in file:
                    all_pkl_files.append(os.path.join(pkl_dir, file))

        # Read depth files.
        depth_dir = os.path.join(split_dir, "data", "depth")
        if os.path.exists(depth_dir):
            for file in os.listdir(depth_dir):
                if file.endswith(".npy") and "_depth" in file:
                    all_depth_files.append(os.path.join(depth_dir, file))

    print(f"Found {len(all_pkl_files)} PKL files and {len(all_depth_files)} depth files")

    # Create a depth-file lookup table.
    depth_map = {}
    for depth_file in all_depth_files:
        base_name = os.path.basename(depth_file).replace("_depth.npy", "")
        depth_map[base_name] = depth_file

    # Group PKL files by timestamp.
    pkl_groups = {}
    for pkl_file in all_pkl_files:
        base_name = os.path.basename(pkl_file).replace("_rgb_scored.pkl", "")
        if base_name not in pkl_groups:
            pkl_groups[base_name] = []
        pkl_groups[base_name].append(pkl_file)

    # Split into train and held-out evaluation groups.
    all_keys = list(pkl_groups.keys())
    random.shuffle(all_keys)
    split_idx = int(len(all_keys) * train_ratio)
    train_keys = all_keys[:split_idx]
    test_keys = all_keys[split_idx:]

    print(f"Dataset split: {len(train_keys)} train groups, {len(test_keys)} evaluation groups")

    # Process train groups.
    print("\nProcessing train groups...")
    for key in tqdm(train_keys):
        process_group(key, pkl_groups[key], depth_map, train_dir)

    # Process held-out evaluation groups.
    print("\nProcessing evaluation groups...")
    for key in tqdm(test_keys):
        process_group(key, pkl_groups[key], depth_map, test_dir)

    print(f"\nProcessing complete. Results saved to: {output_root}")
    print(f"Train sample count: {len(os.listdir(train_dir))}")
    print(f"Evaluation sample count: {len(os.listdir(test_dir))}")


def process_group(base_name, pkl_files, depth_map, output_dir):
    """Process one group of related PKL files."""
    # Load the matching depth map.
    depth_path = depth_map.get(base_name)
    if depth_path is None:
        print(f"Warning: no depth file found for {base_name}; skipping")
        return

    depth_data = np.load(depth_path)

    # Process each PKL file.
    for pkl_file in pkl_files:
        try:
            with open(pkl_file, 'rb') as f:
                data = pickle.load(f)

            # Check required fields.
            if 'head_img' not in data or 'masks' not in data or 'scores' not in data:
                print(f"Warning: {pkl_file} is missing required fields; skipping")
                continue

            head_img = data['head_img']
            masks = data['masks']
            scores = data['scores']

            # Check image and depth shapes.
            if head_img.shape != (480, 640, 3):
                print(f"Warning: invalid image shape in {pkl_file}: {head_img.shape}; skipping")
                continue

            if depth_data.shape != (480, 640) and depth_data.shape != (480, 640, 1):
                print(f"Warning: invalid depth shape for {base_name}: {depth_data.shape}; skipping")
                continue

            # Process each candidate mask.
            for i, (mask, score) in enumerate(zip(masks, scores)):
                # Check mask shape.
                # if mask.shape != (480, 640):
                #     print(f"Warning: invalid mask shape in {pkl_file}, mask {i}: {mask.shape}; skipping")
                #     continue

                # Create one per-mask training sample.
                new_data = {
                    'head_img': head_img,
                    'head_depth': depth_data.reshape(480, 640) if depth_data.ndim == 3 else depth_data,
                    'head_seg': mask,
                    'score': float(score)
                }
                print(new_data['head_img'].shape, new_data['head_depth'].shape, new_data['head_seg'].shape, new_data['score'])

                # Generate output filename.
                new_filename = f"{base_name}_scoredmask_{i}.pkl"
                new_path = os.path.join(output_dir, new_filename)

                # Save the new PKL file.
                with open(new_path, 'wb') as f:
                    pickle.dump(new_data, f)

        except Exception as e:
            print(f"Error while processing {pkl_file}: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process data split folders and generate PKL samples')
    parser.add_argument('--data_root', type=str, default='./mask_score/data_org_files',
                        help='Root directory containing data_split_* folders')
    parser.add_argument('--output_root', type=str, default='mask_score/scoredds_pklfiles',
                        help='Output directory')
    parser.add_argument('--train_ratio', type=float, default=0.7,
                        help='Train split ratio')

    args = parser.parse_args()

    print(f"Data root: {args.data_root}")
    print(f"Output root: {args.output_root}")
    print(f"Train ratio: {args.train_ratio}")

    # Clear the output directory if it already exists.
    if os.path.exists(args.output_root):
        shutil.rmtree(args.output_root)

    # Process data.
    process_data_splits(args.data_root, args.output_root, args.train_ratio)
