# SPS YOLOv12/RandLA-Net Perception Pipeline

This directory contains the paper-specific reference code for the SPS learning-based
visual perception pipeline. It covers RGB-D data capture, YOLO segmentation
training, candidate-mask score labeling, RandLA-Net score training, and an
end-to-end inference/deployment reference.

The scripts use configurable dataset, checkpoint, and camera-calibration paths
so they can be adapted to a local setup.

This module corresponds to the paper's SPS learning-based visual perception
pipeline in Sec. III-C. It is separate from `imitation_learning_framework/`:
the perception pipeline selects the target-part candidate that downstream
manipulation or IL policy code can consume, but it is not itself an IL
policy-training module.

In the full SPS workflow, this module provides the learning-based visual target
selection stage. The downstream learning-based manipulation policy is trained
and deployed through `imitation_learning_framework/IL/`.

## Pipeline

1. Collect RGB-D images of all relevant parts under different lighting conditions
   and viewpoints.
2. Label part masks by part type/name with LabelMe, Roboflow, or another YOLO
   segmentation-compatible annotation tool.
3. Train an Ultralytics YOLO segmentation model.
4. Run the mask-score labeler to assign a graspability score to each detected
   candidate mask.
5. Convert scored masks plus depth maps into per-mask RGB-D point-cloud samples.
6. Train the RandLA-Net scorer to predict the manually labeled graspability score.
7. Run the whole-image inference script to segment candidates, score each mask,
   and select the highest-scoring candidate.
8. Pass the selected candidate to the downstream SPS manipulation or
   imitation-learning policy interface.

## Environment Setup

A local deployment should provide camera launch files, a Python environment,
CUDA or CPU wheel selection, ROS workspace paths, and calibrated camera
intrinsics.

## Files

| Path | Purpose |
| --- | --- |
| `data_collection/collect_rgbd_dataset.py` | RGB-D collection script for RealSense and Orbbec cameras. |
| `data_preparation/prepare_rgbd_dataset.py` | Utility for reorganizing raw RGB/depth captures. |
| `data_preparation/prepare_site_rgbd_dataset.py` | Utility for reorganizing site-capture folders. |
| `yolo_segmentation/train_yolo_segmentation.py` | Ultralytics YOLO segmentation training entry point. |
| `mask_score/candidate_mask_score_labeler.py` | PyQt tool for scoring detected YOLO masks. |
| `mask_score/build_mask_score_dataset.py` | Converts scored masks and depth maps into train/test PKL samples. |
| `mask_score/rgbd_mask_score_dataset.py` | RGB-D/mask to point-cloud conversion and `RGBDDataset`. |
| `mask_score/randla_score_model.py` | RandLA-Net scorer used in the paper pipeline. |
| `mask_score/pyg_randla_score_model.py` | PyG-based RandLA-Net variant/reference implementation. |
| `mask_score/train_mask_score_model.py` | RandLA-Net score training script. |
| `mask_score/visualize_score_predictions.py` | Score-model visualization/testing script. |
| `inference/run_yolo_randla_inference.py` | Whole-image YOLO + RandLA-Net candidate scoring demo. |
| `deployment_reference/ros_grasp_pose_service_reference.py` | ROS deployment reference that connects segmentation/scoring with the grasp service. This is not required for the paper's reported RandLA-Net ablation statistics. |

## YOLO Segmentation Training

The segmentation stage uses Ultralytics YOLO instance segmentation. The
official Ultralytics segmentation task guide documents custom dataset format,
training, validation, prediction, and export:

https://docs.ultralytics.com/tasks/segment/

Prepare a YOLO segmentation dataset with a `data.yaml`, then run:

```bash
cd perception/sps_yolo_randla
python3 yolo_segmentation/train_yolo_segmentation.py
```

The public example default uses:

- pretrained/checkpoint path: `ckpts/yolo12_seg.pt`
- dataset YAML: `sample_part_segmentation/data.yaml`
- epochs: `100`
- image size: `640`
- batch size: `16`
- optimizer: `SGD`
- initial learning rate: `0.001`
- early stopping patience: `30`

Edit the script variables after adapting it to your local dataset layout.

Recommended public data preparation steps:

1. Capture synchronized RGB and depth frames for each part category under
   multiple lighting conditions and viewpoints.
2. Annotate instance masks by part type or part name with LabelMe, Roboflow, or
   another tool that can export YOLO segmentation labels.
3. Store the dataset in YOLO segmentation format and point the training script
   to its `data.yaml`.
4. Store raw datasets and trained weights in local data directories and point
   the scripts to those paths.

## RandLA-Net Scorer Training

The scorer uses one candidate mask as one training sample. Each sample contains:

- `head_img`: RGB image
- `head_depth`: aligned depth map
- `head_seg`: one binary candidate mask
- `score`: manually assigned graspability score

The point-cloud feature for each candidate follows the paper definition:

```text
[x, y, z, r, g, b, mask]
```

The historical training defaults in `mask_score/train_mask_score_model.py` are:

- input dimension: `7`
- nearest neighbors: `16`
- decimation: `4`
- batch size: `4`
- epochs: `100`
- optimizer: Adam
- learning rate: `0.001`
- scheduler: cosine annealing, `eta_min=1e-5`
- loss: MSE between `sigmoid(predicted_score)` and the manual score
- train/evaluation folders: `mask_score/scoredds_pklfiles/train` and
  `mask_score/scoredds_pklfiles/test`

Run:

```bash
cd perception/sps_yolo_randla
python3 mask_score/build_mask_score_dataset.py --data_root mask_score/data_org_files --output_root mask_score/scoredds_pklfiles --train_ratio 0.7
python3 mask_score/train_mask_score_model.py
```

## Dataset Statistics for the Paper Text

For the ablation sentence:

```text
The evaluation is conducted on [XX] candidate-part samples collected from [XX]
scenes and 9 part categories. The dataset is split into training, validation,
and test sets with a ratio of [XX/XX/XX].
```

use the following definitions:

- candidate-part samples: number of generated per-mask PKL samples;
- scenes: number of original RGB-D frames before splitting into candidate masks;
- categories: 9, matching the paper;
- split: the actual split used by your generated PKL folders.

The recovered training script currently uses a train/test split, not a distinct
train/validation/test split. If no separate validation set was used, report it as
a training/held-out evaluation split instead of claiming three splits.

## Notes

- Model weights and datasets are not included.
- Camera intrinsics in the scripts are historical defaults and must be updated
  for a new camera calibration.
- `deployment_reference/` is provided to document how the perception output was
  connected in a ROS deployment. The paper's SPS perception ablation only needs
  the YOLO segmentation and RandLA-Net scoring path above.
