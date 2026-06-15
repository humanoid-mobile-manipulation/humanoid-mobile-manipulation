import argparse
import shutil
from pathlib import Path


def organize_site_rgbd_dataset(
    input_root: Path,
    output_root: Path,
    part_names: list[str],
    source_dir_keyword: str,
) -> None:
    output_root.mkdir(parents=True, exist_ok=True)

    for source_dir in sorted(input_root.iterdir()):
        if not source_dir.is_dir() or source_dir_keyword not in source_dir.name:
            continue

        for capture_dir in sorted(source_dir.iterdir()):
            rgb_dir = capture_dir / "rgb"
            depth_dir = capture_dir / "depth"
            if not rgb_dir.is_dir() or not depth_dir.is_dir():
                continue

            matched_parts = [part for part in part_names if part in capture_dir.name]
            for part in matched_parts:
                target_rgb_dir = output_root / part / "rgb"
                target_depth_dir = output_root / part / "depth"
                target_rgb_dir.mkdir(parents=True, exist_ok=True)
                target_depth_dir.mkdir(parents=True, exist_ok=True)

                for rgb_path in sorted(rgb_dir.iterdir()):
                    if rgb_path.is_file():
                        shutil.copy2(rgb_path, target_rgb_dir / rgb_path.name)

                for depth_path in sorted(depth_dir.iterdir()):
                    if depth_path.is_file():
                        shutil.copy2(depth_path, target_depth_dir / depth_path.name)

            print(f"Processed capture folder: {capture_dir.name}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Reorganize site RGB-D captures by part name.")
    parser.add_argument("--input_root", type=Path, required=True, help="Root folder containing raw capture folders.")
    parser.add_argument("--output_root", type=Path, required=True, help="Output folder for organized RGB-D data.")
    parser.add_argument(
        "--part_names",
        nargs="+",
        required=True,
        help="Part names used to match capture-folder names.",
    )
    parser.add_argument(
        "--source_dir_keyword",
        default="rack",
        help="Only process source folders whose names contain this keyword.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    organize_site_rgbd_dataset(
        input_root=args.input_root,
        output_root=args.output_root,
        part_names=args.part_names,
        source_dir_keyword=args.source_dir_keyword,
    )
