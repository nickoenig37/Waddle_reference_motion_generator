#!/usr/bin/env python3
"""
Convert reference motion JSON files to a single pickle file.

This script reads all JSON motion files from a directory and combines them
into a single pickle file indexed by velocity commands (vx, vy, dtheta).
The resulting pickle file can be used by the PolyMotionLoader class.
"""

import argparse
import json
import pickle
from pathlib import Path

import numpy as np


def parse_step_sizes_from_filename(filename: str) -> tuple[float, float, float] | None:
    """
    Extract step sizes from filename.

    Expected format: {index}_{dx}_{dy}_{dtheta}.json
    Example: 0_-0.03_-0.02_-0.5.json

    Returns:
        Tuple of (dx, dy, dtheta) step sizes, or None if parsing fails
    """
    stem = filename.replace('.json', '')
    parts = stem.split('_')

    if len(parts) < 4:
        return None

    try:
        # Skip the index, extract dx, dy, dtheta
        dx = float(parts[1])
        dy = float(parts[2])
        dtheta = float(parts[3])
        return (dx, dy, dtheta)
    except (ValueError, IndexError):
        return None


def steps_to_vel(step_size: float, period: float) -> float:
    """
    Convert step size to velocity.

    This matches the conversion in gait_generator.py.

    Args:
        step_size: Step size in meters (for dx, dy) or radians (for dtheta)
        period: Gait period in seconds

    Returns:
        Velocity in m/s (for x, y) or rad/s (for theta)
    """
    return (step_size * 2) / period


def compute_phase(num_frames: int, fps: float, period: float) -> np.ndarray:
    """
    Compute gait phase for each frame.

    Phase cycles from 0 to 1 over one gait period.

    Args:
        num_frames: Number of frames in the motion
        fps: Frames per second
        period: Gait period in seconds

    Returns:
        Array of phase values in [0, 1)
    """
    dt = 1.0 / fps
    times = np.arange(num_frames) * dt
    phase = (times / period) % 1.0
    return phase


def convert_json_to_motion_data(json_path: str) -> dict:
    """
    Convert a single JSON file to the motion data format.

    Args:
        json_path: Path to JSON file

    Returns:
        Dictionary with keys: motion_data, phase, period, fps, meta
    """
    with open(json_path, 'r') as f:
        data = json.load(f)

    # Extract motion data
    frames = np.array(data['Frames'], dtype=np.float32)
    fps = float(data['FPS'])
    period = float(data['Placo']['period'])

    # Compute phase
    num_frames = len(frames)
    phase = compute_phase(num_frames, fps, period)

    # Extract metadata
    meta = {
        'Frame_offset': data['Frame_offset'],
        'Frame_size': data['Frame_size']
    }

    return {
        'motion_data': frames,
        'phase': phase,
        'period': period,
        'fps': fps,
        'meta': meta
    }


def main():
    parser = argparse.ArgumentParser(
        description='Convert JSON motion files to a single pickle file indexed by velocity'
    )
    parser.add_argument(
        '--input_dir',
        type=str,
        default='out',
        help='Directory containing JSON motion files (default: out)'
    )
    parser.add_argument(
        '--output',
        type=str,
        default='reference_motions.pkl',
        help='Output pickle file path (default: reference_motions.pkl)'
    )
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Print verbose output'
    )

    args = parser.parse_args()

    # Find all JSON files
    input_path = Path(args.input_dir)
    if not input_path.exists():
        print(f"Error: Input directory '{args.input_dir}' does not exist")
        return 1

    json_files = sorted(input_path.glob('*.json'))
    if not json_files:
        print(f"Error: No JSON files found in '{args.input_dir}'")
        return 1

    print(f"Found {len(json_files)} JSON files in '{args.input_dir}'")

    # Process each file
    motion_dict = {}
    skipped = 0

    for json_file in json_files:
        # Parse step sizes from filename
        step_sizes = parse_step_sizes_from_filename(json_file.name)
        if step_sizes is None:
            if args.verbose:
                print(f"Skipping {json_file.name}: could not parse step sizes")
            skipped += 1
            continue

        dx, dy, dtheta = step_sizes

        # Convert JSON to motion data format first to get the period
        try:
            motion_data = convert_json_to_motion_data(str(json_file))

            # Convert step sizes to velocities using the period
            period = motion_data['period']
            vx = steps_to_vel(dx, period)
            vy = steps_to_vel(dy, period)
            vtheta = steps_to_vel(dtheta, period)

            # Round to 3 decimal places to match gait_generator.py behavior
            vx = round(vx, 3)
            vy = round(vy, 3)
            vtheta = round(vtheta, 3)

            # Create key in the format expected by PolyMotionLoader (velocities in m/s)
            key = f"{vx}_{vy}_{vtheta}"

            motion_dict[key] = motion_data

            if args.verbose:
                print(f"Processed {json_file.name} -> key '{key}' "
                      f"(step_sizes: {dx}, {dy}, {dtheta}) "
                      f"({motion_data['motion_data'].shape[0]} frames, "
                      f"period={motion_data['period']:.3f}s)")
        except Exception as e:
            print(f"Error processing {json_file.name}: {e}")
            skipped += 1
            continue

    if not motion_dict:
        print("Error: No valid motion files processed")
        return 1

    # Save to pickle
    output_path = Path(args.output)
    with open(output_path, 'wb') as f:
        pickle.dump(motion_dict, f)

    print(f"\nSuccessfully converted {len(motion_dict)} motions")
    print(f"Skipped {skipped} files")
    print(f"Output saved to: {output_path.absolute()}")

    # Print summary statistics
    num_frames_list = [v['motion_data'].shape[0] for v in motion_dict.values()]
    periods_list = [v['period'] for v in motion_dict.values()]

    print(f"\nSummary:")
    print(f"  Unique velocity commands: {len(motion_dict)}")
    print(f"  Frame count range: {min(num_frames_list)} - {max(num_frames_list)}")
    print(f"  Period range: {min(periods_list):.3f}s - {max(periods_list):.3f}s")
    print(f"  Frame size: {list(motion_dict.values())[0]['motion_data'].shape[1]}")

    return 0


if __name__ == '__main__':
    exit(main())
