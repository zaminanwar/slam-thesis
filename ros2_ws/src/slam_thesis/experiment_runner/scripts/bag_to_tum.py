#!/usr/bin/env python3
"""Extract trajectory from bag to TUM format.

Supports:
- Odometry topics (e.g., /gt_pose, /odom)
- TF transforms (e.g., map->base_footprint)
- TF chain composition (e.g., map->odom->base_footprint)
"""

import argparse
import os
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


def quat_multiply(q1, q2):
    """Multiply two quaternions (x,y,z,w format)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])


def quat_rotate(q, v):
    """Rotate vector v by quaternion q."""
    x, y, z, w = q
    # Convert to rotation matrix and apply
    r00 = 1 - 2*(y*y + z*z)
    r01 = 2*(x*y - z*w)
    r02 = 2*(x*z + y*w)
    r10 = 2*(x*y + z*w)
    r11 = 1 - 2*(x*x + z*z)
    r12 = 2*(y*z - x*w)
    r20 = 2*(x*z - y*w)
    r21 = 2*(y*z + x*w)
    r22 = 1 - 2*(x*x + y*y)
    return np.array([
        r00*v[0] + r01*v[1] + r02*v[2],
        r10*v[0] + r11*v[1] + r12*v[2],
        r20*v[0] + r21*v[1] + r22*v[2]
    ])


def compose_transforms(t1, t2):
    """Compose two transforms: result = t1 * t2.
    Each transform is (tx, ty, tz, qx, qy, qz, qw)."""
    pos1 = np.array(t1[:3])
    quat1 = np.array(t1[3:])
    pos2 = np.array(t2[:3])
    quat2 = np.array(t2[3:])

    # Composed position: p1 + R1 * p2
    pos = pos1 + quat_rotate(quat1, pos2)
    # Composed rotation: q1 * q2
    quat = quat_multiply(quat1, quat2)

    return (*pos, *quat)


def extract_from_odom(bag_path, topic, output):
    """Extract trajectory from Odometry topic."""
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id=''),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    )

    os.makedirs(os.path.dirname(output) or '.', exist_ok=True)

    count = 0
    with open(output, 'w') as f:
        f.write('# timestamp tx ty tz qx qy qz qw\n')
        while reader.has_next():
            t, data, _ = reader.read_next()
            if t == topic:
                msg = deserialize_message(data, Odometry)
                ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                p = msg.pose.pose.position
                o = msg.pose.pose.orientation
                f.write(f'{ts:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {o.x:.6f} {o.y:.6f} {o.z:.6f} {o.w:.6f}\n')
                count += 1
    return count


def extract_from_tf(bag_path, parent_frame, child_frame, output):
    """Extract trajectory from TF messages (direct transform only)."""
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id=''),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    )

    os.makedirs(os.path.dirname(output) or '.', exist_ok=True)

    count = 0
    last_ts = None
    with open(output, 'w') as f:
        f.write('# timestamp tx ty tz qx qy qz qw\n')
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic in ['/tf', '/tf_static']:
                msg = deserialize_message(data, TFMessage)
                for transform in msg.transforms:
                    if transform.header.frame_id == parent_frame and transform.child_frame_id == child_frame:
                        ts = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                        if last_ts is not None and ts <= last_ts:
                            continue
                        last_ts = ts
                        t = transform.transform.translation
                        r = transform.transform.rotation
                        f.write(f'{ts:.9f} {t.x:.6f} {t.y:.6f} {t.z:.6f} {r.x:.6f} {r.y:.6f} {r.z:.6f} {r.w:.6f}\n')
                        count += 1
    return count


def extract_from_tf_chain(bag_path, frame_chain, output):
    """Extract trajectory by composing a chain of TF transforms.

    frame_chain: list of frames, e.g., ['map', 'odom', 'base_footprint']
    Composes: map->odom * odom->base_footprint = map->base_footprint
    """
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id=''),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    )

    os.makedirs(os.path.dirname(output) or '.', exist_ok=True)

    # Build list of (parent, child) pairs we need
    pairs = [(frame_chain[i], frame_chain[i+1]) for i in range(len(frame_chain)-1)]

    # Store latest transform for each pair
    latest_tf = {pair: None for pair in pairs}
    latest_ts = {pair: 0.0 for pair in pairs}

    count = 0
    last_output_ts = None

    with open(output, 'w') as f:
        f.write('# timestamp tx ty tz qx qy qz qw\n')

        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic in ['/tf', '/tf_static']:
                msg = deserialize_message(data, TFMessage)
                for transform in msg.transforms:
                    pair = (transform.header.frame_id, transform.child_frame_id)
                    if pair in latest_tf:
                        ts = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                        t = transform.transform.translation
                        r = transform.transform.rotation
                        latest_tf[pair] = (t.x, t.y, t.z, r.x, r.y, r.z, r.w)
                        latest_ts[pair] = ts

                        # Check if all transforms available
                        if all(v is not None for v in latest_tf.values()):
                            # Use max timestamp as the composed timestamp
                            composed_ts = max(latest_ts.values())

                            # Skip if same timestamp
                            if last_output_ts is not None and composed_ts <= last_output_ts:
                                continue

                            # Compose chain
                            result = latest_tf[pairs[0]]
                            for i in range(1, len(pairs)):
                                result = compose_transforms(result, latest_tf[pairs[i]])

                            last_output_ts = composed_ts
                            f.write(f'{composed_ts:.9f} {result[0]:.6f} {result[1]:.6f} {result[2]:.6f} '
                                   f'{result[3]:.6f} {result[4]:.6f} {result[5]:.6f} {result[6]:.6f}\n')
                            count += 1
    return count


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True, help='Path to bag')
    parser.add_argument('--output', required=True, help='Output TUM file')
    parser.add_argument('--topic', default=None, help='Odometry topic (e.g., /gt_pose)')
    parser.add_argument('--parent_frame', default=None, help='TF parent frame (e.g., map)')
    parser.add_argument('--child_frame', default=None, help='TF child frame (e.g., base_footprint)')
    parser.add_argument('--chain', default=None, help='TF chain (e.g., map,odom,base_footprint)')
    args = parser.parse_args()

    if args.topic:
        count = extract_from_odom(args.bag, args.topic, args.output)
    elif args.chain:
        frame_chain = [f.strip() for f in args.chain.split(',')]
        if len(frame_chain) < 2:
            print('Error: Chain must have at least 2 frames')
            return 1
        print(f'Composing TF chain: {" -> ".join(frame_chain)}')
        count = extract_from_tf_chain(args.bag, frame_chain, args.output)
    elif args.parent_frame and args.child_frame:
        count = extract_from_tf(args.bag, args.parent_frame, args.child_frame, args.output)
    else:
        print('Error: Specify --topic, --chain, or both --parent_frame and --child_frame')
        return 1

    print(f'Wrote {count} poses to {args.output}')
    return 0


if __name__ == '__main__':
    exit(main())
