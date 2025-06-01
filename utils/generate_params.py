#!/usr/bin/env python3
import json
import argparse
import math
from pathlib import Path

TEMPLATE = """// Auto-generated from {json_path}
#pragma once

inline void loadHardcodedParams(RobotDynamics::RobotParams& params) {{
    // Geometry parameters
    params.a1x = {a1x};
    params.a1y = {a1y};
    params.a2x = {a2x};
    params.a2y = {a2y};
    params.a3x = {a3x};
    params.a3y = {a3y};
    params.d = {d};
    params.v_1 = {v_1};
    params.v_2 = {v_2};
    params.v_3 = {v_3};
    params.h_1 = {h_1};
    params.h_2 = {h_2};
    params.h_3 = {h_3};
    params.l_11 = {l_11};
    params.l_12 = {l_12};
    params.l_21 = {l_21};
    params.l_22 = {l_22};
    params.l_31 = {l_31};
    params.l_32 = {l_32};

    // Mass parameters
    params.m_11 = {m_11};
    params.m_12 = {m_12};
    params.m_21 = {m_21};
    params.m_22 = {m_22};
    params.m_31 = {m_31};
    params.m_32 = {m_32};
    params.m_d = {m_d};
    params.m_arm = {m_arm};

    // Position vectors
    params.vec_elbow = Eigen::Vector3d({elbow_x}, {elbow_y}, {elbow_z});
    params.vec_shoulder = Eigen::Vector3d({shoulder_x}, {shoulder_y}, {shoulder_z});
    params.initial_pos = Eigen::Vector3d({init_x}, {init_y}, {init_z});

    // Computed parameters
    params.l_arm_proth = {l_arm_proth};
}}"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("json_path", help="Path to JSON config file")
    parser.add_argument("output_dir", help="Output directory for header file")
    args = parser.parse_args()

    with open(args.json_path) as f:
        config = json.load(f)

    # Extract parameters
    geom = config["geometry"]
    elbow = config["elbow"]
    init_pos = config["initial_pos"]
    shoulder = config["shoulder"]
    arm = config["arm"]
    mass = config["mass"]

    # Compute derived values
    p0 = [init_pos["x"], init_pos["y"], init_pos["z"]]
    elbow_vec = [elbow["x"], elbow["y"], elbow["z"]]

    # Calculate l_arm_proth as Euclidean distance
    l_arm_proth = math.sqrt(
        (p0[0] - elbow_vec[0]) ** 2 +
        (p0[1] - elbow_vec[1]) ** 2 +
        (p0[2] - elbow_vec[2]) ** 2
    )

    # Compute shoulder position using the same formula as misc.py
    l_humerus = arm["l_humerus"]
    z_shoulder = shoulder["z"]

    # Compute XY offset - same as misc.py
    try:
        offset_val = (l_humerus ** 2 - z_shoulder ** 2) / 2.0
        if offset_val < 0:
            raise ValueError("Negative value in square root")
        xy_offset = math.sqrt(offset_val)
    except ValueError as e:
        print(f"Warning: {e}, using 0 for xy_offset")
        xy_offset = 0.0

    shoulder_x = elbow["x"] + xy_offset
    shoulder_y = elbow["y"] + xy_offset
    shoulder_z = z_shoulder

    # Generate C++ code
    output = TEMPLATE.format(
        json_path=args.json_path,
        a1x=config.get("a1x", 0.0),
        a1y=config.get("a1y", 0.0),
        a2x=config.get("a2x", 0.0),
        a2y=config.get("a2y", 0.0),
        a3x=config.get("a3x", 0.0),
        a3y=config.get("a3y", 0.0),
        d=geom["d"],
        v_1=geom["v"][0],
        v_2=geom["v"][1],
        v_3=geom["v"][2],
        h_1=geom["h"][0],
        h_2=geom["h"][1],
        h_3=geom["h"][2],
        l_11=geom["l1"][0],
        l_12=geom["l1"][1],
        l_21=geom["l2"][0],
        l_22=geom["l2"][1],
        l_31=geom["l3"][0],
        l_32=geom["l3"][1],
        m_11=mass["m_1"][0],
        m_12=mass["m_1"][1],
        m_21=mass["m_2"][0],
        m_22=mass["m_2"][1],
        m_31=mass["m_3"][0],
        m_32=mass["m_3"][1],
        m_d=mass["m_d_seul"],
        m_arm=mass["m_bras"],
        elbow_x=elbow["x"],
        elbow_y=elbow["y"],
        elbow_z=elbow["z"],
        shoulder_x=shoulder_x,
        shoulder_y=shoulder_y,
        shoulder_z=shoulder_z,
        init_x=init_pos["x"],
        init_y=init_pos["y"],
        init_z=init_pos["z"],
        l_arm_proth=l_arm_proth
    )

    # Write output file
    output_path = Path(args.output_dir) / "HardcodedParams.hpp"
    with open(output_path, "w") as f:
        f.write(output)

    print(f"Generated {output_path}")


if __name__ == "__main__":
    main()