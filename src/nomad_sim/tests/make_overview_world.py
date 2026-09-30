#!/usr/bin/env python3
"""Make a temporary diagnostic world with an r1-mounted overhead camera.

The production map and provisional OAK/LiDAR settings are not modified.
The camera output is /forest_overview/image_raw, not a GUI screenshot.
"""
import argparse
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    world = args.world.resolve()
    package = world.parent.parent
    metadata = json.loads((package/'assets/forest/layout.json').read_text())
    start = metadata['waypoints']['start']
    path = metadata['paths']['entrance']
    yaw = math.atan2(path[1][1]-path[0][1], path[1][0]-path[0][0])
    # Transform the world center into the initial vehicle frame.
    dx, dy = -start[0], -start[1]
    local_x = math.cos(yaw)*dx+math.sin(yaw)*dy
    local_y = -math.sin(yaw)*dx+math.cos(yaw)*dy
    fragment = f'''
    <sensor class="camera" name="forest_overview">
      <pose_3d>{local_x:.5f} {local_y:.5f} 70 {-math.degrees(yaw):.5f} 0 180</pose_3d>
      <sensor_period>1.0</sensor_period>
      <ncols>1120</ncols><nrows>800</nrows>
      <cx>560</cx><cy>400</cy><fx>980</fx><fy>980</fy>
      <clip_min>0.01</clip_min><clip_max>150</clip_max>
      <preview_win_visible>false</preview_win_visible>
    </sensor>
'''
    text = world.read_text(encoding='utf-8')
    text = text.replace('${MVSIM_CURRENT_FILE_DIRECTORY}', str(world.parent))
    text = text.replace('>../assets/', f'>{package}/assets/')
    text = text.replace('file="../sensors/', f'file="{package}/sensors/')
    text = text.replace('</vehicle>', fragment+'</vehicle>', 1)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text, encoding='utf-8')
    print(args.output)


if __name__ == '__main__':
    main()
