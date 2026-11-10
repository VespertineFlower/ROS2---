#!/usr/bin/env python3
"""Generate the rectangular-wall maze Gazebo world used by the project.

The maze is a perfect maze (spanning tree over the cells), therefore a
left-wall-following robot is guaranteed to find the exit.  Walls are emitted
as static box models so that Gazebo gets real collision geometry.

Usage:
    python3 tools/generate_maze.py \
        --output src/maze_description/worlds/maze.world

The script prints an ASCII map and the shortest cell route as a sanity
check so that the generated maze is known to be solvable.
"""

import argparse
import random
from collections import deque

CELL = 0.8          # corridor width / cell pitch [m]
WALL_T = 0.10       # wall thickness [m]
WALL_H = 0.5        # wall height [m]
COLS = 5
ROWS = 5
SEED = 7
GAP = 0.62          # entrance / exit opening width [m]

# directions: (dx, dy, wall-attr, opposite-attr, ascii-delta)
N, E, S, W = (0, 1), (1, 0), (0, -1), (-1, 0)


class Maze:
    def __init__(self, cols, rows, seed):
        self.cols = cols
        self.rows = rows
        # wall[c][r] is a set of directions that still have a wall
        self.wall = {(c, r): {N, E, S, W} for c in range(cols) for r in range(rows)}
        self._carve(seed)

    def _carve(self, seed):
        rng = random.Random(seed)
        start = (0, 0)
        visited = {start}
        stack = [start]
        while stack:
            c, r = stack[-1]
            options = []
            for d in (N, E, S, W):
                nc, nr = c + d[0], r + d[1]
                if 0 <= nc < self.cols and 0 <= nr < self.rows and (nc, nr) not in visited:
                    options.append((d, nc, nr))
            if not options:
                stack.pop()
                continue
            d, nc, nr = rng.choice(options)
            opp = (-d[0], -d[1])
            self.wall[(c, r)].discard(d)
            self.wall[(nc, nr)].discard(opp)
            visited.add((nc, nr))
            stack.append((nc, nr))

    def ascii(self):
        lines = []
        for r in range(self.rows - 1, -1, -1):
            top = ""
            mid = ""
            for c in range(self.cols):
                top += "+" + ("  " if N not in self.wall[(c, r)] else "--")
                mid += (" " if W not in self.wall[(c, r)] else "|") + "  "
            top += "+"
            mid += "|"
            lines.append(top)
            lines.append(mid)
        bottom = ""
        for c in range(self.cols):
            bottom += "+" + ("  " if S not in self.wall[(c, r)] else "--")
        bottom += "+"
        lines.append(bottom)
        return "\n".join(lines)

    def bfs_route(self):
        """Shortest cell route from the entrance to the exit (sanity check)."""
        start, goal = (0, 0), (self.cols - 1, self.rows - 1)
        prev = {start: None}
        q = deque([start])
        while q:
            cur = q.popleft()
            if cur == goal:
                break
            for d in (N, E, S, W):
                if d in self.wall[cur]:
                    continue
                nxt = (cur[0] + d[0], cur[1] + d[1])
                if 0 <= nxt[0] < self.cols and 0 <= nxt[1] < self.rows and nxt not in prev:
                    prev[nxt] = cur
                    q.append(nxt)
        route = []
        cur = goal
        while cur is not None:
            route.append(cur)
            cur = prev[cur]
        return list(reversed(route))


def wall_segments(maze):
    """Return a list of (cx, cy, sx, sy, name) wall boxes in world coords.

    Horizontal walls are split around the entrance (bottom of cell 0,0) and
    the exit (top of cell cols-1,rows-1).
    """
    segs = []
    for c in range(maze.cols):
        for r in range(maze.rows):
            x0 = c * CELL
            y0 = r * CELL
            if S in maze.wall[(c, r)]:
                # horizontal wall on the bottom edge of (c, r)
                y = y0
                if r == 0 and c == 0:
                    # entrance opening
                    left = CELL / 2 - GAP / 2
                    right = CELL / 2 + GAP / 2
                    segs.append((x0 + left / 2, y, left, WALL_T, "entrance_w"))
                    segs.append((x0 + right + (CELL - right) / 2, y,
                                 CELL - right, WALL_T, "entrance_e"))
                else:
                    segs.append((x0 + CELL / 2, y, CELL, WALL_T, ""))
            if W in maze.wall[(c, r)]:
                x = x0
                segs.append((x, y0 + CELL / 2, WALL_T, CELL, ""))
    # north boundary (only the top row cells, with the exit opening)
    for c in range(maze.cols):
        y = maze.rows * CELL
        if c == maze.cols - 1:
            left = CELL / 2 - GAP / 2
            right = CELL / 2 + GAP / 2
            x0 = c * CELL
            segs.append((x0 + left / 2, y, left, WALL_T, "exit_w"))
            segs.append((x0 + right + (CELL - right) / 2, y,
                         CELL - right, WALL_T, "exit_e"))
        else:
            segs.append((c * CELL + CELL / 2, y, CELL, WALL_T, ""))
    # east boundary (only the right column)
    for r in range(maze.rows):
        segs.append((maze.cols * CELL, r * CELL + CELL / 2, WALL_T, CELL, ""))
    return segs


def write_world(path, maze, segs):
    tpl_head = """<?xml version="1.0" ?>
<!--
  maze.world - rectangular wall maze for the TurtleBot3 Burger (ROS 2 Humble)
  Generated by tools/generate_maze.py, {cols}x{rows} cells of {cell} m,
  wall thickness {t} m, entrance at the south-west corner and exit at the
  north-east corner.
-->
<sdf version="1.6">
  <world name="maze_world">
    <!-- Sun light and ground plane are declared inline on purpose: referencing
         model://... makes Gazebo query the online model database
         (models.gazebosim.org) while loading the world, which hangs when the
         machine is offline or the mirror is unreachable. -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>20 20</size></plane>
          </geometry>
          <surface>
            <friction><ode><mu>100</mu><mu2>50</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane><normal>0 0 1</normal><size>20 20</size></plane>
          </geometry>
          <material>
            <ambient>0.65 0.65 0.65 1</ambient>
            <diffuse>0.75 0.75 0.75 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <real_time_factor>1.0</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
      </ode>
    </physics>

    <scene>
      <ambient>0.65 0.65 0.65 1</ambient>
      <background>0.90 0.92 0.96 1</background>
      <shadows>true</shadows>
    </scene>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>{cam_x:.2f} {cam_y:.2f} 6.0 0 1.5707 0</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

    <!-- start pad (south-west corner, visual only) -->
    <model name="start_pad">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <pose>0 0 0.002 0 0 0</pose>
          <geometry><cylinder><radius>0.16</radius><length>0.004</length></cylinder></geometry>
          <material><ambient>0.0 0.85 0.2 1</ambient><diffuse>0.0 0.85 0.2 1</diffuse></material>
        </visual>
      </link>
      <pose>{start_x:.3f} {start_y:.3f} 0 0 0 0</pose>
    </model>

    <!-- exit pad (north-east corner, visual only) -->
    <model name="exit_pad">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <pose>0 0 0.002 0 0 0</pose>
          <geometry><cylinder><radius>0.16</radius><length>0.004</length></cylinder></geometry>
          <material><ambient>0.9 0.1 0.1 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material>
        </visual>
      </link>
      <pose>{exit_x:.3f} {exit_y:.3f} 0 0 0 0</pose>
    </model>
"""
    tpl_tail = """  </world>
</sdf>
"""
    body = []
    for i, (cx, cy, sx, sy, tag) in enumerate(segs):
        name = "maze_wall_%03d" % i
        if tag:
            name += "_" + tag
        body.append(f"""
    <model name="{name}">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.3f} {sy:.3f} {WALL_H}</size></box></geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.3f} {sy:.3f} {WALL_H}</size></box></geometry>
          <material>
            <ambient>0.32 0.36 0.46 1</ambient>
            <diffuse>0.42 0.47 0.58 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
      <pose>{cx:.3f} {cy:.3f} {WALL_H/2:.3f} 0 0 0</pose>
    </model>""")

    start = (CELL / 2, CELL / 2)
    exit_ = (maze.cols * CELL - CELL / 2, maze.rows * CELL + 0.25)
    content = tpl_head.format(cols=maze.cols, rows=maze.rows, cell=CELL,
                              t=WALL_T,
                              cam_x=maze.cols * CELL / 2,
                              cam_y=maze.rows * CELL / 2,
                              start_x=start[0], start_y=start[1],
                              exit_x=exit_[0], exit_y=exit_[1])
    content += "\n".join(body) + "\n" + tpl_tail
    with open(path, "w") as f:
        f.write(content)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", required=True)
    ap.add_argument("--seed", type=int, default=SEED)
    args = ap.parse_args()

    maze = Maze(COLS, ROWS, args.seed)
    print(maze.ascii())
    route = maze.bfs_route()
    print("shortest route length:", len(route), "cells")
    print("shortest route:", route)
    segs = wall_segments(maze)
    print("wall boxes:", len(segs))
    write_world(args.output, maze, segs)
    print("world written to", args.output)


if __name__ == "__main__":
    main()
