"""Tests for the world-file parser and the automatic pure-pursuit route."""

import math

import pytest

from maze_control.world_maze import (auto_goal, free_run, parse_world,
                                     route_from_world, route_length,
                                     segment_clearance, world_bounds)

WALL = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
{models}
  </world>
</sdf>
"""

MODEL = """    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0.25 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} 0.5</size></box></geometry>
        </collision>
      </link>
    </model>"""


def write_world(tmp_path, boxes):
    models = '\n'.join(MODEL.format(name=f'maze_wall_{i:03d}', x=x, y=y,
                                    sx=sx, sy=sy)
                       for i, (x, y, sx, sy) in enumerate(boxes))
    path = tmp_path / 'maze.world'
    path.write_text(WALL.format(models=models))
    return str(path)


def corridor(tmp_path):
    """5 m x 3 m room split by a half wall, with a 1.1 m gap at the top.

    y = 0 and y = 3.0 are the long walls; at x = 2.5 a wall runs from the
    floor up to y = 1.9, so the only way through is above it.
    """
    boxes = [
        (2.5, 0.0, 5.1, 0.1),      # south wall
        (2.5, 3.0, 5.1, 0.1),      # north wall
        (2.5, 0.95, 0.1, 1.9),     # half wall, y = 0 .. 1.9
    ]
    return write_world(tmp_path, boxes)


def test_parse_world_reads_boxes(tmp_path):
    path = corridor(tmp_path)
    segments = parse_world(path)
    assert len(segments) == 3
    assert segments[0][4] == 'maze_wall_000'
    assert segments[2][:4] == (2.5, 0.95, 0.1, 1.9)


def test_parse_world_rejects_a_world_without_maze_walls(tmp_path):
    path = tmp_path / 'empty.world'
    path.write_text(WALL.format(models='    <!-- nothing here -->'))
    with pytest.raises(RuntimeError, match='no <model name="maze_wall_'):
        parse_world(str(path))


def test_world_bounds_follows_the_walls(tmp_path):
    path = corridor(tmp_path)
    xmin, xmax, ymin, ymax = world_bounds(parse_world(path), margin=0.3)
    assert xmin == pytest.approx(-0.35, abs=0.01)      # -0.05 - 0.30
    assert xmax == pytest.approx(5.35, abs=0.01)       # 5.05 + 0.30
    assert ymin == pytest.approx(-0.35, abs=0.01)
    assert ymax == pytest.approx(3.35, abs=0.01)


def test_route_goes_around_the_half_wall(tmp_path):
    path = corridor(tmp_path)
    route, walls = route_from_world(path, (0.4, 1.0), (4.6, 1.0))
    assert walls == 3
    assert len(route) >= 3
    assert route[0] == (0.4, 1.0) and route[-1] == (4.6, 1.0)
    # the detour has to climb over the half wall (top is at y = 1.9)
    assert max(y for _, y in route) > 1.9
    for x, y in route:
        assert 0.0 <= x <= 5.1
        assert 0.0 <= y <= 3.0


def test_route_is_longer_than_the_straight_line(tmp_path):
    path = corridor(tmp_path)
    route, _ = route_from_world(path, (0.4, 1.0), (4.6, 1.0))
    assert route_length(route) > math.dist((0.4, 1.0), (4.6, 1.0))


def test_route_reports_when_the_goal_is_walled_off(tmp_path):
    boxes = [(2.5, 0.0, 5.1, 0.1), (2.5, 3.0, 5.1, 0.1),
             (2.5, 1.5, 0.1, 3.0)]          # full height wall, no gap
    path = write_world(tmp_path, boxes)
    with pytest.raises(RuntimeError, match='no route'):
        route_from_world(path, (0.4, 1.0), (4.6, 1.0))



def test_free_run_stops_before_the_wall(tmp_path):
    path = corridor(tmp_path)
    segments = parse_world(path)
    # from x = 0.4 the half wall stands at x = 2.45, so the run is ~1.9 m
    run = free_run(segments, (0.4, 1.0), (1.0, 0.0), clearance=0.16)
    assert run == pytest.approx(1.89, abs=0.05)


def test_free_run_honours_the_bounds(tmp_path):
    path = corridor(tmp_path)
    segments = parse_world(path)
    run = free_run(segments, (0.4, 1.0), (-1.0, 0.0), clearance=0.16,
                   bounds=(0.05, 5.05, 0.05, 2.95))
    assert run < 0.4          # the bounds stop it almost at once


def test_auto_goal_picks_the_longest_free_direction(tmp_path):
    path = corridor(tmp_path)
    goal, where, length = auto_goal(parse_world(path), (0.4, 1.0))
    assert where == 'east'                  # 1.9 m free, vs 1.8 north / 0.3 west
    assert length == pytest.approx(1.89, abs=0.05)
    assert goal[1] == pytest.approx(1.0)    # stays on the corridor axis
    assert goal[0] == pytest.approx(2.16, abs=0.06)


def test_segment_clearance_reports_the_blocking_point(tmp_path):
    path = corridor(tmp_path)
    segments = parse_world(path)
    # down the corridor is clear, but the half wall stops an eastward dash
    clear, _at = segment_clearance(segments, (0.4, 1.0), (2.0, 1.0))
    assert clear > 0.3
    clear, at = segment_clearance(segments, (0.4, 1.0), (4.6, 1.0))
    assert clear == 0.0
    assert at is not None and at[0] > 2.3
