"""Keep the Nav2 envelope consistent with the UGV01-X3 chassis model."""
import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_nav2_footprint_matches_chassis_collision_box():
    robot = ET.parse(ROOT / 'urdf/ugv01_box.urdf')
    size = [float(v) for v in robot.find("link[@name='base_link']/collision/geometry/box").get('size').split()]
    assert size[:2] == pytest.approx([0.232, 0.197])
    config = yaml.safe_load((ROOT / 'config/nav2.yaml').read_text())
    for name in ('local_costmap', 'global_costmap'):
        params = config[name][name]['ros__parameters']
        points = ast.literal_eval(params['footprint'])
        assert max(x for x, y in points) - min(x for x, y in points) == pytest.approx(size[0])
        assert max(y for x, y in points) - min(y for x, y in points) == pytest.approx(size[1])
        assert params['footprint_padding'] > 0


def test_track_visuals_do_not_make_a_242mm_wide_chassis():
    robot = ET.parse(ROOT / 'urdf/ugv01_box.urdf')
    y = float(robot.find("joint[@name='left_tread_blocks_joint']/origin").get('xyz').split()[1]) if robot.find("joint[@name='left_tread_blocks_joint']") is not None else None
    if y is None:
        joint = next(j for j in robot.findall('joint') if j.find('child').get('link') == 'left_tread_blocks')
        y = float(joint.find('origin').get('xyz').split()[1])
    track_width = float(robot.find("link[@name='left_tread_blocks']/visual/geometry/box").get('size').split()[1])
    assert 2 * y + track_width == pytest.approx(0.197, abs=0.003)


def test_zero_encoder_speed_cannot_trap_rotation_at_one_acceleration_step():
    config = yaml.safe_load((ROOT / 'config/nav2.yaml').read_text())
    controller = config['controller_server']['ros__parameters']
    rpp = controller['FollowPath']
    assert rpp['max_angular_accel'] / controller['controller_frequency'] >= rpp['rotate_to_heading_angular_vel']
    # The physical command ramp remains independently limited in OPEN_LOOP.
    smoother = config['velocity_smoother']['ros__parameters']
    assert smoother['feedback'] == 'OPEN_LOOP'
    assert smoother['max_accel'][2] == 0.8
