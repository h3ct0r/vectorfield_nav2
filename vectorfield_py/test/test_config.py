# Copyright 2026 Héctor Azpúrua
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""CoreConfig attribute objects, enums and validate_config()."""

import copy

import pytest
import vectorfield as vf


def test_defaults_validate_without_errors_or_warnings():
    """The default CoreConfig passes, with and without a costmap, in both obstacle modes."""
    config = vf.CoreConfig()
    for mode in (vf.ObstacleMode.NONE, vf.ObstacleMode.CONTOUR):
        config.obstacles.obstacle_mode = mode
        for costmap in (None, vf.CostmapInfo(width_m=3.0, height_m=3.0, resolution=0.05)):
            report = vf.validate_config(config, costmap)
            assert report.ok, report.summary()
            assert report.errors == [] and report.warnings == []
    assert vf.validate_config(vf.CoreConfig(), costmap_info=None).ok


def test_invalid_config_reports_the_error_code():
    """desired_speed above max_linear_vel is rejected with its stable code."""
    config = vf.CoreConfig()
    config.field.desired_speed = 1.0
    report = vf.validate_config(config)
    assert not report.ok
    assert report.has_error(vf.issue.SPEED_ABOVE_MAX)
    assert vf.issue.SPEED_ABOVE_MAX in [e.code for e in report.errors]
    assert 'desired_speed' in report.errors[0].message
    with pytest.raises(vf.ConfigError, match='desired_speed'):
        report.raise_on_error()
    with pytest.raises(ValueError):
        vf.Tracker(config)


def test_domain_errors():
    """A non-positive frequency and a broken footprint are domain errors."""
    config = vf.CoreConfig()
    config.node.controller_frequency = -1.0
    assert vf.validate_config(config).has_error(vf.issue.INVALID_VALUE)
    config = vf.CoreConfig()
    config.node.footprint.polygon = [[0.0, 0.0], [1.0, 0.0]]
    assert vf.validate_config(config).has_error(vf.issue.INVALID_FOOTPRINT)
    assert vf.footprint_is_valid(config.node.footprint)[0] is False


def test_warnings_and_auto_values():
    """A short horizon warns; resolve_auto_values fills the contour distances."""
    config = vf.CoreConfig()
    config.safety.collision_check_horizon = 0.2
    report = vf.validate_config(config)
    assert report.ok and report.has_warning(vf.issue.HORIZON_TOO_SHORT)
    resolved = vf.resolve_auto_values(vf.CoreConfig())
    assert resolved.motion.tracking_law == vf.TrackingLaw.FEEDBACK_LINEARIZATION
    lam = resolved.obstacles.obstacle_contour_distance
    assert lam > 0.0
    assert resolved.obstacles.obstacle_full_switch_distance == pytest.approx(lam + 0.15)
    radii = vf.compute_footprint_radii(vf.CoreConfig().node.footprint)
    assert radii.r_circ == pytest.approx(0.22)
    assert vf.final_approach_distance(vf.CoreConfig(), 0.25) == pytest.approx(0.45)


def test_ackermann_contour():
    """Ackermann contour is valid with the turning-radius auto values; a small lambda is not."""
    config = vf.CoreConfig()
    config.motion.motion_model = vf.MotionModel.ACKERMANN
    config.motion.min_turning_radius = 0.5
    config.obstacles.obstacle_mode = vf.ObstacleMode.CONTOUR
    assert vf.validate_config(config).ok
    resolved = vf.resolve_auto_values(config)
    radii = vf.compute_footprint_radii(config.node.footprint)
    lam = vf.ackermann_auto_contour_distance(radii, 0.0, 0.5)
    assert lam == pytest.approx(0.77)
    assert resolved.obstacles.obstacle_contour_distance == pytest.approx(lam)
    assert resolved.obstacles.obstacle_blend_start_distance == pytest.approx(lam + 0.7)
    config.obstacles.obstacle_contour_distance = 0.5
    report = vf.validate_config(config)
    assert report.has_error(vf.issue.ACKERMANN_CONTOUR_RADIUS), report.summary()
    assert report.has_warning(vf.issue.ACKERMANN_CONTOUR_CURVATURE), report.summary()


def test_attribute_objects_reference_copy_and_keywords():
    """Nested groups are live references; copy() is deep; keyword constructors set fields."""
    config = vf.CoreConfig()
    config.motion.max_linear_vel = 0.8
    assert config.motion.max_linear_vel == 0.8
    other = config.copy()
    other.motion.max_linear_vel = 0.3
    assert config.motion.max_linear_vel == 0.8
    deep = copy.deepcopy(config)
    deep.field.direction = -1
    assert config.field.direction == 1
    built = vf.CoreConfig(field=vf.FieldConfig(desired_speed=0.25, gain_function='rational'))
    assert built.field.desired_speed == 0.25
    assert built.field.gain_function == vf.GainType.RATIONAL
    assert 'desired_speed=0.25' in repr(built.field)
    assert 'desired_speed' in vf.FieldConfig._fields
    with pytest.raises(TypeError):
        vf.FieldConfig(bogus=1.0)


def test_parameter_enums_convert_from_strings():
    """Parameter enums parse and print their parameter strings."""
    assert vf.MotionModel('skid_steer') == vf.MotionModel.SKID_STEER
    assert vf.MotionModel.from_string('ackermann') == vf.MotionModel.ACKERMANN
    assert vf.OmniHeadingPolicy.FIELD.to_string() == 'field'
    config = vf.MotionConfig()
    config.tracking_law = 'heading_tracking'
    assert config.tracking_law == vf.TrackingLaw.HEADING_TRACKING
    with pytest.raises(ValueError):
        vf.GainType('bogus')
    assert vf.is_valid_combination(vf.MotionModel.OMNIDIRECTIONAL, vf.TrackingLaw.HOLONOMIC)
    assert not vf.is_valid_combination('ackermann', 'holonomic')
    assert vf.resolve_tracking_law('ackermann', 'auto') == vf.TrackingLaw.HEADING_TRACKING


def test_curve_options_validation():
    """CurveOptions out of their domain are reported and rejected by make_curve()."""
    options = vf.CurveOptions(search_window=-1.0)
    assert vf.validate_curve_options(options).has_error(vf.issue.INVALID_CURVE_OPTIONS)
    with pytest.raises(vf.CurveError):
        vf.make_curve(vf.CurveSpec.builtin('ellipse'), options)


def test_tracker_exposes_its_resolved_configuration():
    """The Tracker keeps a resolved copy of the configuration and its kinematics."""
    config = vf.CoreConfig()
    config.motion.motion_model = vf.MotionModel.OMNIDIRECTIONAL
    tracker = vf.Tracker(config)
    assert tracker.config.motion.tracking_law == vf.TrackingLaw.HOLONOMIC
    assert tracker.kinematics.is_holonomic
    assert tracker.law.law == vf.TrackingLaw.HOLONOMIC
    assert tracker.footprint.r_circ == pytest.approx(0.22)
    assert tracker.validation.ok


def test_cbf_parameters_and_issues():
    """obstacle_mode cbf: defaults, domain errors, Ackermann rejection, the gain warnings."""
    config = vf.CoreConfig()
    obstacles = config.obstacles
    assert (obstacles.cbf_gain, obstacles.cbf_margin, obstacles.cbf_sites) == (2.0, 0.05, 3)
    obstacles.obstacle_mode = vf.ObstacleMode.CBF
    assert vf.validate_config(config).ok
    obstacles.cbf_sites = 4
    assert vf.validate_config(config).has_error(vf.issue.INVALID_VALUE)
    obstacles.cbf_sites = 2
    obstacles.cbf_gain = 10.0
    assert vf.validate_config(config).has_warning(vf.issue.CBF_GAIN_DECELERATION)
    config.motion.motion_model = vf.MotionModel.ACKERMANN
    config.motion.min_turning_radius = 0.5
    obstacles.cbf_gain = 2.0
    assert vf.validate_config(config).has_error(vf.issue.ACKERMANN_CBF_UNSUPPORTED)


def test_max_reverse_vel():
    """max_reverse_vel: auto = max_linear_vel; (0, v_max]; the reverse field speed and box."""
    config = vf.CoreConfig()
    assert config.motion.max_reverse_vel < 0.0
    assert vf.reverse_speed_limit(config) == config.motion.max_linear_vel
    assert vf.resolve_auto_values(config).motion.max_reverse_vel == config.motion.max_linear_vel
    config.motion.max_reverse_vel = 0.2
    assert vf.validate_config(config).ok
    assert vf.reference_speed(config, -1) == 0.2
    assert vf.reference_speed(config, 1) == config.field.desired_speed
    kinematics = vf.Tracker(config).kinematics
    assert kinematics.max_reverse_vel == 0.2
    assert kinematics.linear_limit() == config.motion.max_linear_vel
    kinematics.drive_backward = True
    assert kinematics.linear_limit() == 0.2
    config.motion.max_reverse_vel = 2.0 * config.motion.max_linear_vel
    assert vf.validate_config(config).has_error(vf.issue.REVERSE_ABOVE_MAX)
    config.motion.max_reverse_vel = 0.0
    assert vf.validate_config(config).has_error(vf.issue.INVALID_VALUE)
    config.motion.max_reverse_vel = 0.01
    config.motion.motion_model = vf.MotionModel.ACKERMANN
    config.motion.min_turning_radius = 0.5
    assert vf.validate_config(config).has_error(vf.issue.ACKERMANN_REVERSE_BELOW_BAND)
