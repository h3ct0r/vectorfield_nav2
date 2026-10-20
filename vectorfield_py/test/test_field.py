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

"""Gains, the distance field, laws, the limiter and streamlines."""

import math

import numpy as np
import pytest
import vectorfield as vf


@pytest.mark.parametrize('gain_type', [vf.GainType.ATAN, vf.GainType.RATIONAL])
def test_gain_identity_on_arrays(gain_type):
    """G^2 + H^2 = 1 elementwise; scalars return floats and arrays keep their shape."""
    gain = vf.GainFunction(gain_type, 3.0)
    D = np.linspace(0.0, 5.0, 101).reshape(101, 1)
    G, H = gain.G(D), gain.H(D)
    assert G.shape == D.shape and isinstance(gain.G(0.5), float)
    np.testing.assert_allclose(G**2 + H**2, 1.0, atol=1e-15)
    np.testing.assert_allclose(gain.Ginv(G), D, rtol=1e-9, atol=1e-12)
    assert gain.dG0() == pytest.approx(gain.dG(0.0))
    with pytest.raises(vf.ConfigError):
        vf.GainFunction(gain_type, -1.0)


@pytest.mark.parametrize(
    'spec',
    [
        vf.CurveSpec.builtin('ellipse', a=2.0, b=1.0),
        vf.CurveSpec.builtin('superellipse'),
        vf.CurveSpec.polyline([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [-1.0, 3.0]]),
    ],
)
@pytest.mark.parametrize('gain', [vf.GainFunction(), vf.GainFunction('rational', 5.0)])
def test_static_field_norm_equals_desired_speed(spec, gain):
    """A static field has |Phi| = v_r everywhere (|Phi_S| = 1, math_spec section 4.1)."""
    curve = vf.make_curve(spec)
    v_r = 0.37
    field = vf.DistanceField(gain, v_r)
    pts = np.random.default_rng(3).uniform(-4.0, 4.0, size=(500, 2))
    phi = field.field_vectors(curve, pts)
    assert phi.shape == (500, 2) and phi.dtype == np.float64
    np.testing.assert_allclose(np.linalg.norm(phi, axis=1), v_r, rtol=1e-12)
    sample = field.sample(curve, pts[0])
    np.testing.assert_allclose(sample.Phi, phi[0], atol=1e-15)
    assert sample.G == pytest.approx(gain.G(sample.D))
    np.testing.assert_allclose(sample.conv + sample.circ + sample.Phi_T, sample.Phi_gamma)


def test_field_converges_toward_the_curve():
    """Far away the field points toward the curve: Phi . n < 0."""
    curve = vf.make_curve(vf.CurveSpec.builtin('ellipse'))
    field = vf.DistanceField.from_config(vf.CoreConfig())
    sample = field.sample(curve, [3.0, 0.0])
    assert sample.Phi @ sample.n < 0.0
    shaped = field.shape(sample, 0.5)
    assert np.linalg.norm(shaped.Phi_gamma) == pytest.approx(0.5 * 0.4)
    assert np.linalg.norm(sample.Phi_gamma) == pytest.approx(0.4)


def test_terminal_profile_is_odd_and_bounded():
    """Phi is odd, |phi| <= 1, and 1 beyond the braking distance."""
    profile = vf.TerminalProfile(0.4, 0.05, 0.5, 0.1)
    sigma = np.linspace(-2.0, 2.0, 81)
    phi = profile(sigma)
    np.testing.assert_allclose(phi, -profile(-sigma))
    assert np.all(np.abs(phi) <= 1.0) and profile(1.0) == 1.0


def test_streamline_of_a_python_field():
    """A streamline of a rotation field stays on its circle; callback errors propagate."""
    pts = vf.streamline(lambda x: np.array([-x[1], x[0]]), [1.0, 0.0], 0.01, max_points=300)
    assert pts.shape == (300, 2)
    np.testing.assert_allclose(np.linalg.norm(pts, axis=1), 1.0, atol=1e-8)

    def broken(x):
        raise RuntimeError('field failed')

    with pytest.raises(RuntimeError, match='field failed'):
        vf.streamline(broken, [1.0, 0.0], 0.01)


def test_feedback_linearization_law_and_matrix():
    """FL (v, omega) = M(psi) F, and M^-1 maps it back to F."""
    law_in = vf.LawInput(pose=vf.Pose2(0.0, 0.0, 0.3), F=[0.3, 0.1])
    out = vf.feedback_linearization_law(law_in, 0.2)
    m = vf.fl_matrix(0.3, 0.2)
    np.testing.assert_allclose([out.cmd.vx, out.cmd.wz], m @ [0.3, 0.1])
    np.testing.assert_allclose(vf.fl_inverse(0.3, 0.2) @ m, np.eye(2), atol=1e-12)


def test_heading_tracking_law_with_python_probe():
    """The psi_d feedforward comes from the Python probe; its errors propagate."""
    law_in = vf.LawInput(pose=vf.Pose2(0.0, 0.0, 0.0), F=[0.4, 0.0], hold_speed=0.02)

    def probe(x, time_offset):
        return vf.ProbeSample(F=[0.4, 0.0], psi_d=0.5 * x[1], s_star=x[0], D=0.0)

    out = vf.heading_tracking_law(law_in, probe, 2.0)
    assert out.cmd.vx == pytest.approx(0.4)
    assert out.psi_d_dot == pytest.approx(0.0, abs=1e-12)
    assert vf.heading_tracking_law(law_in, None, 2.0).cmd.vx == pytest.approx(0.4)

    def broken(x, time_offset):
        raise KeyError('probe')

    with pytest.raises(KeyError):
        vf.heading_tracking_law(law_in, broken, 2.0)


def test_control_law_resolution():
    """AUTO resolves per model; invalid pairs raise ConfigError."""
    config = vf.CoreConfig()
    assert vf.ControlLaw(config).law == vf.TrackingLaw.FEEDBACK_LINEARIZATION
    config.motion.motion_model = 'omnidirectional'
    assert vf.ControlLaw(config).law == vf.TrackingLaw.HOLONOMIC
    config.motion.tracking_law = 'feedback_linearization'
    with pytest.raises(vf.ConfigError):
        vf.ControlLaw(config)


def test_command_limiter_bounds_acceleration():
    """From rest, one step of the limiter moves at most a dt along each axis."""
    kin = vf.Kinematics.from_config(vf.CoreConfig())
    state = vf.LimiterState(valid=True)
    cmd, nxt, status, alpha = vf.CommandLimiter().limit(vf.Twist2(0.5, 0.0, 1.5), state, 0.05, kin)
    assert cmd.vx <= kin.max_linear_accel * 0.05 + 1e-12
    assert abs(cmd.wz) <= kin.max_angular_accel * 0.05 + 1e-12
    assert 0.0 < alpha < 1.0 and status == 0 and nxt.valid
    assert nxt.u_prev.vx == cmd.vx


def test_command_limiter_brakes_translation_on_its_own():
    """brake_translation: v brakes at a_dec while omega ramps at alpha_max."""
    kin = vf.Kinematics.from_config(vf.CoreConfig())
    state = vf.LimiterState(valid=True, u_prev=vf.Twist2(0.09, 0.0, 0.0))
    target = vf.Twist2(0.0, 0.0, 1.5)
    shared = vf.CommandLimiter().limit(target, state, 0.05, kin)[0]
    cmd = vf.CommandLimiter().limit(target, state, 0.05, kin, brake_translation=True)[0]
    assert cmd.vx == pytest.approx(0.09 - kin.max_linear_decel * 0.05, abs=1e-15)
    assert cmd.wz == pytest.approx(kin.max_angular_accel * 0.05, abs=1e-15)
    assert shared.vx > cmd.vx


def test_skid_slip_compensation():
    """skid_yaw_rate_efficiency and skid_icr_offset: CoreConfig fields, Kinematics compensation."""
    config = vf.CoreConfig()
    assert config.motion.skid_yaw_rate_efficiency == 1.0 and config.motion.skid_icr_offset == 0.0
    config.motion.motion_model = 'skid_steer'
    config.motion.skid_yaw_rate_efficiency = 0.75
    config.motion.skid_icr_offset = 0.05
    kin = vf.Kinematics.from_config(config)
    assert kin.skid_yaw_rate_efficiency == 0.75 and kin.skid_icr_offset == 0.05
    sent, saturated = kin.slip_compensated(vf.Twist2(0.3, 0.0, 0.6))
    assert sent.wz == pytest.approx(0.8) and not saturated
    sent, _ = kin.slip_compensated(vf.Twist2(0.3, 0.0, 0.6), lever=0.2)
    assert sent.wz == pytest.approx(0.6 * 0.2 / 0.15 / 0.75)
    sent, saturated = kin.slip_compensated(vf.Twist2(0.0, 0.0, 1.4))
    assert sent.wz == kin.max_angular_vel and saturated


def test_integrate_exact_arc():
    """sim.integrate follows the exact arc of a constant twist."""
    pose = vf.sim.integrate(vf.Kinematics(), (0.0, 0.0, 0.0), (0.5, 0.0, 0.5), math.pi)
    np.testing.assert_allclose([pose.x, pose.y, pose.yaw], [1.0, 1.0, math.pi / 2], atol=1e-12)
    skid = vf.sim.integrate(
        vf.Kinematics(), vf.Pose2(), vf.Twist2(0.0, 0.0, 1.0), 1.0, vf.sim.SlipParams(eta_psi=0.5)
    )
    assert skid.yaw == pytest.approx(0.5)
