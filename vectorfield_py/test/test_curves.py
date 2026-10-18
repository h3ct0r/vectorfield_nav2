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

"""Curves: make_curve() for built-ins and expressions, PolylineCurve, errors."""

import copy
import math

import numpy as np
import pytest
import vectorfield as vf


def _ellipse_closest(a, b, x):
    """Closest point of x on [a cos u, b sin u] by dense sampling and Newton steps."""
    u = np.linspace(0.0, 2.0 * math.pi, 20000, endpoint=False)
    d2 = (a * np.cos(u) - x[0]) ** 2 + (b * np.sin(u) - x[1]) ** 2
    t = float(u[np.argmin(d2)])
    for _ in range(50):
        r = np.array([a * math.cos(t), b * math.sin(t)])
        dr = np.array([-a * math.sin(t), b * math.cos(t)])
        ddr = -r
        f = (r - x) @ dr
        df = dr @ dr + (r - x) @ ddr
        t -= f / df
    return np.array([a * math.cos(t), b * math.sin(t)])


@pytest.mark.parametrize('a,b', [(1.0, 1.0), (2.0, 1.0), (0.8, 1.5)])
def test_builtin_ellipse_closest_point_matches_analytic(a, b):
    """The ellipse foot satisfies the stationarity condition (r - x) . r' = 0 exactly."""
    curve = vf.make_curve(vf.CurveSpec.builtin('ellipse', a=a, b=b))
    assert curve.is_closed and not curve.is_time_varying
    rng = np.random.default_rng(1)
    for x in rng.uniform(-3.0, 3.0, size=(40, 2)):
        if abs(x[0]) / a + abs(x[1]) / b < 0.3:
            continue  # near the centre the nearest foot is ill-conditioned
        cp = curve.closest_point(x)
        foot = _ellipse_closest(a, b, x)
        assert cp.valid
        assert cp.D == pytest.approx(np.linalg.norm(x - foot), abs=1e-7)
        np.testing.assert_allclose(cp.x_star, foot, atol=1e-5)
        assert np.linalg.norm(cp.T) == pytest.approx(1.0, abs=1e-12)
        assert abs(cp.n @ cp.T) < 1e-9
        np.testing.assert_allclose(cp.x_star + cp.D * cp.n, x, atol=1e-9)


def test_circle_length_and_stations():
    """A unit circle has length 2 pi; point_at and tangent_at follow the arc length."""
    curve = vf.make_curve(vf.CurveSpec.builtin('ellipse', a=1.0, b=1.0, cx=0.5))
    assert curve.length() == pytest.approx(2.0 * math.pi, rel=1e-9)
    s = np.array([0.0, math.pi / 2, math.pi])
    np.testing.assert_allclose(curve.point_at(s), [[1.5, 0.0], [0.5, 1.0], [-0.5, 0.0]], atol=1e-7)
    np.testing.assert_allclose(curve.tangent_at(0.0), [0.0, 1.0], atol=1e-7)
    samples = curve.sample(ds=0.1)
    assert samples.shape[1] == 2 and samples.shape[0] >= 63
    np.testing.assert_allclose(samples[0], samples[-1], atol=1e-9)


def test_builtin_shapes_are_listed_with_defaults():
    """builtin_shapes() lists every shape; each default builds a valid closed curve."""
    shapes = vf.builtin_shapes()
    names = [s['name'] for s in shapes]
    assert 'ellipse' in names and 'lemniscate_gerono' in names
    for shape in shapes:
        assert {'cx', 'cy', 'yaw'} <= set(shape['parameters'])
        curve = vf.make_curve(vf.CurveSpec.builtin(shape['name']))
        assert curve.is_closed and curve.length() > 0.0
        assert curve.spec.builtin_shape == shape['name']


def _l_polyline():
    return np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]])


@pytest.mark.parametrize('factory', ['direct', 'make_curve'])
def test_polyline_closest_point_and_tangent(factory):
    """Segment projection, both end rays and the wedge tangent of an L-shaped polyline."""
    if factory == 'direct':
        curve = vf.PolylineCurve(_l_polyline())
        assert len(curve) == 3 and curve.segment_count == 2
    else:
        curve = vf.make_curve(vf.CurveSpec.polyline(_l_polyline()))
    assert not curve.is_closed
    assert curve.length() == pytest.approx(4.0)

    cp = curve.closest_point([1.0, 0.5])
    np.testing.assert_allclose(cp.x_star, [1.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(cp.T, [1.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(cp.n, [0.0, 1.0], atol=1e-12)
    assert cp.D == pytest.approx(0.5) and cp.s_star == pytest.approx(1.0)

    back = curve.closest_point([-1.0, 0.3])
    assert back.on_ray and back.piece == vf.PieceKind.BACKWARD_RAY
    assert back.s_star == pytest.approx(-1.0)
    np.testing.assert_allclose(back.x_star, [-1.0, 0.0], atol=1e-12)

    fwd = curve.closest_point([2.5, 3.0])
    assert fwd.piece == vf.PieceKind.FORWARD_RAY and fwd.s_star == pytest.approx(5.0)
    np.testing.assert_allclose(fwd.T, [0.0, 1.0], atol=1e-12)

    wedge = curve.closest_point([3.0, -1.0])
    assert wedge.in_wedge and wedge.D == pytest.approx(math.sqrt(2.0))
    np.testing.assert_allclose(wedge.x_star, [2.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(wedge.T, [math.sqrt(0.5), math.sqrt(0.5)], atol=1e-12)
    assert curve.remaining_to_end(cp) == pytest.approx(3.0)


def test_closest_points_batch_matches_single_queries():
    """closest_points() equals per-point closest_point() without a hint."""
    curve = vf.make_curve(vf.CurveSpec.builtin('lemniscate_gerono', a=2.0))
    pts = np.random.default_rng(2).uniform(-2.5, 2.5, size=(25, 2))
    batch = curve.closest_points(pts)
    for i, x in enumerate(pts):
        cp = curve.closest_point(x)
        assert batch['D'][i] == pytest.approx(cp.D, abs=1e-12)
        np.testing.assert_allclose(batch['x_star'][i], cp.x_star, atol=1e-12)
    assert batch['valid'].dtype == np.bool_ and batch['valid'].all()


def test_hint_round_trip():
    """make_hint() carries the search state; a hinted search keeps the same foot."""
    curve = vf.make_curve(vf.CurveSpec.builtin('ellipse', a=2.0, b=1.0))
    cp = curve.closest_point([2.5, 0.2], vf.EvalContext(t=0.0, direction=1))
    hint = vf.make_hint(cp)
    assert hint.valid and hint.s == pytest.approx(cp.u_star)
    again = curve.closest_point([2.5, 0.21], hint=hint)
    assert again.s_star == pytest.approx(cp.s_star, abs=0.05)


def test_expression_parse_error_raises_curve_error():
    """A muParser syntax error is INVALID_CURVE, a CurveError, a ConfigError and a ValueError."""
    spec = vf.CurveSpec.expression('cos(s', 'sin(s)', 0.0, 2.0 * math.pi, closed=True)
    with pytest.raises(vf.CurveError) as info:
        vf.make_curve(spec)
    assert info.value.code == 'INVALID_CURVE'
    assert isinstance(info.value, vf.ConfigError) and isinstance(info.value, ValueError)


def test_open_time_varying_expression_is_unsupported():
    """An open curve that depends on t is rejected with UNSUPPORTED."""
    spec = vf.CurveSpec.expression('s + 0.1 * t', 'sin(s)', 0.0, 3.0)
    with pytest.raises(vf.CurveError) as info:
        vf.make_curve(spec)
    assert info.value.code == 'UNSUPPORTED'


def test_expression_curve_with_parameters():
    """Declared parameters are constants of the expression; t makes a closed curve moving."""
    spec = vf.CurveSpec.expression(
        'r * cos(s) + 0.1 * t',
        'r * sin(s)',
        0.0,
        2.0 * math.pi,
        closed=True,
        parameters={'r': 1.5},
    )
    assert spec.parameters == {'r': 1.5}
    curve = vf.make_curve(spec)
    assert curve.is_time_varying
    cp = curve.closest_point([3.0, 0.0], vf.EvalContext(t=10.0))
    assert cp.D == pytest.approx(0.5, abs=1e-6)
    np.testing.assert_allclose(curve.velocity(cp, 10.0), [0.1, 0.0], atol=1e-6)
    with pytest.raises(vf.CurveError):
        vf.make_curve(vf.CurveSpec.builtin('ellipse', bogus=1.0))


def test_curves_are_shared_not_copied():
    """Curves have no copy (ExpressionCurve is non-copyable); rebuild one from its spec."""
    spec = vf.CurveSpec.expression('cos(s)', 'sin(s)', 0.0, 2.0 * math.pi, closed=True)
    curve = vf.make_curve(spec)
    with pytest.raises(TypeError):
        copy.copy(curve)
    assert vf.TrackerInput(curve).curve is curve
    twin = vf.make_curve(curve.spec)
    assert twin is not curve and twin.spec == spec
    assert twin.length() == pytest.approx(curve.length())


def test_polyline_errors_and_immutability():
    """Too few vertices raise CurveError; make_curve() polylines refuse in-place edits."""
    with pytest.raises(vf.CurveError):
        vf.PolylineCurve([[0.0, 0.0]])
    curve = vf.PolylineCurve()
    assert curve.empty
    assert curve.assign([[0.0, 0.0], [math.nan, 1.0]]) == vf.AssignStatus.NON_FINITE
    assert curve.assign([[0.0, 0.0], [1.0, 0.0]]) == vf.AssignStatus.OK
    curve.search_policy = vf.SearchPolicy.window_scan(-1.0, 2.0)
    assert curve.search_policy.kind == vf.SearchPolicy.Kind.WINDOW_SCAN
    built = vf.make_curve(vf.CurveSpec.polyline([[0.0, 0.0], [1.0, 0.0]]))
    assert isinstance(built, vf.PolylineCurve)
    with pytest.raises(ValueError):
        built.assign([[0.0, 0.0], [2.0, 0.0]])
    with pytest.raises(IndexError):
        curve.segment_length(5)


def test_conditioning_helpers():
    """Resampling keeps the geometry; kappa_path of a square corner is its turn per length."""
    pts = vf.resample(_l_polyline(), False, 0.5)
    assert pts.shape == (9, 2)
    np.testing.assert_allclose(pts[[0, 4, 8]], _l_polyline())
    assert vf.kappa_path(_l_polyline()) == pytest.approx((math.pi / 2) / 2.0)
    assert vf.dedupe([[0.0, 0.0], [0.0, 1e-4], [1.0, 0.0]]).shape == (2, 2)
