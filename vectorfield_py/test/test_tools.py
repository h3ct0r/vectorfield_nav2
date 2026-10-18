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

"""The vf-curve, vf-plot, vf-sim and vf-tune CLIs, called through main(argv)."""

import csv
import math
import sys

import pytest
import vectorfield as vf
from vectorfield_tools import _cli, config, curves, vf_curve, vf_plot, vf_sim, vf_tune

CLIS = {'vf-curve': vf_curve, 'vf-plot': vf_plot, 'vf-sim': vf_sim, 'vf-tune': vf_tune}


def _needs(*modules):
    for module in modules:
        pytest.importorskip(module)


def _column(path, name):
    with open(path, encoding='utf-8', newline='') as stream:
        return [float(row[name]) for row in csv.DictReader(stream)]


def _line_csv(tmp_path):
    path = tmp_path / 'line.csv'
    path.write_text('x,y\n-3,0\n3,0\n', encoding='utf-8')
    return path


@pytest.mark.parametrize('prog', sorted(CLIS))
def test_help_for_every_cli(prog, capsys):
    """--help prints the usage line and exits 0."""
    with pytest.raises(SystemExit) as info:
        CLIS[prog].main(['--help'])
    assert info.value.code == 0
    assert f'usage: {prog}' in capsys.readouterr().out


def test_vf_curve_describes_and_round_trips_exports(tmp_path, capsys):
    """--export writes a Curve.msg YAML that reads back as the same spec, and a CSV polyline."""
    _needs('yaml')
    yaml_path, csv_path = tmp_path / 'ellipse.yaml', tmp_path / 'ellipse.csv'
    argv = ['--curve', 'ellipse:a=2,b=1', '--export', str(yaml_path), '--export', str(csv_path)]
    assert vf_curve.main(argv) == 0
    out = capsys.readouterr().out
    assert 'length       9.6884 m' in out and 'regular      yes' in out
    spec = curves.read_curve_file(yaml_path)
    assert spec == vf.CurveSpec.builtin('ellipse', a=2.0, b=1.0)
    polyline = curves.read_curve_file(csv_path)
    assert polyline.type == vf.CurveSpec.Type.POLYLINE and polyline.closed
    assert vf.make_curve(polyline).length() == pytest.approx(9.6884, rel=1e-4)
    assert vf_curve.main(['--curve', str(csv_path), '--export', str(tmp_path / 'p.yaml')]) == 0
    again = curves.read_curve_file(tmp_path / 'p.yaml')
    assert again.closed and len(again.points) == len(polyline.points)

    expression = tmp_path / 'expr.yaml'
    assert (
        vf_curve.main(
            ['--curve', 's; 0.5*sin(s)', '--s-range', '0:6', '--export', str(expression)]
        )
        == 0
    )
    spec = curves.read_curve_file(expression)
    assert spec.type == vf.CurveSpec.Type.EXPRESSION and not spec.closed
    assert (spec.x_expression, spec.s_max) == ('s', 6.0)


def test_vf_curve_rejects_bad_curves(capsys):
    """An invalid expression or parameter, or an unknown name, exits 2 with the reason."""
    assert vf_curve.main(['--curve', 'cos(s; sin(s)']) == 2
    assert 'INVALID_CURVE' in capsys.readouterr().err
    assert vf_curve.main(['--curve', 'ellipse:a=-1']) == 2
    assert vf_curve.main(['--curve', 'no_such_shape']) == 2
    assert 'not a built-in' in capsys.readouterr().err


def test_vf_curve_plot(tmp_path):
    """--plot -o writes a figure."""
    _needs('matplotlib')
    out = tmp_path / 'curve.png'
    assert vf_curve.main(['--curve', 'lemniscate_gerono', '--plot', '-o', str(out)]) == 0
    assert out.stat().st_size > 1000


def test_vf_sim_gif_converges(tmp_path, capsys):
    """Section 11: differential on the ellipse converges; the GIF is small and animated."""
    _needs('matplotlib', 'PIL')
    from PIL import Image

    gif, data = tmp_path / 'run.gif', tmp_path / 'run.csv'
    argv = ['--model', 'differential', '--curve', 'ellipse', '--gif', str(gif), '--csv', str(data)]
    assert vf_sim.main(argv) == 0
    D = _column(data, 'D')
    assert abs(D[-1]) < 0.05 and abs(D[0]) > 0.5
    assert 'phases               FOLLOW' in capsys.readouterr().out
    assert gif.stat().st_size < 3e6
    with Image.open(gif) as image:
        assert image.format == 'GIF' and image.n_frames >= 50


@pytest.mark.parametrize('model', ['differential', 'skid_steer', 'omnidirectional', 'ackermann'])
def test_vf_sim_models_converge_on_a_circle(model, tmp_path):
    """Every model converges (skid-steer keeps the offset of its slip, below 0.07 m)."""
    data = tmp_path / 'run.csv'
    argv = ['--model', model, '--curve', 'ellipse:a=1.5,b=1.5', '--csv', str(data)]
    assert vf_sim.main(argv) == 0
    assert abs(_column(data, 'D')[-1]) < (0.07 if model == 'skid_steer' else 0.01)


def test_vf_sim_open_path_around_an_obstacle(tmp_path, capsys):
    """An open path blocked by a disc: the contour engages and the goal is reached."""
    _needs('matplotlib')
    plot = tmp_path / 'run.svg'
    argv = ['--curve', str(_line_csv(tmp_path)), '--circle', '0,0.05,0.3', '-o', str(plot)]
    assert vf_sim.main(argv) == 0
    out = capsys.readouterr().out
    assert '(settled)' in out and 'contour active       0.00 s' not in out
    assert plot.read_text(encoding='utf-8').startswith('<?xml')


def test_vf_sim_rejects_an_invalid_configuration(capsys):
    """A configuration error is reported and exits 2."""
    assert vf_sim.main(['--set', 'desired_speed=1.0']) == 2
    assert 'desired_speed_above_max_linear_vel' in capsys.readouterr().err
    assert vf_sim.main(['--pose', '1,2']) == 2


def test_vf_plot_field_obstacles_and_decomposition(tmp_path, capsys):
    """Field plots of a moving curve, of an open path with an obstacle; the decomposition."""
    _needs('matplotlib')
    out = tmp_path / 'field.svg'
    argv = ['--curve', '1.5*cos(s) + 0.1*t; 1.5*sin(s)', '--decompose', '2.3,0.8', '-o', str(out)]
    assert vf_plot.main(argv) == 0
    text = capsys.readouterr().out
    assert 'Phi_T      ( 0.089,  0.031)' in text and '|0.400| m/s' in text
    assert out.stat().st_size > 10000
    out = tmp_path / 'contour.png'
    argv = [
        '--curve',
        str(_line_csv(tmp_path)),
        '--circle',
        '0,0,0.3',
        '--box',
        '1.5,1,0.4,0.4,0.3',
    ]
    assert vf_plot.main(argv + ['--pose', '-1.2,0,0', '-o', str(out)]) == 0
    assert vf_plot.main(['--curve', 'ellipse']) == 2


def test_vf_tune_invalid_config_returns_the_error_code(tmp_path, capsys):
    """A Nav2 file with desired_speed above max_linear_vel exits 2 with a suggestion."""
    _needs('yaml')
    path = tmp_path / 'nav2.yaml'
    path.write_text(
        'controller_server:\n  ros__parameters:\n    controller_frequency: 20.0\n'
        '    FollowPath:\n      plugin: "vectorfield_nav2_controller::VectorFieldController"\n'
        '      desired_speed: 1.0\n      bogus_gain: 2\n',
        encoding='utf-8',
    )
    assert vf_tune.main([str(path)]) == 2
    out = capsys.readouterr().out
    assert 'error [desired_speed_above_max_linear_vel]' in out
    assert 'suggestion: lower desired_speed to 0.5' in out and "'bogus_gain'" in out


def test_vf_tune_advice_and_sweep(capsys):
    """A tight curve gets per-plan advice; a sweep prints one row per value."""
    argv = ['--set', 'controller_frequency=20', '--curve', 'ellipse:a=0.5,b=0.3']
    assert vf_tune.main(argv) == 0
    assert 'warning [fl_path_curvature]' in capsys.readouterr().out
    assert vf_tune.main(['--sweep', 'convergence_gain=1:2:0.5', '--duration', '20']) == 0
    rows = [
        line
        for line in capsys.readouterr().out.splitlines()
        if line.startswith('  ') and '.' in line
    ]
    assert len(rows) == 3
    assert vf_tune.main(['--sweep', 'convergence_gain=2:1:0.5']) == 2


def test_config_mapping_layouts():
    """Nav2, node and flat layouts set the same fields; the costmap gives footprint and size."""
    _needs('yaml')
    nav2 = {
        'controller_server': {
            'ros__parameters': {'controller_frequency': 10.0, 'FollowPath': {'heading_gain': 3}}
        },
        'local_costmap': {
            'local_costmap': {
                'ros__parameters': {
                    'rolling_window': True,
                    'width': 4,
                    'height': 3,
                    'resolution': 0.05,
                    'footprint': '[[0.2, 0.15], [0.2, -0.15], [-0.2, -0.15], [-0.2, 0.15]]',
                }
            }
        },
    }
    loaded = config.config_from_mapping(nav2)
    assert loaded.config.node.controller_frequency == 10.0
    assert loaded.config.motion.heading_gain == 3.0
    assert loaded.config.node.footprint.polygon.shape == (4, 2)
    assert (loaded.costmap.width_m, loaded.costmap.height_m) == (4.0, 3.0)
    flat = config.config_from_mapping({'FollowPath.motion_model': 'skid_steer', 'visualize': True})
    assert flat.config.motion.motion_model == vf.MotionModel.SKID_STEER
    assert flat.ignored == ['visualize']
    node = config.config_from_mapping({'follower': {'ros__parameters': {'desired_speed': 0.3}}})
    assert node.config.field.desired_speed == 0.3
    with pytest.raises(config.InputError):
        config.config_from_mapping({'gain_function': 'bogus'})


@pytest.mark.parametrize(
    'prog,argv',
    [
        ('vf-curve', ['--plot', '-o', 'curve.png']),
        ('vf-plot', ['-o', 'field.png']),
        ('vf-sim', ['--duration', '1', '-o', 'run.png']),
    ],
)
def test_missing_matplotlib_prints_the_install_hint(prog, argv, monkeypatch, tmp_path, capsys):
    """Without matplotlib, the plotting CLIs exit 1 with the [tools] install hint."""
    monkeypatch.chdir(tmp_path)
    monkeypatch.setitem(sys.modules, 'matplotlib', None)
    assert CLIS[prog].main(argv) == 1
    err = capsys.readouterr().err
    assert "pip install './vectorfield_py[tools]'" in err and 'python3-matplotlib' in err


def test_missing_yaml_prints_the_install_hint(monkeypatch, tmp_path, capsys):
    """Without PyYAML, reading a YAML configuration exits 1 with the install hint."""
    path = tmp_path / 'params.yaml'
    path.write_text('desired_speed: 0.3\n', encoding='utf-8')
    monkeypatch.setitem(sys.modules, 'yaml', None)
    assert vf_tune.main([str(path)]) == 1
    assert 'PyYAML is required' in capsys.readouterr().err


def test_option_values_may_start_with_a_minus_sign():
    """--pose -1,0,0 parses on every Python version (argparse alone takes it for an option)."""
    parsed = _cli.parse_args(vf_sim.build_parser(), ['--pose', '-1.2,0,0', '--s-range', '-pi:pi'])
    assert (parsed.pose, parsed.s_range) == ('-1.2,0,0', '-pi:pi')


def test_number_parsing():
    """Numbers accept pi forms; bad input is an InputError."""
    assert _cli.number('-pi/2') == pytest.approx(-math.pi / 2)
    assert _cli.number('3*pi/4') == pytest.approx(0.75 * math.pi)
    assert _cli.numbers('1, 2pi', 2) == pytest.approx([1.0, 2.0 * math.pi])
    with pytest.raises(_cli.InputError):
        _cli.numbers('1,2', 3)
