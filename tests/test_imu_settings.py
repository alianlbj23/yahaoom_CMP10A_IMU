# Copyright 2026 alianlbj23
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

"""Tests for the user-editable launcher settings in ``imu.py``."""

import json

import imu

import pytest

from test_imu_launcher import FakeDocker, _has_pair, make_launcher


@pytest.fixture
def docker(monkeypatch):
    """Replace subprocess.run with a recording fake."""
    fake = FakeDocker()
    monkeypatch.setattr(imu.subprocess, 'run', fake)
    return fake


# ------------------------------------------------------------- defaults
def test_default_settings_only_expose_the_imu():
    """By default nothing but the IMU frame is published: no world TF."""
    s = imu.Settings()
    assert s.imu_topic == '/imu/data'
    assert s.frame_id == 'imu_link'
    assert s.publish_tf is False
    assert s.network == 'bridge'
    assert s.domain_id == 0


# ----------------------------------------------------------- validation
@pytest.mark.parametrize('text', ['/imu', '/imu/data', 'imu',
                                  '/robot1/imu_raw'])
def test_validate_topic_accepts_ros_names(text):
    """Check: validate topic accepts ros names."""
    assert imu.validate_topic(text) == text


@pytest.mark.parametrize('text', ['', '/', '/imu/', '//imu', '/1imu',
                                  '/imu data', '/imu;rm', '/imu:=x',
                                  'imu-data'])
def test_validate_topic_rejects_bad_names(text):
    """Check: validate topic rejects bad names."""
    with pytest.raises(imu.LauncherError):
        imu.validate_topic(text)


@pytest.mark.parametrize('text', ['imu_link', 'imu', 'base_imu_1'])
def test_validate_frame_accepts_simple_names(text):
    """Check: validate frame accepts simple names."""
    assert imu.validate_frame(text) == text


@pytest.mark.parametrize('text', ['', 'imu link', '/imu_link', 'imu/link',
                                  '1imu', 'imu:=x'])
def test_validate_frame_rejects_bad_names(text):
    """Check: validate frame rejects bad names."""
    with pytest.raises(imu.LauncherError):
        imu.validate_frame(text)


@pytest.mark.parametrize('text,value', [('0', 0), ('42', 42), ('232', 232)])
def test_validate_domain_id_accepts_the_ros_range(text, value):
    """Check: validate domain id accepts the ros range."""
    assert imu.validate_domain_id(text) == value


@pytest.mark.parametrize('text', ['', '-1', '233', '1.5', 'abc', '0x10'])
def test_validate_domain_id_rejects_out_of_range(text):
    """Check: validate domain id rejects out of range."""
    with pytest.raises(imu.LauncherError):
        imu.validate_domain_id(text)


def test_validate_network():
    """Check: validate network."""
    assert imu.validate_network('host') == 'host'
    assert imu.validate_network('bridge') == 'bridge'
    with pytest.raises(imu.LauncherError):
        imu.validate_network('none')


# ---------------------------------------------------------- persistence
def test_load_settings_missing_file_gives_defaults(tmp_path):
    """Check: load settings missing file gives defaults."""
    assert imu.load_settings(str(tmp_path / 'nope.json')) == imu.Settings()


def test_save_then_load_roundtrip(tmp_path):
    """Check: save then load roundtrip."""
    path = str(tmp_path / 'config' / 'launcher.json')
    s = imu.Settings(imu_topic='/imu', frame_id='my_imu', publish_tf=True,
                     network='host', domain_id=7)
    imu.save_settings(path, s)
    assert imu.load_settings(path) == s


@pytest.mark.parametrize('content', [
    'not json', '[]', '{"domain_id": 999}', '{"network": "weird"}',
    '{"frame_id": "a b"}', '{"imu_topic": 5}', '{"publish_tf": "yes"}',
])
def test_load_settings_rejects_invalid_files(tmp_path, content):
    """A hand-edited or corrupted file is reported, never silently used."""
    path = tmp_path / 'launcher.json'
    path.write_text(content)
    with pytest.raises(imu.LauncherError) as excinfo:
        imu.load_settings(str(path))
    assert str(path) in str(excinfo.value)


# ------------------------------------------------------- docker run args
def test_run_args_pass_settings_to_the_driver():
    """Check: run args pass settings to the driver."""
    s = imu.Settings(imu_topic='/imu', frame_id='my_imu', publish_tf=True,
                     network='bridge', domain_id=7)
    args = imu.build_run_args('linux', '/dev/ttyUSB0', 'foxglove', False,
                              '/repo', {}, settings=s)
    assert _has_pair(args, '-e', 'ROS_DOMAIN_ID=7')
    assert 'imu_topic:=/imu' in args
    assert 'frame_id:=my_imu' in args
    assert 'publish_tf:=true' in args
    assert '--network' not in args
    assert _has_pair(args, '-p', '127.0.0.1:8765:8765')


def test_run_args_default_settings_turn_the_world_tf_off():
    """Check: run args default settings turn the world tf off."""
    args = imu.build_run_args('linux', '/dev/ttyUSB0', 'foxglove', False,
                              '/repo', {})
    assert 'publish_tf:=false' in args
    assert 'frame_id:=imu_link' in args
    assert 'imu_topic:=/imu/data' in args
    assert _has_pair(args, '-e', 'ROS_DOMAIN_ID=0')


@pytest.mark.parametrize('mode', ['foxglove', 'rviz', 'calibrate', 'shell'])
def test_run_args_host_network_has_no_port_mapping(mode):
    """Host networking shares the host stack: -p is meaningless and omitted."""
    s = imu.Settings(network='host', domain_id=3)
    args = imu.build_run_args('linux', '/dev/ttyUSB0', mode, False, '/repo',
                              {}, settings=s)
    assert _has_pair(args, '--network', 'host')
    # DDS shared memory between containers needs a shared IPC namespace.
    assert _has_pair(args, '--ipc', 'host')
    assert '-p' not in args
    assert _has_pair(args, '-e', 'ROS_DOMAIN_ID=3')


# ------------------------------------------------------------------ menu
def test_settings_menu_changes_and_saves_every_field(docker, tmp_path,
                                                     capsys):
    """Check: settings menu changes and saves every field."""
    answers = ['1', '/imu', '2', 'my_imu', '3', '4', '5', '9', '0']
    launcher = make_launcher(answers=answers, tmp_path=tmp_path)
    launcher.settings_menu()
    saved = json.loads((tmp_path / 'config' / 'launcher.json').read_text())
    assert saved == {'imu_topic': '/imu', 'frame_id': 'my_imu',
                     'publish_tf': True, 'network': 'host', 'domain_id': 9}
    out = capsys.readouterr().out
    assert '/imu' in out and 'my_imu' in out


def test_settings_menu_rejects_bad_input_keeps_old_value(docker, tmp_path,
                                                         capsys):
    """Check: settings menu rejects bad input and keeps the old value."""
    answers = ['5', '999', '2', 'bad name', '0']
    launcher = make_launcher(answers=answers, tmp_path=tmp_path)
    launcher.settings_menu()
    assert imu.load_settings(str(tmp_path / 'config' / 'launcher.json')) \
        == imu.Settings()
    assert '0' in capsys.readouterr().out


def test_settings_menu_can_restore_defaults(docker, tmp_path):
    """Check: settings menu can restore defaults."""
    path = tmp_path / 'config' / 'launcher.json'
    imu.save_settings(str(path), imu.Settings(domain_id=5, network='host'))
    launcher = make_launcher(answers=['6', 'y', '0'], tmp_path=tmp_path)
    launcher.settings_menu()
    assert imu.load_settings(str(path)) == imu.Settings()


def test_start_uses_the_saved_settings(docker, tmp_path):
    """Check: start uses the saved settings."""
    imu.save_settings(str(tmp_path / 'config' / 'launcher.json'),
                      imu.Settings(imu_topic='/imu', network='host',
                                   domain_id=4))
    # Host networking: the LAN question is not asked, so no answers are needed
    # beyond choosing the serial port.
    launcher = make_launcher(answers=['1'], tmp_path=tmp_path)
    launcher.start('foxglove')
    args = docker.run_call()[0]
    assert _has_pair(args, '--network', 'host')
    assert _has_pair(args, '-e', 'ROS_DOMAIN_ID=4')
    assert 'imu_topic:=/imu' in args


def test_main_menu_offers_the_settings_entry():
    """Check: main menu offers the settings entry."""
    assert any('設定' in text for _, text in imu.MENU)
