"""Policy vision contracts, independent of ROS and the web."""
import pytest
import yaml

from moveit_gui.policies import PolicyManifest
from moveit_gui.policy_vision import vision_needs, vision_issues


def policy(tmp_path, observation=None, robot=None, training=None):
    if observation and observation.get('cluster_mode') == 'fixed_band_targets':
        observation = {'cluster_position_components': ['z'], **observation}
    (tmp_path / 'training.yaml').write_text(yaml.safe_dump(
        training if training is not None else {'observation': observation or {}}))
    (tmp_path / 'robot.yaml').write_text(yaml.safe_dump(robot or {}))
    return PolicyManifest(name='round44_heldout', required_mode='position', command=['true'],
                          source=tmp_path / 'manifest.yaml',
                          files={'training_config': 'training.yaml', 'robot_config': 'robot.yaml'})


def test_fixed_bands_need_direct_topic_or_configured_conversion(tmp_path):
    m = policy(tmp_path, {'cluster_mode': 'fixed_band_targets'})
    needs = vision_needs(m)
    assert needs.kind == 'fixed_band_targets'
    assert not needs.band_sorting and not needs.vial_bottom_z_set
    issues = vision_issues(m, lambda topic: 0)
    assert len(issues) == 1 and not issues[0].blocking
    assert 'nothing publishes /scraping_vision/fixed_band_targets' in issues[0].detail
    assert 'band_sorting is off' in issues[0].detail
    assert not vision_issues(m, lambda topic: int(topic == '/scraping_vision/fixed_band_targets'))


def test_band_conversion_uses_overridden_midpoint_topic(tmp_path):
    m = policy(tmp_path, {'cluster_mode': 'fixed_band_targets'}, {
        'band_sorting': {'enabled': True, 'vial_bottom_z': 0.0},
        'topics': {'clusters': '/custom/midpoints', 'fixed_band_targets': '/custom/bands'}})
    assert vision_needs(m).vial_bottom_z_set
    issues = vision_issues(m, lambda topic: 0)
    assert len(issues) == 1 and 'band_sorting is on' in issues[0].detail
    assert '/custom/midpoints' in issues[0].detail
    for live in ['/custom/midpoints', '/custom/bands']:
        assert not vision_issues(m, lambda topic: int(topic == live))


@pytest.mark.parametrize('mode', ['kmeans', 'z_quantile', 'fixed_z_quantile'])
def test_legacy_clusters_and_default_count(tmp_path, mode):
    m = policy(tmp_path, {'cluster_mode': mode}, {'observations': {'num_clusters': 0}})
    assert vision_needs(m).kind == 'clusters'  # training shape wins over robot config
    issues = vision_issues(m, lambda topic: 0)
    assert len(issues) == 1 and 'reads clusters' in issues[0].detail
    assert not vision_issues(m, lambda topic: int(topic == '/scraping_vision/world_midpoints'))


def test_zero_clusters_and_legacy_top_level_count(tmp_path):
    m = policy(tmp_path, training={'num_clusters': 0})
    assert vision_needs(m).kind == 'none'
    assert not vision_issues(m, lambda _: 0)
    m = policy(tmp_path, {'cluster_mode': 'fixed_band_targets', 'num_clusters': 0})
    assert vision_needs(m).kind == 'fixed_band_targets'


@pytest.mark.parametrize('bottom', [None, 'bad', float('nan'), float('inf')])
def test_invalid_band_origin_refuses_start_even_with_direct_publisher(tmp_path, bottom):
    m = policy(tmp_path, {'cluster_mode': 'fixed_band_targets'},
               {'band_sorting': {'enabled': True, 'vial_bottom_z': bottom}})
    issues = vision_issues(m, lambda _: 1)
    assert any(i.blocking and 'vial_bottom_z' in i.detail and 'runner would refuse' in i.detail for i in issues)


def test_conversion_requires_fixed_band_policy(tmp_path):
    m = policy(tmp_path, {'cluster_mode': 'kmeans'},
               {'band_sorting': {'enabled': True, 'vial_bottom_z': 0.1}})
    assert any(i.blocking and 'fixed_band_targets' in i.detail for i in vision_issues(m))


@pytest.mark.parametrize('content', ['[broken', '[]', 'null'])
def test_unreadable_training_is_unknown_not_no_vision(tmp_path, content):
    m = policy(tmp_path)
    (tmp_path / 'training.yaml').write_text(content)
    issues = vision_issues(m, lambda _: 1)
    assert vision_needs(m).kind == 'unknown'
    assert any('cannot check vision' in i.detail for i in issues)


def test_graph_failure_does_not_claim_no_publisher(tmp_path):
    m = policy(tmp_path)
    def unavailable(topic):
        raise RuntimeError('graph unavailable')
    issues = vision_issues(m, unavailable)
    assert len(issues) == 1 and 'cannot check publishers' in issues[0].detail
    assert not issues[0].blocking


def test_occupancy_histogram_only_warns(tmp_path):
    m = policy(tmp_path, {'cluster_mode': 'occupancy_histogram'})
    issues = vision_issues(m, lambda _: 1)
    assert len(issues) == 1 and not issues[0].blocking and 'no real-robot vision source' in issues[0].detail


def test_configs_are_read_again_only_when_they_change(tmp_path, monkeypatch):
    import moveit_gui.policy_vision as PV
    m = policy(tmp_path, {'cluster_mode': 'fixed_band_targets'})
    calls = []
    real = PV._vision_needs
    monkeypatch.setattr(PV, '_vision_needs', lambda *a: calls.append(1) or real(*a))
    for _ in range(5):
        vision_needs(m)
    assert len(calls) <= 1
    import os
    robot = tmp_path / 'robot.yaml'
    robot.write_text(yaml.safe_dump({'band_sorting': {'enabled': True, 'vial_bottom_z': 0.01}}))
    st = robot.stat()
    os.utime(robot, ns=(st.st_atime_ns, st.st_mtime_ns + 10**9))
    assert vision_needs(m).band_sorting


@pytest.mark.parametrize('observation', [
    {'cluster_mode': 'fixed_band_targets', 'occupancy_bins': 7},
    {'cluster_mode': 'fixed_band_targets', 'cluster_position_components': ['x', 'z']},
    {'cluster_mode': 'typo'},
])
def test_known_unsupported_observation_contract_refuses_start(tmp_path, observation):
    m = policy(tmp_path, observation)
    assert any(i.blocking and 'runner would refuse' in i.detail for i in vision_issues(m, lambda _: 1))


def test_flat_runtime_keys_override_structured_values(tmp_path):
    m = policy(tmp_path, {'cluster_mode': 'fixed_band_targets'}, {
        'band_sorting': {'enabled': False, 'vial_bottom_z': None},
        'topics': {'clusters': '/ignored'},
        'band_sorting_enabled': True, 'band_sorting_vial_bottom_z': 0.1,
        'cluster_topic': '/custom/midpoints', 'fixed_band_targets_topic': '/custom/bands'})
    assert not vision_issues(m, lambda t: int(t == '/custom/midpoints'))
    assert not vision_issues(m, lambda t: int(t == '/custom/bands'))
    m = policy(tmp_path, {'cluster_mode': 'fixed_band_targets'}, {'band_sorting_enabled': True})
    assert any(i.blocking and 'vial_bottom_z' in i.detail for i in vision_issues(m))


@pytest.mark.parametrize('robot', [
    {'topics': None, 'band_sorting': None},
    {'topics': {'clusters': None, 'fixed_band_targets': None}, 'band_sorting': {'enabled': None}},
])
def test_null_structured_values_keep_runner_defaults(tmp_path, robot):
    m = policy(tmp_path, {'cluster_mode': 'fixed_band_targets'}, robot)
    needs = vision_needs(m)
    assert needs.kind == 'fixed_band_targets'
    assert needs.fixed_topic == '/scraping_vision/fixed_band_targets'
    assert needs.clusters_topic == '/scraping_vision/world_midpoints'
    assert not vision_issues(m, lambda t: int(t == '/scraping_vision/fixed_band_targets'))
