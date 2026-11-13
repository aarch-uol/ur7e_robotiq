"""Policy vision compatibility. No ROS or UI dependencies.

Mirrors the observation shape and band_sorting contract of the scraping runner.
Publisher counts are graph evidence only; the runner validates payloads and age.
"""
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Callable, Optional

import yaml

MIDPOINTS = '/scraping_vision/world_midpoints'
FIXED_BANDS = '/scraping_vision/fixed_band_targets'


@dataclass(frozen=True)
class VisionIssue:
    detail: str
    blocking: bool = False


@dataclass(frozen=True)
class VisionNeeds:
    kind: str
    clusters_topic: str = MIDPOINTS
    fixed_topic: str = FIXED_BANDS
    band_sorting: bool = False
    vial_bottom_z_set: bool = False
    issues: tuple = ()


def _mapping(value, label):
    if not isinstance(value, dict):
        raise ValueError(f'{label} must be a mapping')
    return value


def _runtime_vision_config(robot):
    # Runner runtime_config: structured nulls keep defaults; legacy flat keys
    # (short names or node attributes) override sections, in file order.
    mappings = {
        'topics': {'clusters': 'cluster_topic', 'fixed_band_targets': 'fixed_band_targets_topic'},
        'band_sorting': {'enabled': 'band_sorting_enabled', 'vial_bottom_z': 'band_sorting_vial_bottom_z'},
    }
    values = {'cluster_topic': MIDPOINTS, 'fixed_band_targets_topic': FIXED_BANDS,
              'band_sorting_enabled': False, 'band_sorting_vial_bottom_z': None}
    aliases = {}
    for section, fields in mappings.items():
        raw = robot.get(section)
        fields_config = {} if raw is None else _mapping(raw, section)
        for key, attr in fields.items():
            aliases[key] = aliases[attr] = attr
            if fields_config.get(key) is not None:
                values[attr] = fields_config[key]
    for key, value in robot.items():
        if key in aliases:
            values[aliases[key]] = value
    return values


_CACHE: dict = {}


def _stamp(path):
    try:
        return Path(path).stat().st_mtime_ns if path else None
    except OSError:
        return None


def vision_needs(manifest) -> VisionNeeds:
    """What band data the policy needs. Cached until the manifest's config files change (AUTOMATION polls this)."""
    files = manifest.expanded_files()
    key = (manifest.name, files.get('training_config'), _stamp(files.get('training_config')),
           files.get('robot_config'), _stamp(files.get('robot_config')), bool(manifest.command))
    if key not in _CACHE:
        if len(_CACHE) > 64:
            _CACHE.clear()
        _CACHE[key] = _vision_needs(manifest, files)
    return _CACHE[key]


def _vision_needs(manifest, files) -> VisionNeeds:
    """Read files relative to the manifest, as the runner command does.

    Training observation shape wins over robot_config.observations. An explicit
    command with no config files has no inspectable scraping contract.
    """
    if not files.get('training_config'):
        issues = () if manifest.command and not files.get('robot_config') else (
            VisionIssue(f'{manifest.name}: cannot check vision without a training config'),)
        return VisionNeeds('unknown', issues=issues)
    try:
        training = _mapping(yaml.safe_load(Path(files['training_config']).read_text()), 'training config')
        robot = (_mapping(yaml.safe_load(Path(files['robot_config']).read_text()) or {}, 'robot config')
                 if files.get('robot_config') else {})
        obs = _mapping(training.get('observation', {}), 'observation')
        runtime = _runtime_vision_config(robot)
        mode = str(obs.get('cluster_mode', 'z_quantile')).strip().lower()
        try:
            count = max(0, int(obs.get('num_clusters', training.get('num_clusters', 3))))
        except (TypeError, ValueError):
            count = 3  # runner ObservationConfig default
        kind = 'fixed_band_targets' if mode == 'fixed_band_targets' else 'clusters' if count else 'none'
        enabled = bool(runtime['band_sorting_enabled'])
        try:
            bottom_set = math.isfinite(float(runtime['band_sorting_vial_bottom_z']))
        except (TypeError, ValueError):
            bottom_set = False
        issues = []
        refusal = None
        if mode == 'occupancy_histogram':          # accepted by the runner, but nothing on the robot provides it
            issues.append(VisionIssue(f'{manifest.name}: occupancy_histogram has no real-robot vision source'))
        if mode not in ('fixed_band_targets', 'z_quantile', 'fixed_z_quantile', 'kmeans', 'occupancy_histogram'):
            refusal = f"unknown observation.cluster_mode: {mode}"
        elif kind == 'fixed_band_targets':
            try:
                bins = max(1, int(obs.get('occupancy_bins', 8)))
            except (TypeError, ValueError):
                bins = 8
            components = obs.get('cluster_position_components')
            axes = tuple(str(c).strip().lower() for c in components) if isinstance(components, (list, tuple)) else ()
            if bins != 8:
                refusal = 'fixed_band_targets requires observation.occupancy_bins: 8'
            elif axes != ('z',):
                refusal = 'fixed_band_targets requires observation.cluster_position_components: [z]'
        if refusal:
            issues.append(VisionIssue(f'{manifest.name}: runner would refuse to start: {refusal}', True))
        if enabled and kind != 'fixed_band_targets':
            issues.append(VisionIssue(f'{manifest.name}: runner would refuse to start: band_sorting requires '
                                      'observation.cluster_mode: fixed_band_targets', True))
        if enabled and not bottom_set:
            issues.append(VisionIssue(f'{manifest.name}: runner would refuse to start: band_sorting is on but '
                                      'vial_bottom_z is not a finite measurement in its robot config', True))
        return VisionNeeds(kind, str(runtime['cluster_topic']),
                           str(runtime['fixed_band_targets_topic']), enabled, bottom_set, tuple(issues))
    except (OSError, ValueError, yaml.YAMLError) as exc:
        return VisionNeeds('unknown', issues=(VisionIssue(f'{manifest.name}: cannot check vision: {exc}'),))


def vision_issues(manifest, publishers: Optional[Callable[[str], int]] = None):
    """Known startup refusals plus nonblocking live graph warnings, if requested."""
    needs = vision_needs(manifest)
    issues = list(needs.issues)
    if publishers is None or needs.kind in ('none', 'unknown') or any(i.blocking for i in issues):
        return issues
    try:
        if needs.kind == 'fixed_band_targets':
            if publishers(needs.fixed_topic) > 0:
                return issues
            if not needs.band_sorting:
                issues.append(VisionIssue(f'{manifest.name} needs fixed band targets: nothing publishes '
                                          f'{needs.fixed_topic} and band_sorting is off in its robot config'))
            elif publishers(needs.clusters_topic) <= 0:
                issues.append(VisionIssue(f'{manifest.name}: band_sorting is on but the vision node is not '
                                          f'running (no publisher on {needs.clusters_topic})'))
        elif publishers(needs.clusters_topic) <= 0:
            issues.append(VisionIssue(f'{manifest.name} reads clusters but the vision node is not running '
                                      f'(no publisher on {needs.clusters_topic})'))
    except Exception as exc:  # ROS graph can disappear during shutdown
        issues.append(VisionIssue(f'{manifest.name}: cannot check publishers: {exc}'))
    return issues
