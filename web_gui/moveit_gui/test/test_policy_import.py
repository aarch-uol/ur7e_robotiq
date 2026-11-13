import json
import sys

import pytest
import yaml

from moveit_gui.policies import (PolicyImportError, PolicyLibrary, PolicyManifest, list_policy_checkpoints,
                                 policy_range_warnings)


def helper_script(path):
    path.write_text(
        "import json, pathlib, sys\n"
        "if sys.argv[1] == '--list': print(json.dumps([str(pathlib.Path(sys.argv[2]) / 'checkpoint.zip')]))\n"
        "else:\n"
        " out = pathlib.Path(sys.argv[sys.argv.index('--out') + 1])\n"
        " out.write_text(json.dumps({'version': 1, 'model': sys.argv[1], 'normalization': None, "
        "'training_config': str(out.parent / 'config.yaml'), 'algorithm': 'PPO', 'recurrent': False, "
        "'control_hz': 5.0, 'action_names': ['dx', 'dz', 'dpitch'], 'observation_dim': 52, "
        "'obs_terms': [], 'physical_ranges': {'ft_sensor': {'min': [-1] * 6, 'max': [1] * 6}, "
        "'ee_pos_z': {'min': 0.1, 'max': 0.2}}, 'warn_sigma': 4.0}))\n"
    )


def test_imports_card_renames_and_edits_without_overwriting(tmp_path):
    source, library_dir = tmp_path / "source", tmp_path / "policies"
    source.mkdir()
    library_dir.mkdir()
    checkpoint = source / "checkpoint.zip"
    checkpoint.write_bytes(b"model")
    helper = tmp_path / "helper.py"
    helper_script(helper)
    lib = PolicyLibrary(library_dir, allowed_roots=[source], card_command=[sys.executable, str(helper)])

    assert list_policy_checkpoints(source, [sys.executable, str(helper)]) == [checkpoint.resolve()]
    result = lib.import_policy(source, checkpoint, "sample_policy", "first policy")
    assert result["card_error"] == ""
    manifest = PolicyManifest.load(library_dir / "sample_policy.yaml")
    assert manifest.policy_card and manifest.card()["algorithm"] == "PPO"

    with pytest.raises(PolicyImportError, match="already exists"):
        lib.import_policy(source, checkpoint, "sample_policy")
    lib.edit_manifest("sample_policy", "renamed_policy", "updated description")
    assert not (library_dir / "sample_policy.yaml").exists()
    assert lib.manifests["renamed_policy"].description == "updated description"


def test_import_rejects_a_folder_outside_allowed_roots(tmp_path):
    allowed = tmp_path / "allowed"
    outside = tmp_path / "outside"
    allowed.mkdir()
    outside.mkdir()
    lib = PolicyLibrary(tmp_path / "policies", allowed_roots=[allowed])
    with pytest.raises(PolicyImportError, match="outside"):
        lib.import_policy(outside, outside / "checkpoint.zip", "sample_policy")


def test_import_keeps_a_manifest_when_card_helper_fails(tmp_path):
    source, library_dir = tmp_path / "source", tmp_path / "policies"
    source.mkdir()
    library_dir.mkdir()
    checkpoint = source / "checkpoint.zip"
    checkpoint.write_bytes(b"model")
    helper = tmp_path / "helper.py"
    helper.write_text("raise SystemExit('helper unavailable')\n")
    lib = PolicyLibrary(library_dir, allowed_roots=[source], card_command=[sys.executable, str(helper)])

    result = lib.import_policy(source, checkpoint, "without_card")

    assert result["card_error"]
    assert PolicyManifest.load(library_dir / "without_card.yaml").expanded_files().get("normalization") is None


def test_policy_range_warnings_use_robot_config_axes_and_scale(tmp_path):
    card = tmp_path / "policy_card.json"
    card.write_text(json.dumps({"version": 1, "physical_ranges": {
        "ft_sensor": {"min": [-1] * 6, "max": [1] * 6}, "ee_pos_z": {"min": 0.1, "max": 0.2}}}))
    config = tmp_path / "robot.yaml"
    config.write_text(yaml.safe_dump({"observations": {"ft_obs_axes": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                                                          "ft_obs_scale": [1, 1, 1, 1, 1, 1]}}))
    manifest = PolicyManifest(name="sample_policy", command=["true"], required_mode="position",
                              files={"policy_card": str(card), "robot_config": str(config)})
    warnings = policy_range_warnings(manifest, [2, 0, 0, 0, 0, 0], 0.3)
    assert any("F/T outside training range" in x and "2" in x for x in warnings)
    assert any("TCP z" in x and "0.1..0.2" in x for x in warnings)


def test_policy_range_warnings_prefer_the_match_simulation_result(tmp_path):
    card = tmp_path / "policy_card.json"
    card.write_text(json.dumps({"version": 1, "physical_ranges": {
        "ft_sensor": {"min": [-1] * 6, "max": [1] * 6}}}))
    config = tmp_path / "robot.yaml"
    config.write_text(yaml.safe_dump({"observations": {"ft_obs_axes": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                                                          "ft_obs_scale": [1] * 6}}))
    manifest = PolicyManifest(name="sample_policy", command=["true"], required_mode="position",
                              files={"policy_card": str(card), "robot_config": str(config)})
    # sensor +X push = policy -Y with the turned tool: fy -2 is out of range, fx is not
    result = {"ft_obs_axes": [[0, 1, 0], [-1, 0, 0], [0, 0, 1]], "ft_obs_scale": [1] * 6}
    warnings = policy_range_warnings(manifest, [2, 0, 0, 0, 0, 0], ft_result=result)
    assert any("fy -2" in x for x in warnings) and not any("fx " in x for x in warnings)


def test_policy_range_warnings_skip_ft_without_measured_axes(tmp_path):
    card = tmp_path / "policy_card.json"
    card.write_text(json.dumps({"version": 1, "physical_ranges": {
        "ft_sensor": {"min": [-1] * 6, "max": [1] * 6}}}))
    manifest = PolicyManifest(name="sample_policy", command=["true"], required_mode="position",
                              files={"policy_card": str(card)})
    warnings = policy_range_warnings(manifest, [50, 0, 0, 0, 0, 0])
    assert warnings == ["F/T axes not measured (SETUP > F/T + payload > Match simulation): F/T range not checked"]
