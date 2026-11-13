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
        " out.write_text(json.dumps({'version': 1, 'model': sys.argv[1], 'normalization': str(out.parent / 'vec_normalize.pkl'), "
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
    (source / "vec_normalize.pkl").write_bytes(b"normalizer")
    (source / "config.yaml").write_text("training: true\n")
    helper = tmp_path / "helper.py"
    helper_script(helper)
    lib = PolicyLibrary(library_dir, allowed_roots=[source], card_command=[sys.executable, str(helper)])

    assert list_policy_checkpoints(source, [sys.executable, str(helper)]) == [checkpoint.resolve()]
    result = lib.import_policy(source, checkpoint, "sample_policy", "first policy")
    assert result["card_error"] == ""
    manifest = PolicyManifest.load(library_dir / "sample_policy" / "manifest.yaml")
    assert manifest.policy_card and manifest.card()["algorithm"] == "PPO"
    assert manifest.expanded_files()["model"].endswith("sample_policy/checkpoint.zip")
    assert manifest.expanded_files()["normalization"].endswith("sample_policy/vec_normalize.pkl")
    assert (library_dir / "sample_policy" / "checkpoint.policy_card.json").is_file()

    with pytest.raises(PolicyImportError, match="already exists"):
        lib.import_policy(source, checkpoint, "sample_policy")
    lib.edit_manifest("sample_policy", "renamed_policy", "updated description")
    assert not (library_dir / "sample_policy").exists()
    assert lib.manifests["renamed_policy"].description == "updated description"
    assert (library_dir / "renamed_policy" / "manifest.yaml").is_file()


def test_import_rejects_a_folder_outside_allowed_roots(tmp_path):
    allowed = tmp_path / "allowed"
    outside = tmp_path / "outside"
    allowed.mkdir()
    outside.mkdir()
    lib = PolicyLibrary(tmp_path / "policies", allowed_roots=[allowed])
    with pytest.raises(PolicyImportError, match="outside"):
        lib.import_policy(outside, outside / "checkpoint.zip", "sample_policy")


def test_import_refuses_missing_normalization_unless_opted_out(tmp_path):
    source, library_dir = tmp_path / "source", tmp_path / "policies"
    source.mkdir()
    library_dir.mkdir()
    checkpoint = source / "checkpoint.zip"
    checkpoint.write_bytes(b"model")
    helper = tmp_path / "helper.py"
    helper.write_text("raise SystemExit('helper unavailable')\n")
    lib = PolicyLibrary(library_dir, allowed_roots=[source], card_command=[sys.executable, str(helper)])

    with pytest.raises(PolicyImportError, match="no normalisation file"):
        lib.import_policy(source, checkpoint, "without_card")

    result = lib.import_policy(source, checkpoint, "without_card", no_normalization=True)
    assert result["card_error"]
    manifest = PolicyManifest.load(library_dir / "without_card" / "manifest.yaml")
    assert manifest.no_normalization
    assert manifest.expanded_files().get("normalization") is None


def test_manifest_with_null_normalization_needs_explicit_opt_out(tmp_path):
    path = tmp_path / "bad.yaml"
    path.write_text("name: bad\nrequired_mode: position\nmodel: model.zip\nnormalization: null\n")
    with pytest.raises(ValueError, match="normalization"):
        PolicyManifest.load(path)


def test_checkpoint_listing_is_a_cheap_file_scan(tmp_path):
    folder = tmp_path / "source"
    folder.mkdir()
    checkpoint = folder / "checkpoint.zip"
    checkpoint.write_bytes(b"model")
    helper = tmp_path / "helper.py"
    helper.write_text("raise SystemExit('must not run')\n")
    assert list_policy_checkpoints(folder, [sys.executable, str(helper)]) == [checkpoint.resolve()]


def test_import_without_card_tool_still_saves_a_normalized_policy(tmp_path, monkeypatch):
    monkeypatch.delenv("MOVEIT_GUI_POLICY_CARD_COMMAND", raising=False)
    source, library_dir = tmp_path / "source", tmp_path / "policies"
    source.mkdir()
    checkpoint = source / "checkpoint.zip"
    checkpoint.write_bytes(b"model")
    (source / "vec_normalize.pkl").write_bytes(b"normalizer")
    lib = PolicyLibrary(library_dir, allowed_roots=[source], card_command=None)
    result = lib.import_policy(source, checkpoint, "no_card_tool")
    assert "Policy card not made: card tool not set (--policy-card-command)" in result["msg"]
    assert PolicyManifest.load(library_dir / "no_card_tool" / "manifest.yaml").expanded_files()["normalization"].endswith(
        "no_card_tool/vec_normalize.pkl")


def test_edit_manifest_uses_source_path_and_moves_saved_folder(tmp_path):
    folder = tmp_path / "policies" / "actual-folder"
    folder.mkdir(parents=True)
    source = folder / "manifest.yaml"
    source.write_text("name: handwritten\nrequired_mode: position\ncommand: ['true']\n")
    lib = PolicyLibrary(tmp_path / "policies")
    lib.edit_manifest("handwritten", "renamed")
    assert not folder.exists()
    assert (tmp_path / "policies" / "renamed" / "manifest.yaml").is_file()
    assert lib.manifests["renamed"].source == (tmp_path / "policies" / "renamed" / "manifest.yaml").resolve()


def test_edit_legacy_manifest_uses_actual_filename(tmp_path):
    policies = tmp_path / "policies"
    policies.mkdir()
    source = policies / "handwritten-file.yaml"
    source.write_text("name: handwritten\nrequired_mode: position\ncommand: ['true']\n")
    lib = PolicyLibrary(policies)
    lib.edit_manifest("handwritten", description="edited")
    assert not source.exists()
    assert yaml.safe_load((policies / "handwritten.yaml").read_text())["description"] == "edited"


def test_card_for_another_model_is_not_used(tmp_path):
    card = tmp_path / "card.json"
    card.write_text(json.dumps({"version": 1, "model": "other.zip"}))
    manifest = PolicyManifest(name="sample", command=["true"], required_mode="position",
                              files={"model": str(tmp_path / "model.zip"), "policy_card": str(card)})
    assert manifest.card() is None


def test_import_rejects_symlinked_copied_file(tmp_path):
    source, library_dir = tmp_path / "source", tmp_path / "policies"
    source.mkdir()
    checkpoint = source / "checkpoint.zip"
    checkpoint.write_bytes(b"model")
    normalization = source / "vec_normalize.pkl"
    outside = tmp_path / "outside.pkl"
    outside.write_bytes(b"normalizer")
    normalization.symlink_to(outside)
    lib = PolicyLibrary(library_dir, allowed_roots=[source])
    with pytest.raises(PolicyImportError, match="symlinked policy file"):
        lib.import_policy(source, checkpoint, "symlinked")


def test_uploaded_policy_folder_is_atomic_and_restricted(tmp_path):
    lib = PolicyLibrary(tmp_path / "policies")
    saved = lib.save_uploaded_folder("training_1", [("checkpoint.zip", b"model"), ("vec.pkl", b"norm")])
    assert saved == (tmp_path / "policies" / "uploads" / "training_1").resolve()
    assert (saved / "checkpoint.zip").read_bytes() == b"model"
    with pytest.raises(PolicyImportError, match="file type not allowed"):
        lib.save_uploaded_folder("bad", [("checkpoint.zip", b"model"), ("notes.txt", b"no")])
    with pytest.raises(PolicyImportError, match="already exists"):
        lib.save_uploaded_folder("training_1", [("checkpoint.zip", b"model")])


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
    assert "F/T: checked during the run" in warnings
    assert not any("F/T outside training range" in x for x in warnings)
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
    assert warnings == ["F/T: checked during the run"]


def test_policy_range_warnings_skip_ft_without_measured_axes(tmp_path):
    card = tmp_path / "policy_card.json"
    card.write_text(json.dumps({"version": 1, "physical_ranges": {
        "ft_sensor": {"min": [-1] * 6, "max": [1] * 6}}}))
    manifest = PolicyManifest(name="sample_policy", command=["true"], required_mode="position",
                              files={"policy_card": str(card)})
    warnings = policy_range_warnings(manifest, [50, 0, 0, 0, 0, 0])
    assert warnings == ["F/T: checked during the run"]


def test_policy_range_warnings_explain_missing_robot_config_and_identity_stats(tmp_path):
    card = tmp_path / "policy_card.json"
    card.write_text(json.dumps({"version": 1, "normalization_stats": False,
                                "physical_ranges": {"ft_sensor": {"min": [-1], "max": [1]}}}))
    manifest = PolicyManifest(name="sample", command=["true"], required_mode="position",
                              files={"policy_card": str(card)})
    warnings = policy_range_warnings(manifest, tcp_z=0.2)
    assert warnings == ["Policy ranges not checked: no normalisation statistics"]


def test_policy_range_warnings_skip_tcp_without_robot_config(tmp_path):
    card = tmp_path / "policy_card.json"
    card.write_text(json.dumps({"version": 1, "physical_ranges": {"ee_pos_z": {"min": 0.1, "max": 0.2}}}))
    manifest = PolicyManifest(name="sample", command=["true"], required_mode="position",
                              files={"policy_card": str(card)})
    assert policy_range_warnings(manifest, tcp_z=0.2) == ["TCP height: not checked (no robot config)"]


def test_policy_range_warnings_apply_runner_position_bias(tmp_path):
    card = tmp_path / "policy_card.json"
    card.write_text(json.dumps({"version": 1, "physical_ranges": {"ee_pos_z": {"min": 0.1, "max": 0.2}}}))
    config = tmp_path / "robot.yaml"
    config.write_text(yaml.safe_dump({"observations": {"obs_position_bias_xyz_ee_servo_admittance": [0, 0, 0.1]}}))
    manifest = PolicyManifest(name="sample", command=["true"], required_mode="position", backend="servo_admittance",
                              files={"policy_card": str(card), "robot_config": str(config)})
    assert any("TCP z 0.3" in x for x in policy_range_warnings(manifest, tcp_z=0.2))


def test_import_never_takes_files_from_another_run(tmp_path):
    logs, library_dir = tmp_path / "logs", tmp_path / "policies"
    run_a, run_b = logs / "run_a" / "best_model", logs / "run_b" / "best_model"
    run_a.mkdir(parents=True)
    run_b.mkdir(parents=True)
    (run_a / "best_model.zip").write_bytes(b"model a")
    (run_b / "vec_normalize.pkl").write_bytes(b"norm b")
    lib = PolicyLibrary(library_dir, allowed_roots=[logs], card_command=[])
    with pytest.raises(PolicyImportError, match="no normalisation"):
        lib.import_policy(logs, run_a / "best_model.zip", "a_policy")
    (run_a.parent / "config.yaml").write_text("x: 1\n")
    (run_a / "vec_normalize_best.pkl").write_bytes(b"norm a")
    (run_a / "vec_normalize_last.pkl").write_bytes(b"norm a last")
    lib.import_policy(logs, run_a / "best_model.zip", "a_policy")
    files = lib.manifests["a_policy"].expanded_files()
    assert open(files["normalization"], "rb").read() == b"norm a"
    assert files["training_config"].endswith("a_policy/config.yaml")


def test_skipped_checks_are_notes_not_warnings():
    from moveit_gui.policies import split_range_notes
    w, n = split_range_notes(["F/T: checked during the run", "TCP height: not checked (no robot config)",
                              "TCP z 0.5 m outside training range 0..0.2 m"])
    assert w == ["TCP z 0.5 m outside training range 0..0.2 m"]
    assert len(n) == 2


def test_imported_folder_is_readable_by_other_users(tmp_path):
    src, lib_dir = tmp_path / "src", tmp_path / "policies"
    src.mkdir()
    (src / "m.zip").write_bytes(b"m")
    (src / "vec_normalize.pkl").write_bytes(b"n")
    lib = PolicyLibrary(lib_dir, allowed_roots=[src], card_command=[])
    lib.import_policy(src, src / "m.zip", "p_one")
    assert (lib_dir / "p_one").stat().st_mode & 0o777 == 0o755


def test_import_uses_the_run_command_template_and_flags_missing_robot_config(tmp_path):
    src, lib_dir = tmp_path / "src", tmp_path / "policies"
    src.mkdir()
    (src / "m.zip").write_bytes(b"m")
    (src / "vec_normalize.pkl").write_bytes(b"n")
    run = "python3 /ws/scraping/run_policy.py --model {model} --normalization {normalization} " \
          "--robot-config {robot_config} --robot-controller servo_admittance"
    lib = PolicyLibrary(lib_dir, allowed_roots=[src], card_command=[], run_command=run)
    out = lib.import_policy(src, src / "m.zip", "p_one")
    assert "no robot config" in out["msg"]
    m = lib.manifests["p_one"]
    assert m.command[:2] == ["python3", "/ws/scraping/run_policy.py"]
    assert any("needs {robot_config}" in p for p in m.contract_problems())
    (src / "robot_config.yaml").write_text("observations: {}\n")
    lib.import_policy(src, src / "m.zip", "p_one", overwrite=True)
    m = lib.manifests["p_one"]
    assert m.contract_problems() == []
    cmd = m.build_command()
    assert cmd[cmd.index("--robot-config") + 1].endswith("p_one/robot_config.yaml")
