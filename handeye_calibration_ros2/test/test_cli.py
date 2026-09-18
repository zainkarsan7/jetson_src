import json

from handeye_calibration_ros2.cli import main
from handeye_calibration_ros2.storage import save_session
from test_calibration import dataset
from test_storage import CONTEXT


def test_offline_target_solve_validate_workflow(tmp_path, capsys):
    main(['target', '--output', str(tmp_path/'target.png')])
    metadata = json.loads(capsys.readouterr().out)
    assert metadata['board']['kind'] == 'charuco'
    assert (tmp_path/'target.png').stat().st_size > 1000
    samples, _, _ = dataset(noise=True)
    save_session(tmp_path/'train.yaml', CONTEXT, samples[:18])
    save_session(tmp_path/'heldout.yaml', CONTEXT, samples[18:])
    main(['solve', str(tmp_path/'train.yaml'), '--output-directory', str(tmp_path/'result')])
    result = json.loads(capsys.readouterr().out)
    assert result['quality_passed']
    main(['validate', str(tmp_path/'result/calibration.yaml'), str(tmp_path/'heldout.yaml')])
    validation = json.loads(capsys.readouterr().out)
    assert validation['translation_rms_m'] < 0.003
