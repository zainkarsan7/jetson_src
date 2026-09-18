"""Offline target creation, calibration and held-out validation (no ROS needed)."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from .calibration import METHODS, evaluate, solve
from .storage import complete_result, export_result, load_session, read_yaml, write_yaml
from .target import BoardSpec, Target


def board_from_config(path):
    with Path(path).expanduser().open(encoding='utf-8') as f:
        config = yaml.safe_load(f)
    if 'handeye' in config:
        params = config['handeye']['ros__parameters']
        config = params.get('board', {k[6:]: v for k, v in params.items() if k.startswith('board.')})
    return BoardSpec(**config)


def generate_target(path, spec):
    path = Path(path).expanduser()
    if path.suffix.lower() != '.png':
        raise ValueError('Save targets as lossless .png files')
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), Target(spec).render()):
        raise OSError('Could not write target image')
    w, h = spec.dimensions()
    metadata = {'schema_version': 1, 'board': spec.to_dict(), 'opencv_version': cv2.__version__,
                'pattern': 'legacy_pre_4_6', 'board_width_m': w, 'board_height_m': h,
                'note': 'Print flat with preserved aspect ratio; measure actual square/marker size after printing.'}
    write_yaml(path.with_suffix('.yaml'), metadata)
    return metadata


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    target = sub.add_parser('target', help='Generate a calibration board PNG plus geometry sidecar')
    target.add_argument('--config', help='Board YAML or installed-style handeye node configuration')
    target.add_argument('--output', required=True)
    solver = sub.add_parser('solve', help='Solve saved session, export YAML and preview launch')
    solver.add_argument('session')
    solver.add_argument('--method', choices=list(METHODS), default='park')
    solver.add_argument('--output-directory', required=True)
    solver.add_argument('--max-translation-rms-m', type=float, default=0.01)
    solver.add_argument('--max-rotation-rms-deg', type=float, default=2.0)
    validator = sub.add_parser('validate', help='Validate separate samples against a solved calibration')
    validator.add_argument('calibration')
    validator.add_argument('session')
    args = parser.parse_args(argv)
    try:
        if args.command == 'target':
            spec = board_from_config(args.config) if args.config else BoardSpec()
            result = generate_target(args.output, spec)
        elif args.command == 'solve':
            context, samples = load_session(args.session)
            result = complete_result(solve(samples, context['mode'], args.method), context)
            result['quality_passed'] = (result['residuals']['translation_rms_m'] <= args.max_translation_rms_m and
                                        result['residuals']['rotation_rms_deg'] <= args.max_rotation_rms_deg)
            if not result['quality_passed']:
                raise ValueError('Residual thresholds failed: '+json.dumps(result['residuals']))
            export_result(args.output_directory, result)
        else:
            result_file = read_yaml(args.calibration)
            context, samples = load_session(args.session)
            if context != result_file['context']:
                raise ValueError('Validation session context differs from calibration context')
            training_reference = result_file['residuals']['closure_reference']
            result = evaluate(samples, np.asarray(result_file['parent_from_camera']), result_file['mode'], training_reference)
            result['note'] = 'Held-out closure residuals assume the target mount stayed fixed between sessions.'
        print(json.dumps(result, indent=2, allow_nan=False))
    except (ValueError, KeyError, TypeError, OSError, cv2.error) as exc:
        parser.exit(2, f'Error: {exc}\n')


if __name__ == '__main__':
    main()
