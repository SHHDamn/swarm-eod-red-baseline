"""Compare saved real benchmark runs; no simulator inputs are changed."""
import argparse
import hashlib
import json
from pathlib import Path


def load(paths):
    rows = {}
    for path in paths:
        report = json.loads(path.read_text(encoding='utf-8'))
        if not report.get('competition_mode'):
            raise ValueError(f'Not a competition run: {path}')
        for row in report['rounds']:
            key = (row['map'], row['seed'], row['zones'])
            if key in rows:
                raise ValueError(f'Duplicate scenario: {key}')
            if (row['crashed'] or row['walltime_limit_reached'] or
                    not row['map_submitted'] or row['inventory']):
                raise ValueError(f'Incomplete or failed run: {path}, {key}')
            rows[key] = (row, report, path)
    return rows


def compare(baseline, candidate):
    if not baseline or baseline.keys() != candidate.keys():
        raise ValueError('Scenario/seed/zone sets must match and be nonempty')
    pairs = []
    for key in sorted(baseline):
        old, old_report, old_path = baseline[key]
        new, new_report, new_path = candidate[key]
        for field in ['python', 'engine_commit']:
            if old_report[field] != new_report[field]:
                raise ValueError(f'{key}: different {field}')
        unchanged = ['steps', 'coverage_percent', 'placed', 'inventory', 'destroyed_percent', 'health']
        motion_equal = all(old[f] == new[f] for f in unchanged)
        scoring_equal = all(old['scores'][f] == new['scores'][f] for f in ['score_wall', 'score_time'])
        map_gain = new['scores']['score_map'] - old['scores']['score_map']
        total_gain = new['scores']['round_score'] - old['scores']['round_score']
        pairs.append({'map': key[0], 'seed': key[1], 'zones': key[2],
            'baseline_map': old['scores']['score_map'], 'candidate_map': new['scores']['score_map'],
            'map_gain': map_gain, 'baseline_partial': old['scores']['round_score'],
            'candidate_partial': new['scores']['round_score'], 'partial_gain': total_gain,
            'baseline_penalty': old['scores']['map_breakdown']['penalty'],
            'candidate_penalty': new['scores']['map_breakdown']['penalty'],
            'baseline_credit': old['scores']['map_breakdown']['credit'],
            'candidate_credit': new['scores']['map_breakdown']['credit'],
            'steps': new['steps'], 'motion_equal': motion_equal,
            'wall_and_time_equal': scoring_equal,
            'accepted': motion_equal and scoring_equal and map_gain >= -1e-9 and total_gain >= -1e-9,
            'baseline_report_sha256': hashlib.sha256(old_path.read_bytes()).hexdigest(),
            'candidate_report_sha256': hashlib.sha256(new_path.read_bytes()).hexdigest(),
            'baseline_strategy_sha256': old_report['strategy_sha256'],
            'candidate_strategy_sha256': new_report['strategy_sha256']})
    return pairs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, nargs='+', required=True)
    parser.add_argument('--candidate', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Refusing to overwrite comparison')
    pairs = compare(load(args.baseline), load(args.candidate))
    passed = all(p['accepted'] for p in pairs) and any(p['map_gain'] > 1e-9 for p in pairs)
    report = {'passed': passed, 'pairs': pairs}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return int(not passed)


if __name__ == '__main__':
    raise SystemExit(main())
