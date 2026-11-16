"""Run the offline studies (S2 identification, S3 twin replays) as parallel GPU jobs.

    python run_offline.py s2 [--workers N]      identification fits and cost landscapes
    python run_offline.py s3 [--workers N]      twin replays (needs the S2 live-input fits)

Each job is a separate process (`id_fit.py` or `twin_replay.py`) writing one result file under
results/; finished jobs are skipped, so the command can be re-run after an interruption.
"""
import argparse
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
RESULTS = HERE.parent/'results'
DATA = Path(os.environ.get('SIM_EVAL_DATA', '/media/taha/ExtermeLinux/oarbot_simulation/logs/sim_eval'))
PY = os.environ.get('SIM_EVAL_PYTHON', str(Path.home()/'venvs/fabric_ros/bin/python'))
LADDER = {  # (state, grips, points); each perceived input alone, then all together
    'L0': ('truth', 'truth', 'truth'), 'Dp': ('truth', 'truth', 'depth'), 'Gm': ('truth', 'measured', 'truth'),
    'St': ('twin', 'truth', 'truth'), 'L3': ('twin', 'measured', 'depth')}


def datasets():
    return sorted(p for p in (DATA/'id').glob('id_*') if (p/'manifest.json').exists()
                  and json.loads((p/'manifest.json').read_text()).get('exit_status') == 0)


def s2_jobs():
    jobs = []
    for run in datasets():
        # per-input attribution and cost landscapes: repeat 1 of the groups with a material mismatch
        first = run.name in ('id_matched_B_r1', 'id_matched_C_r1', 'id_fine_B_r1')
        for level in ('L0', 'L3'):
            for holds in ('extended', 'baseline'):
                jobs.append((RESULTS/'s2'/f'{run.name}.{level}.{holds}.json',
                             ['id_fit.py', str(run), None, '--inputs', *LADDER[level], '--holds', holds]))
        for level in ('L0', 'L3'):  # S2c online adaptation from nominal coefficients
            jobs.append((RESULTS/'s2'/f'{run.name}.{level}.adapt.json',
                         ['adapt_trace.py', str(run), None, '--inputs', *LADDER[level]]))
        if first:
            for level in ('Dp', 'Gm', 'St'):
                jobs.append((RESULTS/'s2'/f'{run.name}.{level}.extended.json',
                             ['id_fit.py', str(run), None, '--inputs', *LADDER[level], '--holds', 'extended']))
            for level in ('L0', 'L3'):
                jobs.append((RESULTS/'s2'/f'{run.name}.{level}.extended.landscape.json',
                             ['id_fit.py', str(run), None, '--inputs', *LADDER[level], '--holds', 'extended', '--landscape-only']))
    return jobs


def fitted(run, level='L3'):
    path = RESULTS/'s2'/f'{run.name}.{level}.extended.json'
    if not path.exists():
        return None
    fit = json.loads(path.read_text())['fit']
    return fit['bend'], fit['shear']


def s3_jobs():
    jobs = []
    for run in datasets():
        manifest = json.loads((run/'manifest.json').read_text())
        true = manifest['true_scales']['bend'], manifest['true_scales']['shear']
        fit = fitted(run)
        conditions = {
            'off_nominal': ('off', (1., 1.), 'measured'),
            'off_true': ('off', true, 'measured'),
            'est_nominal': ('estimator', (1., 1.), 'measured'),
            'oracle_nominal': ('oracle', (1., 1.), 'measured'),
            'off_nominal_truthgrips': ('off', (1., 1.), 'truth'),
            'est_nominal_truthgrips': ('estimator', (1., 1.), 'truth'),
        }
        if fitted(run, 'L0') is not None:
            conditions['off_l0fit'] = ('off', fitted(run, 'L0'), 'measured')
        if fit is not None:
            conditions['off_fitted'] = ('off', fit, 'measured')
            conditions['est_fitted'] = ('estimator', fit, 'measured')
        for label, (correction, scales, grips) in conditions.items():
            jobs.append((RESULTS/'s3'/f'{run.name}.{label}.npz',
                         ['twin_replay.py', str(run), None, '--correction', correction, '--scales', *map(str, scales),
                          '--grips', grips, '--label', label]))
        if run.name.endswith('_r1'):
            for k in (.25, 4.):
                jobs.append((RESULTS/'s3'/f'{run.name}.est_nominal_gain{k:g}.npz',
                             ['twin_replay.py', str(run), None, '--correction', 'estimator', '--scales', '1', '1',
                              '--feedback-scale', str(k), '--label', f'est_nominal_gain{k:g}']))
    return jobs


def execute(job):
    output, command = job
    if output.exists():
        return f'skip {output.name}'
    output.parent.mkdir(parents=True, exist_ok=True)
    command = [PY, str(HERE/command[0]), command[1], str(output), *command[3:]]
    started = time.time()
    log = output.with_suffix(output.suffix+'.log')
    with open(log, 'w') as f:
        code = subprocess.call(command, cwd=HERE, stdout=f, stderr=subprocess.STDOUT)
    tail = [line for line in log.read_text().splitlines() if line.strip() and not line.startswith(('Warp', '   ', 'Module'))]
    return f'{"ok" if code == 0 and output.exists() else f"FAILED({code})"} {output.name} {time.time()-started:.0f}s {tail[-1][:200] if tail else ""}'


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('study', choices=('s2', 's3'))
    parser.add_argument('--workers', type=int, default=3)
    parser.add_argument('--filter', default='')
    options = parser.parse_args()
    jobs = s2_jobs() if options.study == 's2' else s3_jobs()
    jobs = [j for j in jobs if options.filter in j[0].name]
    # longest first: landscapes and extended fits, then the rest
    jobs.sort(key=lambda j: ('landscape' not in j[0].name, 'extended' not in j[0].name, j[0].name))
    print(f'{len(jobs)} jobs, {options.workers} workers', flush=True)
    with ThreadPoolExecutor(options.workers) as pool:
        for line in pool.map(execute, jobs):
            print(time.strftime('%H:%M:%S'), line, flush=True)
    print('OFFLINE_DONE', options.study, flush=True)


if __name__ == '__main__':
    sys.exit(main())
