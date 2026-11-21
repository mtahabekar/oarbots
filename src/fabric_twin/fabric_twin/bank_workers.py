"""Independent, spawned Style3D models for the bank's candidate simulations.

Never fork a CUDA context or combine candidates in one model: Style3D's global
PCG couples batched worlds. Cases are copied once per worker, not per candidate.
"""
import multiprocessing as mp
import signal
import subprocess
import warnings


def worker_count(requested, device, per_worker_gb=6., reserve_gb=4.):
    if isinstance(requested, bool) or int(requested) != requested or requested < 1:
        raise ValueError('workers must be a positive integer')
    if requested == 1:
        return 1
    try:
        with open('/proc/meminfo') as stream:
            available = next(int(line.split()[1]) for line in stream if line.startswith('MemAvailable:')) / 1e6
        count = min(int(requested), max(1, int((available-reserve_gb)//per_worker_gb)))
    except (OSError, StopIteration):
        count = 1  # Do not guess how many multi-GB CUDA processes will fit.
    if count > 1 and str(device).startswith('cuda'):
        try:
            control = subprocess.run(['nvidia-cuda-mps-control'], input='get_server_list\n',
                                     text=True, capture_output=True, timeout=2., check=False)
            if control.returncode:
                count = 1
        except (OSError, subprocess.TimeoutExpired):
            count = 1
    if count < requested:
        warnings.warn(f'Bank requested {requested} workers; using {count}. Each needs about '
                      '6 GB available host memory, with 4 GB reserved; CUDA parallelism also '
                      'requires running MPS (fabric_run.sh mps start).', RuntimeWarning)
    return count


_worker = {}


def _initialize(spec, grips, cases):
    # Parent handles Ctrl+C and tears down the pool. Build lazily so a CUDA/model
    # construction error is returned as a task exception, not an initializer respawn loop.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    _worker.update(spec=spec, grips=grips, cases=cases, bank=None)


def _cost(scales):
    from .bank import Bank
    if _worker['bank'] is None:
        _worker['bank'] = Bank(_worker['spec'], _worker['grips'])
    return _worker['bank'].cost(scales, _worker['cases'])


class CandidatePool:
    def __init__(self, workers, spec, grips, cases):
        self.pool = mp.get_context('spawn').Pool(workers, initializer=_initialize,
                                                 initargs=(spec, grips, cases))

    def __enter__(self):
        return self

    def costs(self, candidates):
        # map preserves input order (including ties), independently of completion order.
        # Pool otherwise silently respawns a worker killed by CUDA/OOM and leaves
        # its lost task waiting forever. Watch the original processes while waiting.
        processes = tuple(self.pool._pool)
        pending = self.pool.map_async(_cost, candidates, chunksize=1)
        while True:
            try:
                return pending.get(timeout=.2)
            except mp.TimeoutError:
                if any(process.exitcode is not None for process in processes):
                    raise RuntimeError('Bank worker exited during a simulation; no result released. '
                                       'Check memory/CUDA logs and restart MPS before retrying.')

    def __exit__(self, exc_type, exc, traceback):
        if exc_type is None:
            self.pool.close()
        else:
            self.pool.terminate()
        self.pool.join()
