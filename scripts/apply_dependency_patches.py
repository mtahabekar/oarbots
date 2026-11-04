#!/usr/bin/env python3
"""Apply reviewed fixes only to the exact revisions in packages.repos.

No fetching, reset, checkout, or replacement of local work is performed.
"""
import argparse
import json
from pathlib import Path
import subprocess


def git(repo, *args):
    return subprocess.run(['git', '-C', str(repo), *args], text=True, capture_output=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace-src', type=Path, required=True)
    parser.add_argument('--check', action='store_true', help='Validate without applying')
    args = parser.parse_args()
    patch_dir = Path(__file__).resolve().parents[1] / 'patches'
    manifest = json.loads((patch_dir / 'manifest.json').read_text())
    pending = []
    for name, info in manifest.items():
        repo = args.workspace_src / name
        head = git(repo, 'rev-parse', 'HEAD')
        if head.returncode or head.stdout.strip() != info['revision']:
            raise SystemExit(f'{repo}: expected revision {info["revision"]}; import packages.repos into a clean workspace')
        patch = str(patch_dir / info['patch'])
        if git(repo, 'apply', '--reverse', '--check', patch).returncode == 0:
            print(f'{name}: patch already applied')
            continue
        check = git(repo, 'apply', '--check', patch)
        if check.returncode:
            raise SystemExit(f'{name}: patch conflicts with local changes; no patches applied\n{check.stderr}')
        pending.append((repo, patch))
    for repo, patch in pending:
        if not args.check:
            result = git(repo, 'apply', patch)
            if result.returncode:
                raise SystemExit(result.stderr)
        print(f'{repo.name}: {"ready to apply" if args.check else "applied"}')


if __name__ == '__main__':
    main()
