# OARBots

This repository is for the Omnidirectional Assistive Robots (OARBots), built, developed, and maintained by Rensselaer Polytechnic Institute (RPI) using [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html). For an overview, installation instructions, and more, visit [https://rpirobotics.github.io/oarbots/](https://rpirobotics.github.io/oarbots/).

The ROS1-to-Jazzy audit, migration changes, tested commands, and hardware acceptance requirements are in [docs/platform-audit.md](docs/platform-audit.md). This branch requires the exact revisions in `packages.repos` **and** the patches in `patches/`:

```bash
vcs import src < packages.repos
python3 scripts/apply_dependency_patches.py --workspace-src src
```

After building and sourcing the workspace, run `bash scripts/check_platform.sh` for isolated software checks. Read the audit before deploying: camera extrinsics, serial identities, body selection, and physical stopping/calibration tests remain installation-specific.

## Fabric perception

See [the perception pipeline](docs/fabric-perception.md) for tagged-glove tracking,
per-camera fabric observations, configuration, build and replay instructions.
