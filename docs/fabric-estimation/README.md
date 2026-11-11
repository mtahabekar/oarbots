# Fabric mesh estimation

Tier B is a causal, localized ensemble observer. Tier A is an offline, non-causal weak-constraint smoother. Both use a persistent material mesh and the same sensor observation operators. Human and robot attachments remain at four prescribed corners; human holding is an assumption.

Start with the [software walkthrough](usage.md), then follow the [hardware commissioning guide](hardware.md). Read the [implementation and interfaces](implementation.md) before changing a numerical module. The [test guide](testing.md) gives independent commands and explains what the tests demonstrate.

| Document | Contents |
|---|---|
| [Progress report and technical review](../fabric-state-estimation-progress-report.md) | 10 November source audit, measured simulation limitations, theory, implementation challenges, alternatives and staged next steps |
| [Learning methods report](../fabric-learning-methods-report.md) | Data/labels, architectures, losses, training/tuning, uncertainty, deployment and validation across perception, contact, dynamics, identification and control |
| [Implementation](implementation.md) | Modules, algorithms, units, covariance, observation association, ROS and recording contracts |
| [Usage](usage.md) | Installation, complete synthetic example, replay, offline fitting, output inspection, configuration |
| [Real hardware](hardware.md) | Sheet measurements, corner mapping, camera/TF/TCP/glove calibration, synchronization, force calibration, capture and validation |
| [Testing](testing.md) | Numerical, Newton, ROS, MCAP and regression tests; independent truth evaluation; current limitations |

The research design is in [fabric-state-estimation-research.md](../fabric-state-estimation-research.md). Camera processing and tagged-hand tracking are documented separately in [fabric-perception.md](../fabric-perception.md).

This is an executable first implementation, intended for replay and shadow operation. Neither estimator has demonstrated the research's hardware accuracy or latency targets. Tier A outputs **reference candidates**, with fixed material parameters and no posterior covariance. Calibration/time refinement, material identification, learned completion, self-contact, obstacle contact and simulator assimilation are subsequent modules; they are not silently approximated as completed features.
