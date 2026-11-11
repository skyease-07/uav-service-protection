# UAV Service Protection

Core research code for **Conditional Recovery under Individual Entry Commitments in UAV Route Networks: Construction Dependence and Service Trade-offs**.

This repository accompanies the manuscript and contains the scheduling model, SPC construction, uniform/weighted safe reinsertion (UR/WSR), protected LNS baselines, construction-objective comparison, and separate service-protection rules. See [中文说明](README.zh-CN.md).

## Install

Python 3.11 or newer is recommended. Validation of this release uses Python 3.13 and the pinned numerical dependencies.

```bash
python -m venv .venv
# Windows: .venv\Scripts\activate
# macOS/Linux: source .venv/bin/activate
python -m pip install -r requirements.txt
python scripts/verify_sources.py
python scripts/run_tests.py
```

Run commands from the repository root. No private dataset, absolute local path, GitHub credential, or prior research directory is required for these commands. Matplotlib is required at import time by the preserved scheduler, even when no plots are generated, and is included in the dependencies.

## Quick start

Each command creates a new output directory and refuses to overwrite an existing one.

```bash
python scripts/run_example.py --study main --output outputs/main
python scripts/run_example.py --study construction --output outputs/construction
python scripts/run_example.py --study service --output outputs/service
# Optional LNS-I and LNS-M baselines (1000 attempts, unless the objective reaches zero):
python scripts/run_example.py --study main --include-lns --output outputs/main-lns
```

Defaults are a 12-flight development scenario, seed 100, a 60-second request window and the single-merge network. These are smoke examples, not the manuscript's aggregate estimates. Outputs include generated inputs, schedules, accepted updates, weighted ground delay, service metrics, source-model signatures, environment information and resource/speed audit results. Service examples additionally save compressed event logs. Disabled infinite LNS wall-clock limits are serialized as JSON `null`, as in the original formal runner.

Example with the first manuscript main-study scenario:

```bash
python scripts/run_example.py --study main --n 100 --seed 20000 --window 60 --include-lns --output outputs/paper-main-seed20000
```

The alternative networks are `layered20` and `layered40`. See [reproduction scope](docs/REPRODUCTION.md) for the complete scenario ranges and evidence boundaries.

## Published results and version

[Release v1.0.0](https://github.com/skyease-07/uav-service-protection/releases/tag/v1.0.0) packages the code and complete summary results. Browse [results/](results/README.md) for all 1,980 scenario-method endpoints, 262,000 flight-metric rows, archived estimates and confidence intervals, audits, development summaries and manuscript table snapshots. Read [field definitions and paper mapping](docs/RESULTS_GUIDE.md), then run `python scripts/verify_results.py` to check the published data. The full raw event/replay archive is outside this release.

## Code map

Original study folder names are retained to preserve protocol-relative imports. The scientific files are copied byte-for-byte; `provenance/source_sha256.json` records their hashes. `.gitattributes` disables line-ending conversion so these hashes remain valid after a Git clone on Windows or Linux.

| Folder / file | Purpose |
|---|---|
| `Drones_Revision_Stage7/code/final_model.py` | Unified physical model and deterministic scenario generation |
| `Drones_Revision_Stage7/code/indexed_scheduler.py` | Reservation interval index |
| `Drones_Revision_Stage7/code/confirmation_solver.py` | SPC, SPC-UR, SPC-WSR and joint service protection |
| `Drones_Revision_Stage7/code/protected_lns.py` | LNS-I / LNS-M with initial / monotone individual delay bounds |
| `Drones_Revision_Stage7/code/service_metrics.py` | Completion time, airborne time, horizontal and vertical distance |
| `Drones_Revision_Stage7/code/independent_geometry.py` | Independent nominal interval geometry audit implementation |
| `Drones_Construction_Objective_Control/code/objective_solver.py` | Composite-score vs delay-first construction |
| `Drones_Service_Protection_Split/code/split_solver.py` | WSR branches F, T, D, ALL and event diagnostics |
| `scripts/` | Portable release entry points and verification |

The `frozen/` scheduler is a dependency of the unified model; use the release entry points to run the manuscript model. Historical standalone defaults in that dependency are not the final manuscript configuration.

## Interpretation

SPC-UR and SPC-WSR use the same SPC start and candidate domain. Each accepted reinsertion protects current individual ground delay. F additionally protects completion time; T airborne time; D horizontal and vertical distance separately; ALL protects all four service metrics and corresponds to WSR-S. Construction and service-split follow-ups reuse previously inspected scenarios and are post-hoc analyses.

This is a **core-code and complete summary-results release**, with the precise data scope documented in `results/README.md`. It does not include the full raw research archive, manuscript drafts, downloaded literature, NetLogo assets or full formal event logs, and does not regenerate every manuscript figure. Resource and speed checks in the quick-start are not a continuous-time safety certificate. Service quantities are proxies, not calibrated energy or money.

## Attribution and reuse

Repository maintained by [skyease-07](https://github.com/skyease-07). The manuscript is not assigned a fabricated DOI or publication status here. Cite the manuscript title and the exact repository commit used. No software license has yet been selected by the authors; public availability alone does not grant an open-source license.
