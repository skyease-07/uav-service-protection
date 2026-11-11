# Reproduction scope

## Scientific versions

The source archive was selected against the manuscript's Step29 handoff (8 November 2026). Scientific evidence remains in three original research studies. Original Python and protocol bytes are retained; `scripts/run_example.py` is a new portable adapter and is not an original archived formal runner.

Run `python scripts/verify_sources.py` to verify all 51 copied source/configuration files. This checks equality to the packaging manifest, not an external timestamp or preregistration.

## Manuscript scenario design

Seed ranges below are half-open, as in Python `range(start, stop)`.

| Network | Flights | Request window (s) | Seeds | Scenarios |
|---|---:|---:|---|---:|
| single_merge | 100 | 60 | [20000, 20080) | 80 |
| layered20 | 200 | 120 | [21000, 21020) | 20 |
| layered40 | 200 | 120 | [22000, 22020) | 20 |

Main study: SPC, SPC-UR, SPC-WSR, WSR-S on all 120 scenarios. LNS-I/M use only single_merge seeds [20000, 20030), with random seed offset 1030000, neighborhood 8, 1000 attempts, and no wall-clock termination (early stop at zero objective remains possible): 540 method records in total. See `Drones_Revision_Stage7/confirmation_protocol.json`.

Construction follow-up: two construction objectives × SPC/UR/WSR × 120 scenarios = 720 records. Service follow-up: SPC/WSR/F/T/D/ALL × 120 scenarios = 720 records. Their `protocol.json` files specify the descriptive analyses and diagnostics. Both follow-ups reuse inspected scenarios; they are not new holdouts.

The portable runner accepts each scenario's network, n, seed and window. It uses the original solvers and single-thread limits, but writes a simplified output layout rather than the archived `run.json` hierarchy. It does not implement the formal method-order rotation or full experiment-lock/parity workflow. Runtime from this entry point should not be substituted for manuscript timings.

## Tests and limits

`scripts/run_tests.py` invokes the existing tests through unittest in separate processes, avoiding module-name collisions between study copies and avoiding their historical report-writing entry points. Tests cover solver parity, reservation bounds, service guards, statistical calculations, independently reconstructed geometry, construction trade-offs and service event replay.

The quick-start records resource and speed checks. Independent nominal continuous-interval geometry code and tests are included, but the quick-start does not perform the complete formal geometry audit. It also does not regenerate bootstrap summaries or paper figures. Archived analysis/audit helper modules are retained where required by tests; their command-line main functions expect the original full result hierarchy and should not be run on the simplified example outputs.

Release v1.0.0 includes the archived aggregate estimates, scenario-method endpoints and per-flight metrics in `results/`. The result verifier checks file hashes, design coverage, weighted means, flight totals and primary paired effects. Regenerating every bootstrap, event-level diagnostic and figure additionally requires the formal raw records, original execution locks, event logs and plotting/replay pipeline; those large archives remain outside this release.

## Source selection

Scientific source modules were selected from the three named studies along with their local import dependencies and protocol files. Their `frozen/uav_scheduler_delay_optimized.py` dependencies are included. Manuscript compilation, browser/download helpers, local environment dumps, one-time freezing scripts and archive packaging scripts are excluded. No scientific algorithm or parameter was changed in the copied files.

Original files may describe historical workflows and adjacent archives. The top-level README and `scripts/` commands are the supported portable interface for this release.
