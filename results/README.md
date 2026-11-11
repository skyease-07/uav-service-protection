# Published results / 公开结果

This directory publishes the complete archived statistical summaries for the three manuscript studies, all 1,980 scenario-method endpoints and 262,000 per-flight service-metric rows. Development and sensitivity summaries used in the supplementary material are included separately. No experiments were rerun or estimates changed to assemble this release.

| Directory | Method records | Scenario set | Flight-metric rows | Contents |
|---|---:|---|---:|---|
| `main/` | 540 | 120 scenarios; LNS subset of 30 | 70,000 | Main inference, transfer, service and LNS summaries |
| `construction/` | 720 | Same 120 scenarios, two construction objectives | 96,000 | Composite vs delay-first follow-up |
| `service/` | 720 | Same 120 scenarios, six methods | 96,000 | F/T/D/ALL follow-up and mechanism summaries |
| `development/` | — | Earlier development sets, kept separate | — | Model, baseline, protection, capacity, geometry and replay summaries |
| `paper_tables/` | — | Step29 manuscript source snapshot | — | Original table fragments and label index |

The three studies reuse scenarios: these counts are not 360 independent scenarios. Flights, visits, candidates and method runs are not independent statistical samples. Follow-up analyses are post-hoc.

Each main study directory contains `analysis.json` (full original estimates, confidence intervals, paired values and statistics), `scenario_metrics.csv` (one method run per row), `service_flights.csv` (one flight-method per row) and `audit.json` (archived audit results). JSON files and flight CSVs are copied byte-for-byte. Scenario CSVs are derived from archived `run.json` files, with source-relative paths and SHA-256 hashes for traceability; those paths refer to the research archive, not downloadable files in this repository.

Run `python scripts/verify_results.py` from the repository root to check original-file hashes, exact prescribed method/seed coverage, all study cell weighted-delay means, per-flight weighted-delay totals and the H1/H2 paired effects. This does not rerun every bootstrap or reconstruct event logs. `validation_report.json` records the local packaging checks. `source_manifest.json` maps copied files to their original paths and hashes.

See [field definitions and paper mapping](../docs/RESULTS_GUIDE.md). Archived table fragments may require manuscript macros; they are evidence snapshots, not standalone TeX documents. `paper_tables/index.csv` identifies stable LaTeX labels, because table numbers change between manuscript revisions. A generated fragment can be retained but inactive in the final manuscript; the standalone `sections/confirmation_lns_table.tex` is the revised LNS table text.

**Scope:** this is the complete summary-results release described above, not the entire raw research archive. Full per-run schedules/accepted-update traces, compressed candidate-event logs, input replicas, old source locks, and NetLogo/figure rendering assets are not included. Original audit files reference such archival paths/hashes. Code plus these published results supports inspection of reported estimates; it is not a one-command reproduction of every figure or a flight-safety certificate.

中文：本目录公开三组实验的完整汇总统计、1,980 条逐场景方法指标及 262,000 条逐航班服务指标，并附开发阶段汇总和论文表格快照。原始 JSON/航班 CSV 保持字节一致；新增逐场景表可追溯到原运行文件及其哈希。三组使用重叠场景，不能把方法记录数或航班数当作独立样本数。大型原始事件日志、完整方案轨迹及回放资产未纳入本次发布，不能表述为“全部原始数据已公开”。
