# 无人机航路网络：个体承诺下的延误削减与服务保护

本仓库整理论文 **Conditional Recovery under Individual Entry Commitments in UAV Route Networks: Construction Dependence and Service Trade-offs** 所用的核心代码。

包含三组：主实验（SPC、UR、WSR、WSR-S 与 LNS-I/M）、构造目标对照（综合评分/延误优先）、服务保护拆分（F/T/D/ALL）。51 个科研源码及配置文件保持原始字节，来源哈希见 `provenance/source_sha256.json`。新增 `scripts/` 提供不依赖本机旧档案的运行入口。

## 安装与运行

建议 Python 3.11 及以上；本次本地验证使用 Python 3.13。在仓库根目录执行：

```bash
python -m pip install -r requirements.txt
python scripts/verify_sources.py
python scripts/run_tests.py
python scripts/run_example.py --study main --output outputs/main
python scripts/run_example.py --study construction --output outputs/construction
python scripts/run_example.py --study service --output outputs/service
```

需要 LNS 基线时，在主实验命令中添加 `--include-lns`。输出目录必须是新目录，以免覆盖已有结果。建议先建立虚拟环境，方法见英文 README。

默认运行 12 架次、种子 100 的开发示例，不是论文完整实验。每次生成输入、方案、接受更新、服务指标、资源和速度审计、运行环境与结果 JSON；服务拆分另保存压缩事件日志。完整场景范围及复现限制见 [REPRODUCTION.md](docs/REPRODUCTION.md)。

## 科学含义与范围

UR/WSR 从相同 SPC 起点出发；重插入始终保护当前个体地面延误。F 保护完成时刻，T 保护空中时长，D 分别保护水平和垂直距离，ALL 联合保护四项，对应 WSR-S。构造对照及服务拆分使用已查看场景，属于事后分析。

正式版本 [v1.0.0](https://github.com/skyease-07/uav-service-protection/releases/tag/v1.0.0) 发布核心算法及完整汇总结果：1,980 条逐场景方法指标、262,000 条逐航班指标、完整统计 JSON、审计结果、开发阶段汇总和论文表格快照。入口见 [results/](results/README.md)，字段及表图对应见 [RESULTS_GUIDE.md](docs/RESULTS_GUIDE.md)。运行 `python scripts/verify_results.py` 可验证数据。

完整原始事件日志、全部方案轨迹及 NetLogo 回放资产未纳入本次发布，不能将本仓库描述为全部原始数据已公开。服务时间/距离指标不等同能耗或经济成本，示例审计不构成飞行安全认证。

维护账号：`skyease-07`。作者尚未指定软件许可证，暂未代选 MIT/GPL 等授权。引用时请使用论文题目和实际代码提交号。
