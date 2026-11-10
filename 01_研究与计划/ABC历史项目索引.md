# 项目证据索引

2026-10-26。统一分类不改变原批次合同，三组工作均在本私有仓库。

## A · OTTER 仿真学习与基线

目的为MATLAB／MSS Otter学习、海流与控制核对、静态参考与跟踪探索。42保存闭环＋4执行前拒绝，见[编号映射](../../EXPERIMENT_IDS.md)、[静态清单](../../site/static_manifest.json)、[历史回放](../../site/archive.html)。不同控制政策和参考转换不作同条件排名。

32组汇报、36组论文表格为历史写作子集，不增加次数；完整来源见[本地盘点](audit_local.md)。[Notion A](https://app.notion.com/p/404e6b921cf08193a0b4fc49eb53209f)按讲解与数据分层。

## B · 动态选法与安全监督

| 子课题 | 权威入口 | 解释范围 |
|---|---|---|
| 船模、速度环、参考接口 | [P0验收](../P0_ACCEPTANCE.md) | 工程资格，不替代任务成功 |
| 监督、桥接、存档恢复 | [P1-A](../../experiments/p1a_bridge_v1/README.md) | 26入口含重叠恢复后缀，非26独立世界 |
| 动态选法 | [P1-B](../../experiments/p1b_dynamic_selection_v1/README.md) | 20开发、60留出分列；留出安全36/60 |
| 末端判据 | [独立旁支](../../experiments/p1b_terminal_judge_v1/README.md) | 开发4／留出12；真实计算成本保留 |
| 监督ON/OFF | [3对6航程报告](../../experiments/safety_supervision_ablation_v1/reports/final_report.md) | 所选病例ON1/3、OFF3/3，非普遍排名 |
| 同状态与公开输入 | [研究入口](../research_20261022/README.md)、[Notion专题](https://app.notion.com/p/404e6b921cf0813786f8ed8f56152147) | 6短窗、9短窗、seed102覆盖、首次公开他船、输入标签导出分别解释 |
| 完整任务先导失败 | [已停止记录](../../experiments/p1c_full_terminal_pilot_v1/reports/native_attempt1_failure_report_v01.md) | 111.30s前缀、0后缀；日志摘要错误，修复草案未完成运行资格 |

[182记录回放](../../site/nightly.html)含4首步前失败、资格和恢复片段，不是182独立任务。Notion原181行已按原失败收据补登缺失JSON启动失败；时长／净距留空，不伪填0。原55行历史表中的9个计划占位不视为已运行，后来的实际九组避障仍在独立L批。

[研究报告DOCX](../research_20261021/ASV_Dynamic_Planner_Research_Report_20261021.docx)、[材料档案](ARCHIVE.md)区分原理、结果与方案；历史文档中的执行建议仅作原文参考。

## C · 路线比较与失败诊断

| 批次 | 权威入口 | 独立范围 |
|---|---|---|
| 原120完整任务 | [冻结协议](../../experiments/q1_route_geometry_pilot_v1/protocol.json)、[汇总](../../experiments/q1_route_geometry_pilot_v1/reports/summary.json)、[回放](../../site/q1_route_geometry_pilot_v1/index.html) | 60对／5开发种子／2图／4确定性路线，统一局部与执行栈 |
| 主58机制诊断 | [干预汇总](../../experiments/q1_failure_diagnosis_v1/reports/intervention_summary.json)、[栈审计](../../experiments/q1_failure_diagnosis_v1/reports/stack_audit.md) | 18负控＋40机制任务；事后选定5世界 |
| VO交互补充10 | [独立批次](../../experiments/q1_stop_veto_interaction_v1)、[回放](../../site/q1_failure_diagnosis_v1/index.html) | 新增10；四格复用主批30，不能重复加总 |
| 模型资格 | [保持指令资格](../../experiments/q1_failure_diagnosis_v1/qualification/prediction_interface_v1) | 6工程job，非完整任务；模型与几何评价数量按原报告 |

原120计数51安全／20到达裕度不足／21碰撞／28超时；A*22/60、Theta*29/60安全，仅19对双方安全进入效率子集。诊断网页80目录＝68新航程＋12原病例；参考交付合同与原120不同，不合并成功率。Q2／Q3和RL仍为方案。

[Notion C](https://app.notion.com/p/404e6b921cf0811489acebe84a7d01d5)按原120、主诊断、交互补充、模型资格及数据分层。
