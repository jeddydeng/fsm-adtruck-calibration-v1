# Mac / 工控机隔离备份验证

状态：完成。所有工控机文件位于 `white_ws/calibration/offline_*`，不在 Autoware
参数目录中；没有启动驱动、Autoware 或 CAN。

| 内容 | Mac | 工控机 | SHA-256 |
|---|---|---|---:|
| C200 BEV 复核候选 | `result/fisheye_bev_20260903/review_only_c200` | `~/white_ws/calibration/offline_results/fisheye_bev_20260903/review_only_c200` | 40/40 |
| 严格 14 组检测证据 | `result/fisheye_bev_20260903/selected14_analysis` | `~/white_ws/calibration/offline_results/fisheye_bev_20260903/selected14_analysis` | 6/6 |
| 可重复程序 | `process/fisheye_bev` | `~/white_ws/calibration/offline_tools/fisheye_bev` | 6/6 |
| README、补采计划与本验证记录 | `result/fisheye_bev_20260903` | `~/white_ws/calibration/offline_results/fisheye_bev_20260903` | 3/3 |

合计双端逐文件验证 55/55。工控机只同步检测索引、原始图 SHA、程序、最终预览
和候选矩阵；56 张原始鱼眼 PNG 本来就在工控机采集目录中，没有重复占用空间。
Mac 保存完整检测预览。候选文件明确包含 `deployment_allowed: 0`。
