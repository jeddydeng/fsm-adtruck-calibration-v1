# 2026-09-03 四鱼眼 BEV 标定交付索引

## 当前结论

四路 1600×1200 equidistant 内参已经通过独立质量验收。`relative_usable/` 是
严格使用最后 14 组 Part 3 数据完成闭环接缝优化和自检的 BEV 交付包；它的
`deployment_allowed: 1` **只授权四鱼眼地面相对拼接**，可用于环视 BEV 显示。

车辆绝对外参是另一层结论：Part 1/2 的车体锚点未通过跨批次验证，尤其后鱼眼
绝对高度存在明显冲突。因此四路 R/t 仍是 `review_only_not_signed`，不得据此更新
Autoware TF。`review_only_c200/` 保留为可追溯的初始地面映射，不是部署结果。

## 数据边界

- `selected14_analysis/`：真正的 BEV 序列，共 14 组、56 张鱼眼原图；这是正式分析。
- `relative_usable/`：通过自检的相对地面拼接、四路 remap、H、预览和完整 SHA。
- `review_only_c200/`：相对优化使用的初始地面映射；绝对车体 R/t 未签发。
- 根目录的 `board_detections.*` 和 `candidate_old_extrinsics/`：早期按 144 组宽时间段
  做的诊断，只用于发现分段问题，不能作为 BEV 结果。
- 原始文件未删除；完整备份仍在 `data/raw_extrinsic_data`。

真正 BEV 序列分为：初始 3 组、第一次四角径向外移后 6 组、第二次外移后 5 组。
对可观测的六次移动，设计值为 x/y 各约 0.45 m，测量分量误差中位 40.3 mm、
最大 43.5 mm。此误差与人工“约 45 cm”移动精度一致，证明内参、板尺寸和
地面投影的尺度没有数量级错误，但它不是毫米级测量基准。

## `relative_usable/` 验收结果

- 四条闭环接缝都跨越至少 7 组独立采集，保留特征数为 195、56、260、113；
- 保留特征 P95 残差分别为 1.837、3.868、2.798、1.170 px；
- 0.75–3.50 m 环带联合覆盖 97.66%，两路及以上重叠覆盖 60.52%；
- 优化只允许平移和旋转，四路尺度严格保持 1.0；
- 14 组拼接预览、四色接缝叠图、映射形状/类型/有限值、H 可逆性和输入哈希均通过。

以 0.015 m/px 换算，5 px 是 75 mm；上述结果是图像接缝指标，不是毫米级绝对外参
声明。高出地面的物体受视差影响，不能用单个地面 H 做完全重合。

## `relative_usable/` 内容

- `front.yaml`、`right.yaml`、`rear.yaml`、`left.yaml`：K、D、可用 BEV H、
  相对修正矩阵和仅供追溯的未签发 R/t；
- `*_bev_map_x.npy`、`*_bev_map_y.npy`：原始 equidistant 图直达 BEV 的推荐映射；
- `*_bev_preview.png`、`four_camera_bev_preview.png`：单路和四路融合预览；
- `strict14_bev_contact_sheet.jpg`：14 组结果的全量人工复核图；
- `overlap_validation_contact_sheet.jpg`：四条相邻接缝的品红/绿色重合叠图；
- `bev_coverage_count.png`：每个像素被几颗相机覆盖；
- `source_capture_groups.txt`、`raw_image_index.txt`、`raw_image_sha256.txt`：输入证据；
- `bev_calibration.json`、`overlap_metrics.csv`、`VALIDATION_SUMMARY.md`：机器和人工报告；
- `manifest.sha256`：交付文件的完整性清单。

## 为什么能签发 BEV，但不能签发绝对 R/t

平放板能可靠约束相机 roll、pitch 和离地高度 z，但不能单独给出车辆平面内的
x、y、yaw。它们需要来自通过交叉验证的车体/主雷达锚点。旧锚点与 Part 3 的
前/右/后/左鱼眼高度差分别约 16/16/223/38 mm，后鱼眼冲突明显；C200 本身
也没有通过 Part 1↔Part 2 验证。

相对 BEV 则可以直接用静止车辆的四路公共地面纹理闭环校正。`relative_usable/`
以 front 为画布规范的参考，联合优化 front↔right↔rear↔left↔front 四条接缝，
并通过覆盖、接缝、尺度、文件完整性和 14 组预览验收。因此可签发 H/remap 用于
地面环视拼接；这不等于把参考 R/t 签发为车辆绝对外参。

## 复采重点

1. 现有 tag16h5 字典只有 30 个 ID，四块 6×5 板无法分配互不重叠的 ID；
   每帧只显示一块板并在采集界面记录物理板名，或改印 tag36h11 不重叠 ID 板；
2. 在车辆坐标系中实测至少四个不共线地面控制点，精度建议 ≤5 mm；
3. 后鱼眼必须在三个径向阶段都清晰看到至少一块板，本批第二次外移后后鱼眼
   未得到有效检测；
4. 每个板位车辆静止 2–3 秒后保存三帧，记录唯一物理板编号和实测 `(x,y,z)`；
5. 新 C 结果只有在独立数据差异 ≤1 cm/1°、且 Part 3 同板跨相机误差通过后，
   才能重新运行 `process/fisheye_bev/run_part3_bev.sh` 生成部署候选。

具体数量和摆放见同目录 `RECAPTURE_PLAN.md`。
