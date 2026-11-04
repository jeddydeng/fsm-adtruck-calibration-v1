# 生产提取与验收记录

提取时间：2026-09-04（Asia/Shanghai）

来源为车载工控机当时正在运行的正式版本；未复制旧备份、tests、benchmark、mock、日志、缓存、抓拍/录像或 `voice-env`。正式程序被整理为路径无关结构，服务和桌面模板在安装时按当前用户和克隆目录渲染。

提取时在线检查：

- 工控机：Ubuntu 24.04.4 LTS，x86_64，车内网 `192.168.1.102`。
- 树莓派：`192.168.1.186`，四相机 online，fresh mask 15，BEV 约 10 Hz，CAN online。
- 交通灯相机：320×240，约 5 Hz，标定已加载，UDP 租约 active。
- 当前旧服务保持 active，提取/整理过程没有重启或替换生产服务。
- 两套 ONNX 模型、配置和测试 MP3 的 SHA-256 全部通过。
- 使用提取后的 Huayan 模型和正式 `piper_worker.py` 成功生成中文 WAV。
- 全部 shell 脚本通过 `bash -n`，全部 Python 文件通过 AST 解析，正式模块导入成功，网页状态接口仍返回 connected/BEV/四相机。

由于不能中断正在使用的车载系统，本次没有在该工控机上执行会替换旧服务的 `install.sh --replace-legacy`。全新机器仍须按 README 执行 `install.sh` 和完整 `verify.sh`；只有 `verify.sh` 零失败才算部署验收完成。

