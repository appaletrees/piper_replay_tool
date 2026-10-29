Piper 原始采集数据回放工具
==========================

本目录独立于 /home/midea/phoenix-mwa。原训练/推理文件与机器人驱动均不修改。
工具不加载 Phoenix 权重，也不运行 RTC 或模型推理。

启动真机网页：
  cd /home/midea/piper-dataset-replay
  OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  ../phoenix-mwa/.venv-piper-inference/bin/python -u app.py --hardware --port 8766

启动离线演示：
  cd /home/midea/piper-dataset-replay
  OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  ../phoenix-mwa/.venv-piper-inference/bin/python -u app.py --demo --port 8766

浏览器打开 http://127.0.0.1:8766 。同一个端口只运行一个实例；Ctrl+C 停止网页服务。
已复用现有 Python 环境，无需另外安装依赖。迁移时可参考 requirements.txt。
硬件模式还需要机器人控制目录原有的 Piper SDK、scipy、pyrealsense2 和可用 CAN。

页面操作：
1. 选择数据集、原始 episode 编号（从 0 开始）以及 action / state，点击加载。
2. 选择起始帧、回放频率和起点对齐时长。
3. 点击连接设备与相机。连接过程只读取反馈和画面，不自动使能。
4. 点击开始真机回放。该按钮会使能双臂，按设定时长移动到起始帧，再逐帧回放。
5. 暂停/停止回放会保持当前姿态；停止并失能会停止两臂且不自动回零。
6. Ctrl+C 退出时也执行停止、失能及无回零断开。
关闭或刷新网页不会结束服务端回放；结束时使用停止按钮或服务端终端 Ctrl+C。

单帧定位（不连续回放）：
- 在进度条下输入原始帧号；“预览该帧”只切换录像位置，不移动机械臂。
- 连接设备后点“移动到该帧并保持”，按起点对齐时长到达该帧的 action/state 目标。
- 过渡结束后保持该固定目标，不进入下一帧；蓝线保持固定，黑线继续显示反馈并写入 CSV。
- “保持指定帧目标”表示目标已下发，并不宣称实际关节已经无误差到位。
- 可再次指定其他帧；“开始真机回放”将从当前选定帧开始连续回放。
- 暂停/停止仍使用当前反馈保持姿态，因此分析固定目标下的抖动时，直接观察单帧保持阶段。

频率与数据含义：
- 原采集频率为 30 Hz：15 Hz 为半速、30 Hz 为原速、60 Hz 为两倍速。
- 每个原始帧只下发一次，不删除迟到帧，不快速连续补发积压帧。
- 底层驱动运行于 100 Hz，目标更新频率可选 (0,100] Hz；页面同时显示实际频率。
- action 是采集时的目标动作；state 是采集时测得的关节状态。
- 直接读取 datasets 下原始 parquet，不使用标准化数据或模型预测。
- 单位根据现有明确的源数据映射转换：关节统一为绝对弧度，发给驱动时转为度。
- 前 14 维为左臂 6 关节、左夹爪、右臂 6 关节、右夹爪。第 15 维只显示，绝不下发。
- 起点对齐是可配置的单独过渡；轨迹内部不增加平滑。设为 0 会直接发送首帧目标。
- 现有驱动的机械关节限位保持原样；若限位修正目标，网页计数且 CSV 同时保留原目标与驱动接受目标。

画面与图表：
- 采集录像：读取所选轨迹原始三路视频，与当前回放帧同步。
- 实时相机：显示外部、左腕、右腕三个 RealSense 画面。缺失画面会显示状态；视频不参与动作生成。
- 12 关节图：蓝线为回放目标，黑线和圆点为当前真实关节反馈，按本次运行时间对齐。
- 离线演示模式黑线明确标为“模拟反馈”，不代表真机测量。演示不导入或连接硬件。
- 图表可切换 rad / °、30 秒 / 60 秒 / 全部记录，并导出 PNG。

记录位于本工具 runs/<时间-编号>/，不会写入 Phoenix 项目：
- run.json：数据来源、字段、频率、模式、起点对齐参数。
- joints.csv：30 Hz 采样的目标、反馈、驱动接受目标、原始帧号、阶段与设定频率。
- summary.json：完成阶段、实际下发帧数、调度迟到、驱动限位计数。
页面“导出关节 CSV”下载最近一次运行记录。关节为 rad，夹爪为 0–1。

可选启动参数：
  --project-root ../phoenix-mwa
  --interface-root ../Desktop/piper_tele/sink-bot-control_bk/example_agilex
  --robot-config configs/inference/piper_robot.json   （相对 project-root）
  --output-dir runs                                 （相对本工具目录）
  --port 8766

运行验证：
  ../phoenix-mwa/.venv-piper-inference/bin/python -m unittest -v test_replay

已验证：14 项自动检查通过；现有 546 条原始轨迹的 action/state 共 1092 个字段加载检查通过。
网页已验证选轮次、字段切换、暂停续播、15/30/60 Hz 设置，以及 CSV/PNG 下载。
这些验证使用离线演示设备；没有连接或移动真机。示例黑线为模拟反馈。

设备被占用时请在原控制程序终端正常停止；本工具不会杀死其他控制程序。
本工具沿用驱动已有的 CAN 锁和按 USB 序列号绑定左右臂，保留右臂反馈 ID 偏移适配。
