# Piper Replay Tool

Piper 双臂机器人的**原始采集数据回放工具**。直接读取数据集里未处理的 parquet 与三路视频，按 `action`（采集时的目标动作）或 `state`（采集时的实测关节状态）逐帧下发到真机，同时提供网页界面、实时相机画面和 12 关节目标/反馈对比图。

工具不加载策略权重，不运行 RTC 或任何模型推理，也不修改原始训练数据与机器人驱动。硬件模式只做"读取反馈 + 下发关节目标"。

## 特性

- **原始数据直读**：直接解析 `datasets/` 下原始 parquet，不使用标准化数据或模型预测值。
- **完整回放时钟**：原采集频率 30 Hz，支持 7.5 / 15 / 30 / 60 Hz（¼× / ½× / 1× / 2×）及任意 (0, 100] Hz。
- **单帧定位**：可跳到任意原始帧并"移动到该帧并保持"，用于分析固定目标下的抖动与稳态误差。
- **起点对齐**：从当前姿态到首帧的可配置过渡时长；设为 0 则直接发送首帧，轨迹内部不插值、不平滑。
- **同步录像**：读取所选轨迹原始三路视频，与当前回放帧对齐。
- **实时相机**：外部、左腕、右腕三路 RealSense 画面。
- **关节曲线**：蓝线为回放目标，黑线为真实反馈，支持 rad / ° 切换、30 s / 60 s / 全部记录窗口、PNG 导出。
- **运行留痕**：每次回放写入 `runs/<时间-编号>/`（`run.json` / `joints.csv` / `summary.json`），记录调度迟到与驱动限位修正次数。
- **离线演示模式**：`--demo` 不导入、不连接任何硬件，黑线明确标注为"模拟反馈"。

## 安装

```bash
git clone https://github.com/appaletrees/piper_replay_tool.git
cd piper_replay_tool

python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`requirements.txt` 只覆盖网页与数据读取（Flask / numpy / pyarrow / av / opencv）。**硬件模式**还需要机器人控制环境中已有的 Piper SDK、`scipy`、`pyrealsense2`，以及可用的 CAN 接口；不需要 torch 或 CUDA。

## 运行

离线演示（无需硬件）：

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python -u app.py --demo --port 8766
```

真机模式：

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python -u app.py --hardware --port 8766
```

浏览器打开 <http://127.0.0.1:8766>。同一端口只运行一个实例，`Ctrl+C` 停止网页服务。

### 常用参数

| 参数 | 说明 |
| --- | --- |
| `--project-root` | 数据与配置所在的项目根目录，默认 `../phoenix-mwa` |
| `--interface-root` | 机器人控制示例目录，默认 `../Desktop/piper_tele/sink-bot-control_bk/example_agilex` |
| `--robot-config` | 机器人配置路径（相对 `--project-root`），默认 `configs/inference/piper_robot.json` |
| `--output-dir` | 运行记录目录（相对本工具目录），默认 `runs` |
| `--port` | 网页端口，默认 `8766` |
| `--demo` / `--hardware` | 离线模拟 / 真机执行 |

首次在自己机器上运行前，请按实际路径覆盖 `--project-root` 与 `--interface-root`。

## 使用流程

1. 选择数据集、原始 episode 编号（从 0 开始）与 `action` / `state`，点击加载。
2. 选择起始帧、目标更新频率和起点对齐时长。
3. 点击"连接设备与相机"——该步骤只读取反馈与画面，**不自动使能**。
4. 点击"开始真机回放"——会使能双臂，按设定时长移动到起始帧，再逐帧回放。
5. 暂停 / 停止回放会保持当前姿态；"停止并失能"停止两臂且不自动回零。
6. `Ctrl+C` 退出时同样执行停止、失能及无回零断开。

> 关闭或刷新网页不会结束服务端回放。结束请用停止按钮或服务端终端 `Ctrl+C`。
>
> 设备被占用时，请在原控制程序终端正常停止；本工具不会杀死其他控制程序。

## 数据约定

- 前 14 维依次为：左臂 6 关节、左夹爪、右臂 6 关节、右夹爪。**第 15 维只显示，绝不下发。**
- 关节统一按绝对弧度处理，发给驱动时转为度；夹爪保持原有 0–1 定义。
- 每个原始帧只下发一次，不删除迟到帧，不快速连续补发积压帧。
- 底层驱动运行于 100 Hz，页面同时显示设定频率与实际频率。
- 驱动的机械关节限位保持原样；若限位修正了目标，网页会计数，CSV 中同时保留原目标与驱动接受的目标。

## 输出格式

记录位于 `runs/<时间-编号>/`，不写入上游项目：

- `run.json` — 数据来源、字段、频率、模式、起点对齐参数
- `joints.csv` — 30 Hz 采样的目标、反馈、驱动接受目标、原始帧号、阶段、设定频率
- `summary.json` — 完成阶段、实际下发帧数、调度迟到、驱动限位计数

页面"导出关节 CSV"下载最近一次运行记录。

## 测试

```bash
python -m unittest -v test_replay
```

## 已验证范围

- 14 项自动检查通过；现有 546 条原始轨迹的 action / state 共 1092 个字段加载检查通过。
- 网页已验证选轮次、字段切换、暂停续播、7.5 / 15 / 30 / 60 Hz 设置，以及 CSV / PNG 下载。
- 上述验证均使用离线演示设备，**未连接或移动真机**；演示中的黑线为模拟反馈，不代表真机测量。

## 许可证

MIT，见 [LICENSE](LICENSE)。
