# 独立复核：空扫描存在概率语义门禁

日期：2026-10-14。复核范围为本次 `protocol.json`、`protocol_receipt.json`、`closed_form_check.json`、`official_test_summary.json`、本地 `check_empty_scan.py` 及锁定官方提交 `a4336b920a799cfe0a77ecb05867c5deeb371c7a`。没有复跑滤波器、实现修补或读取 FMCW 科学输入/结果。

**结论：应在此严格停止，不进行多帧场景或 A/B/C 控制。** 已记录的合法调用与标准常数正 (P_D) 的有效空扫描模型等价，但官方返回的存在概率没有执行应有的观测修正。数值差异不能由本次模型中的杂波、条件粒子分布或重采样解释。它是本次固定输入的可复现语义反例；不是对所有 Stone Soup 滤波器或其他版本的笼统判断。

## 输入及冻结核验

**VERIFIED**：独立重新计算 `protocol.json` 的 SHA-256 为 `d9657d13d3ef0207efcd736e45da7aab7147615f2d5023360e8670550185e076`，与冻结收据和运行结果中的协议哈希一致。收据时间 `2026-10-14T10:25:02.578914+00:00` 早于闭式计算记录及执行记录（`10:26:23.820742`、`10:26:24.183999` UTC）。参数来自官方组件测试/默认值及一个小型标量核查配置，协议没有多帧场景，也没有运行后参数搜索。

**VERIFIED**：脚本构造了 `BernoulliParticleState`、官方预测器和更新器；状态先验是预设位置网格与零速度，出生是预设矩形上的均匀备用采样。未提供目标真值、真值存在序列、未来测量或 oracle 位置。固定 (P_D=0.9)，无可见性扩展、约束或正则化。杂波配置为每扫描 Poisson 期望数 2、均匀空间密度 (1/20\ \mathrm{m}^{-1})，本次观测的实际测量集为空。

**VERIFIED**：`official_test_summary.json` 记录 7 个标准 Bernoulli 组件/状态测试通过，返回码为 0；它没有声称完整测试套件全部通过。可见性扩展及非 Bernoulli 组件被明确排除，与本次常数 (P_D) 标量问题一致。官方更新器测试仅检查有检测的更新，且对存在概率只断言非 `None`，因此这 7 个通过结果不证明空扫描后验数学正确。见锁定 [`updater/tests/test_particle.py` L139–232](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/updater/tests/test_particle.py#L139-L232)。

## 调用是否合法

**VERIFIED**：脚本实际创建的输入为

```python
MultipleHypothesis([
    SingleHypothesis(prediction, MissedDetection(timestamp=t1))
])
```

它符合官方 `MultipleHypothesis` 的 `SingleHypothesis` 序列类型要求，且含有提供预测的第一个元素。`MissedDetection` 是文档规定的 null hypothesis；没有虚构测量、修改布尔语义或绕过 API。见 [`types/multihypothesis.py` L18–36](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/types/multihypothesis.py#L18-L36)、[`types/detection.py` L43–57](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/types/detection.py#L43-L57)。

**REPRODUCED（所审查的本地执行记录）**：容器长度为 1，`SingleHypothesis` 布尔值为 `False`，`any(hypotheses)` 为 `False`。这与官方 [`types/hypothesis.py` L62–64](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/types/hypothesis.py#L62-L64) 完全一致。它不是空 Python 列表造成的 API 错误；也不是 `MultipleHypothesis([])` 缺少预测的输入错误。

## 数学等价性和差异来源

**VERIFIED**：使用预录参数 (r_{\rm prev}=0.5,p_B=0.01,p_S=0.98,P_D=0.9)，独立 binary64 算术复核得到

\[
r^- = 0.01(1-0.5)+0.98(0.5)=0.495,
\]

\[
r_{\varnothing}=\frac{0.495(1-0.9)}{1-0.495(0.9)}
=0.08926961226330025.
\]

官方预测公式相同，见 [`predictor/particle.py` L371–374](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/predictor/particle.py#L371-L374)。

**REPRODUCED（所审查的本地执行记录）**：预测返回 (r^-=0.495)，更新返回 (r^+=0.495)。相对闭式空扫描后验的绝对差为 `0.4057303877366997`，远大于冻结的 (10^{-12}) 数值容差。预测与更新的存在概率有限且在 `[0,1]`；权重有限、非负，权重和均为 `0.9999999999999998`。所以失败并非越界、权重未归一化或浮点舍入误差。

**VERIFIED（数学解释）**：有效空扫描的似然为

\[
L_0=e^{-\lambda},\qquad L_1=(1-P_D)e^{-\lambda}.
\]

公共的 Poisson 空杂波因子在 Bayes 比率中抵消。因此本次 λ、杂波空间密度的取值不能使正确的后验回到 `0.495`。常数 (P_D) 也使条件粒子密度的积分恰为 (1-P_D)；出生位置、过程噪声、测量噪声和粒子位置近似不能解释该标量差异。固定正 (P_D) 与“已进行观测”是这个门禁的明确假设；传感器关闭或 (P_D=0) 的另一种模型不适用于解释本次结果。

**VERIFIED（源码解释）**：官方更新器复制预测后，仅在 `if any(hypotheses)` 分支中计算存在概率修正。本次假设为假，分支被跳过，随后没有空测量集存在概率修正。见 [`updater/particle.py` L474–540](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/updater/particle.py#L474-L540)。重采样只抽取条件粒子并赋均匀权重；Bernoulli 状态切片保留原存在概率。见 [`resampler/particle.py` L34–56](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/resampler/particle.py#L34-L56)、[`types/state.py` L978–994](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/types/state.py#L978-L994)。因此实际返回值与这个分支行为一致，而不等价于标准有效空扫描 Bernoulli Bayes 更新。

## 停止决定与复核局限

**VERIFIED**：保持这个合法输入和观测模型时，要获得闭式后验，需要改变官方更新算法或在外部改写后验；本任务明确禁止这两种做法。依照用户的先决语义门禁与停止条件，保留失败证据并停止多帧实验是必要的。不得将单次闭式失败改写成总体复现成功，也不得用只有检测的曲线绕过漏检问题。

**UNKNOWN**：上游维护者是否将 `MissedDetection` 特意解释为“无观测机会”、是否有未文档化的外部调用约定，或其他版本是否已处理该分支，本次限定来源不能确定；不应据此猜测。即使存在不同约定，也需要上游提供与本次常数正 (P_D) 有效空扫描模型相符的合法调用证据，才能重新开启该复现。

本复核没有重新执行滤波器；执行结论以所审查的 JSON 运行记录和脚本为依据。协议收据是本地证据链，不是外部时间戳认证。本复核不独立检查安装日志或完整源目录哈希，也不声称多帧场景、目标缺席控制、杂波控制或实际传感器适用性已被验证。成功标准中的数学一致性尚未满足；加入 E18 的结论仍为 **NO**。
