# Stone Soup v1.9.1 Bernoulli 源码语义核查

核查日期：2026-10-14。范围仅限官方锁定提交 `a4336b920a799cfe0a77ecb05867c5deeb371c7a` 的源码、类型定义和测试；本文件不报告运行结果，也未使用 FMCW 输入或结果。`VERIFIED` 表示可直接从该提交核查的事实；本文件没有单独宣称 `REPRODUCED` 的数值结果。

## Bernoulli 状态与条件分布

**VERIFIED**：`BernoulliParticleState` 将 `existence_probability` 作为独立属性存储。粒子及归一化权重描述“目标存在条件下”的状态分布，不应将粒子权重之和当作目标存在概率。Bernoulli 表示为

\[
P(X=\varnothing)=1-r,\qquad
f(X=\{x\})=r\,s(x),\qquad \int s(x)\,dx=1.
\]

其中 (r) 是存在概率，(s(x)) 是条件状态密度。状态切片会保留存在概率。见 [`types/state.py` L942–960、L965–995](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/types/state.py#L942-L995)。

## 预测：出生、存活与时间

**VERIFIED**：`BernoulliParticlePredictor` 的 `birth_probability`、`survival_probability` 和 `birth_sampler` 分别定义出生概率、存活概率与出生空间采样器。`predict` 添加出生粒子，将存活和出生粒子一起通过转移模型传播，再构造存在概率与粒子权重。见 [`predictor/particle.py` L280–289、L317–368](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/predictor/particle.py#L280-L368)。

存在概率预测在 `estimate_existence` 中直接计算：

\[
r^- = p_B(1-r_{\mathrm{prev}})+p_Sr_{\mathrm{prev}}.
\]

存活和出生粒子的权重分别乘以 (p_Sr_{\mathrm{prev}}/r^-) 和 (p_B(1-r_{\mathrm{prev}})/r^-)，随后再次归一化。见 [`predictor/particle.py` L371–391](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/predictor/particle.py#L371-L391)。

**VERIFIED**：这里的 (p_B,p_S) 每次调用使用一次；存在预测函数没有将它们根据时间间隔换算为连续时间风险率。时间戳差会传给状态转移模型，但不进入上述存在概率公式。因此采样间隔必须显式固定，不能将同一组概率无说明地用于任意扫描周期。见 [`predictor/particle.py` L310–345、L371–374](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/predictor/particle.py#L310-L374)。

**VERIFIED**：预测器从 **prior** 的假设集合提取测量，并将其传给出生采样器；不是读取未来真值。`SwitchingDetectionSampler` 在没有测量时使用预设的备用采样器。一个宽的、预先指定的备用状态分布可支持非 oracle 出生，但具体实验仍须记录它与初始状态先验、真值之间的区别。见 [`predictor/particle.py` L395–405](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/predictor/particle.py#L395-L405)、[`sampler/detection.py` L107–144](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/sampler/detection.py#L107-L144)。

## 测量更新：检测概率与杂波

**VERIFIED**：`BernoulliParticleUpdater` 的参数具有不同含义：

| 参数 | 锁定源码的含义 |
|---|---|
| `detection_probability` | 基础类为每个粒子使用相同标量 (P_D) |
| `clutter_rate` | 每扫描期望杂波测量数 λ；文档假定计数服从 Poisson 分布 |
| `clutter_distribution` | 测量空间中的杂波空间密度 (c(z))，通常为均匀密度 |
| `clutter_rate * clutter_distribution` | 更新中使用的杂波强度 κ(z)=λ c(z)，量纲为“每扫描、每测量空间体积的期望计数” |

这些量不是空间单元历史占用频率。见 [`updater/particle.py` L443–459、L490–515、L542–547](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/updater/particle.py#L443-L547)。属性说明提到 `detection_probability=None` 时继承输入，但基础类 `get_log_detection_probability` 实际直接计算 `np.log(self.detection_probability)`；该实验应显式提供数值 (P_D)，不依赖未验证的 `None` 行为。

**VERIFIED**：对进入非空测量更新分支的假设集合，记预测条件粒子密度为 (s^-(x))，源码实现下述量的粒子近似：

\[
\delta = \int P_D(x)s^-(x)\,dx
-\sum_{z\in Z}\frac{\int P_D(x)g(z\mid x)s^-(x)\,dx}{\lambda c(z)},
\qquad
r^+=\frac{r^-(1-\delta)}{1-r^-\delta}.
\]

条件粒子权重乘以

\[
1-P_D(x)+\sum_{z\in Z}\frac{P_D(x)g(z\mid x)}{\lambda c(z)},
\]

随后归一化。源码的常数杂波密度配置对应这里的常数 (c(z)=c)。见 [`updater/particle.py` L483–519](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/updater/particle.py#L483-L519)。

## 漏检假设与 `any(hypotheses)`

**VERIFIED**：`MissedDetection` 是官方用于“没有检测与该轨迹关联”的 null hypothesis 类型，默认状态向量为 `None`，其布尔值为 `False`。`SingleHypothesis.__bool__` 在测量为 `MissedDetection` 或 `None` 时返回 `False`。`MultipleHypothesis` 接受并验证 `SingleHypothesis` 的序列。见 [`types/detection.py` L43–57](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/types/detection.py#L43-L57)、[`types/hypothesis.py` L53–64](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/types/hypothesis.py#L53-L64)、[`types/multihypothesis.py` L12–43、L67–69](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/types/multihypothesis.py#L12-L69)。

只含漏检的合法类型表示为：

```python
hypotheses = MultipleHypothesis([
    SingleHypothesis(
        prediction,
        MissedDetection(timestamp=prediction.timestamp),
    )
])
```

其 `len(hypotheses)` 为 1，但 `any(hypotheses)` 为 `False`。因此 `if any(hypotheses)` 表示“至少有一个真实测量假设”，而不是“容器是否非空”，也不是判断生成这些测量的真实目标是否存在。杂波测量仍然是交给滤波器的普通 `Detection`；不得使用生成真值将其在更新前剔除。

**VERIFIED**：空 Python 列表 `[]` 不是该更新器的合法输入；`update` 在开始即访问 `hypotheses.single_hypotheses[0].prediction`。根据源码，`[]` 预期触发 `AttributeError`；`MultipleHypothesis([])` 虽是合法的容器对象，却不能为该更新调用提供预测，预期触发 `IndexError`。这是源码推导的异常类型，运行是否吻合由单独的执行记录报告。见 [`updater/particle.py` L461–482](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/updater/particle.py#L461-L482)。

**VERIFIED**：不要把漏检占位符与真实测量混放进 Bernoulli 检测分支后视为标准测量集合；该分支直接遍历所有假设的 `measurement` 并计算测量似然，没有先过滤 `MissedDetection`。官方 Bernoulli 检测测试传入的是仅包含真实测量的集合。见 [`updater/particle.py` L483–499](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/updater/particle.py#L483-L499)、[`updater/tests/test_particle.py` L214–217](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/updater/tests/test_particle.py#L214-L217)。

## 空测量集的数学检查与源码分支

**VERIFIED（数学推导）**：对于进行了有效观测、常数 (P_D)、归一化条件密度且测量集 (Z=\varnothing) 的单目标 Poisson 杂波模型，

\[
P(Z=\varnothing\mid X=\varnothing)=e^{-\lambda},\qquad
P(Z=\varnothing\mid X=\{x\})=(1-P_D)e^{-\lambda}.
\]

Bayes 更新使公共杂波项抵消，给出

\[
r_{\mathrm{empty}}=\frac{r^-(1-P_D)}{1-r^-P_D}.
\]

也可将上述通用更新的求和置空，得到 δ=P_D。对 (0<r^-<1,0<P_D<1)，它严格小于 (r^-)。在常数 (P_D) 下条件空间密度保持不变；存在概率下降并不意味着条件粒子密度必须收缩。若 (P_D(x)) 随状态变化，空扫描还会按 (1-P_D(x)) 改变条件状态分布。

**VERIFIED（源码）**：更新器首先从预测复制状态，然后把存在概率修正放在 `if any(hypotheses)` 内。只有漏检假设时，该分支不执行。后续仅有可选约束、重采样和条件正则化，没有显式空测量集存在概率修正。见 [`updater/particle.py` L474–540](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/updater/particle.py#L474-L540)。重采样通过状态切片保留存在概率并重新赋归一化粒子权重，见 [`resampler/particle.py` L34–56](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/resampler/particle.py#L34-L56)、[`types/state.py` L978–994](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/types/state.py#L978-L994)。

**UNKNOWN（本源码核查本身未执行）**：合法只含漏检假设的实际返回值与上述闭式结果是否一致，须由独立记录的数值检查确定。本文件不先行将分支差异称为已执行证实的 bug，也不通过改源码、虚构测量或覆盖返回值来强制一致。

**VERIFIED（条件区分）**：(Z=\varnothing) 必须表示一次有效观测没有产生测量。传感器关闭、扫描未执行、数据丢失或目标处于无观测能力区域，不等价于具有正 (P_D) 的有效空扫描；若此时 (P_D=0)，观测本身不降低存在概率。基础更新器没有单独的 `sensor_off` 参数；调用方必须保证观测模型的适用性。不得用“传感器关闭”的解释掩盖固定正 (P_D) 的合法空扫描检查结果。

## 官方测试覆盖与局限

**VERIFIED**：

- `test_bernoulli_particle_no_detection` 构造无历史测量的预测，核查出生备用采样、存在概率预测、状态传播和权重归一化。这里的 “no_detection” 是预测器输入场景，不是对空扫描后验存在概率的测试。见 [`predictor/tests/test_particle.py` L107–219](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/predictor/tests/test_particle.py#L107-L219)。
- `test_bernoulli_particle_detection` 使用 prior 中的历史检测采样出生粒子，并核查预测。见 [`predictor/tests/test_particle.py` L241–368](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/predictor/tests/test_particle.py#L241-L368)。
- `test_bernoulli_particle` 的更新输入全部是真实测量对象；断言检查输出粒子数、时间戳、权重和假设，并仅检查存在概率“不是 None”，没有核查闭式数值，也未覆盖仅有漏检占位符的后验。见 [`updater/tests/test_particle.py` L139–232](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/updater/tests/test_particle.py#L139-L232)。
- `BernoulliParticleState` 参与类型层的粒子索引/父状态测试；这些测试不能替代存在性 Bayes 更新检查。见 [`types/tests/test_state.py` L266–299、L372–389](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/types/tests/test_state.py#L266-L389)。

因此，官方组件测试通过只能证明其覆盖的断言成立。合法漏检表示、闭式存在更新和多扫描目标缺席行为，必须分别执行并记录；不能从检测分支测试通过推断它们已经复现。

## 后续本地执行补记（根代理，2026-10-14）

**REPRODUCED：** 7项锁定官方测试通过。使用上述合法只含 `MissedDetection` 的容器，`len=1`、`any=False`；`r_prev=.5,p_B=.01,p_S=.98,P_D=.9` 时，官方预测 `r_pred=.495` 与独立公式一致，但官方更新 `r_post=.495`，独立空扫描后验为 `.08926961226330025`。详见 [closed_form_check.json](closed_form_check.json)。本节消除了前文源码核查本身保留的数值UNKNOWN，但不扩展为所有Stone Soup配置均有缺陷的断言。

固定正P_D、有效空扫描、无约束/正则化；Poisson空杂波项抵消。未发现可使两结果等价的附加模型项；返回值不经补正。按停止条件终止于该单步门槛，未生成/运行A/B/C多帧场景，未改上游算法。
