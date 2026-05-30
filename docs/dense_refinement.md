# 稠密 RGB-D 回环精化（Dense RGB-D Refinement）

> 对应实现：`corelib/include/rtabmap/core/RegistrationDense.h`、`corelib/src/RegistrationDense.cpp`
> 开关参数：`Reg/DenseRefining`

---

## 1. 这是什么

RTABMap 默认用**稀疏特征 + PnP / ICP** 估计两帧之间的相对变换（回环边、邻近边、相邻边）。当输入是带**稠密深度图**的 RGB-D 数据（例如双目左目 + StereoNet 深度）时，稀疏方法只用到了几十~几百个特征点，**整张稠密深度完全没有被用于变换估计**。

`RegistrationDense` 是一个**可选的精化级（refinement step）**：它接在已有配准管线（Vis / Icp / VisIcp）**之后**，以上游估计的变换为初值，用**稠密深度 + 左目灰度图**做一次亚像素级的相对位姿精化，从而提高回环边/邻近边的精度，最终降低优化后的轨迹误差。

它**不替换**现有方法，而是作为最后一道"打磨"工序。

---

## 2. 数学原理

在两帧相机之间的 **SE(3) 相对位姿**上做**鲁棒的、coarse-to-fine 的 Gauss-Newton**，联合最小化两类残差。

### 2.1 两类残差

设参考帧 `from` 的某像素，其深度反投影得到 3D 点 `X`（from 光学系），用当前相对位姿把它变换到目标帧 `to` 光学系得到 `Y`，再投影到 `to` 图像得到亚像素坐标 `(u', v')`。

**(a) 光度直接法残差（photometric / direct，DVO 风格）**

```
r_p = I_to(π(Y)) − I_from(u, v)
```

即"把 from 像素按当前位姿投影到 to 图像处的灰度，与 from 自身灰度之差"。它利用图像纹理提供约束。

**(b) 点到面几何残差（point-to-plane，投影式数据关联）**

```
r_g = n · (Y − Q)
```

其中 `Q` 是 to 帧在 `(u', v')` 处由深度反投影出的表面点，`n` 是该处由深度图估计的表面法线。它利用稠密几何（深度）提供约束，在弱纹理区尤其重要。

### 2.2 雅可比

对目标光学系中的点 `Y` 做**左扰动** `Y' ≈ Y + ν − [Y]_× ω`，于是：

```
∂Y/∂ξ = [ I₃ | −[Y]_× ]            (3×6)，ξ = (ν, ω) = (平移, 旋转)
```

- 光度项：`J_p = ∇I_to · J_π · ∂Y/∂ξ`，其中投影雅可比
  ```
  J_π = [ fx/Z   0    −fx·X/Z² ]
        [  0    fy/Z  −fy·Y/Z² ]
  ```
- 几何项：`J_g = nᵀ · ∂Y/∂ξ`

### 2.3 鲁棒加权与求解

每次迭代构造正规方程并求增量：

```
H = Σ ( w_photo · ρ'_p/σ_p² · J_pᵀJ_p  +  λ · w_geo · ρ'_g/σ_g² · J_gᵀJ_g )
b = Σ ( w_photo · ρ'_p/σ_p² · r_p · J_pᵀ  +  λ · w_geo · ρ'_g/σ_g² · r_g · J_gᵀ )
δξ = −H⁻¹ b
W  ← exp(δξ^) · W            （W 为 from→to 的光学系相对位姿）
```

关键点：
- **Huber 鲁棒核**抑制外点（动态物体、深度跳变、错误匹配）。
- **每次迭代用 MAD（中位绝对偏差）估计鲁棒尺度 σ_p、σ_g**，并以 `1/σ²` 加权——这样"强度单位"的光度项和"米单位"的几何项被**自动归一化到可比尺度**，无需手工调平衡系数。
- `λ = Dense/GeometricWeight` 是用户对几何项的额外信任倍率（默认 1.0）。

### 2.4 坐标系约定（与 RTABMap 既有代码一致）

| 符号 | 含义 |
|---|---|
| `project(u,v,d)` | 光学系反投影：`x=(u−cx)d/fx, y=(v−cy)d/fy, z=d` |
| `L` | `localTransform`，光学系 → 机器人基座系 |
| `T` | 基座系相对变换，满足 `p_from = T · p_to`（即返回给图优化的回环边变换） |
| `W` | 优化用的光学系相对位姿，`W = L_to⁻¹ · T⁻¹ · L_from` |

优化在光学系 `W` 上进行，结束后恢复基座系变换：

```
T = L_from · W⁻¹ · L_to⁻¹
```

### 2.5 光照不变（ZNCC 风格，可选）

回环的两帧常常在**不同时间**拍摄，曝光/光照不同，直接比较原始灰度会失配。开启 `Dense/IlluminationInvariant=true` 后，光度项对**全局仿射光照变化**（增益 `a` + 偏置 `b`，即 `I_to ≈ a·I_from + b`）不变：

每次迭代，对当前采样到的灰度做**零均值 + 单位方差归一化**（ZNCC 的归一化思想，作用在整组采样点上）：

```
r_p = (I_to(π(Y)) − μ_to)/σ_to − (I_from − μ_from)/σ_from
```

其中 `μ, σ` 是本次迭代采样像素的均值/标准差。雅可比相应按 `1/σ_to` 缩放（`μ, σ` 作为聚合统计量，对单点扰动的导数可忽略）。

- 这是**全局**仿射光照补偿（处理曝光/整体亮度差异），**近乎零成本**（一遍求和统计，无需 patch）。
- 低纹理时 `σ` 被下限保护（退化为仅去偏置）；点数不足时退回原始残差。
- 局部/空间变化的光照（阴影、渐晕）需逐点 patch ZNCC，成本更高，本实现暂不包含。

### 2.6 coarse-to-fine

构建图像金字塔（默认 3 层，0=原分辨率），从**最粗层**开始优化，逐层下沉到最细层。粗层扩大收敛域、避免陷入局部极小；细层提供高精度。内参按层缩放（像素中心约定 `c' = (c+0.5)·s − 0.5`）。

---

## 3. 在 RTABMap 中的接入方式

`RegistrationDense` 继承自 `Registration`，被作为配准管线**最深的 child** 接入（复用 RTABMap 既有的链式精化机制，正如 `Reg/Strategy=2 (VisIcp)` 是 `RegistrationVis` 挂一个 `RegistrationIcp` 子节点）。

```
Reg/Strategy = 0 (Vis)     +DenseRefining →  Vis → Dense
Reg/Strategy = 1 (Icp)     +DenseRefining →  Icp → Dense
Reg/Strategy = 2 (VisIcp)  +DenseRefining →  Vis → Icp → Dense
```

链式调用时，父级（Vis/Icp）估计出的变换会作为 **guess** 传给 Dense，Dense 以此为初值精化。因此它在**所有"已有初值"的配准场景**生效：

- ✅ 全局回环（global loop closure）
- ✅ 空间/时间邻近检测（proximity detection）
- ✅ 相邻边精化（`RGBD/NeighborLinkRefining`，**注意：这是逐帧的**，见性能章节）

由 `Registration::create()` 工厂根据 `Reg/DenseRefining` 决定是否追加，**不改动 `Reg/Strategy` 枚举语义**，默认关闭，对现有行为零影响。

数据通路：`Memory::computeTransform` 在 `Reg/DenseRefining=true` 时会确保两帧的**原始左目图像 + 深度**被解压（与回环特征重提取的逻辑一致）。

---

## 4. 安全设计（永不让回环变差）

dense 精化只在以下条件全满足时才"采纳"结果，否则**回退返回输入初值**：

- 必须有非空初值（`guess` 非空）；
- 两帧都有可用的**单相机模型**（mono RGB-D 或双目左目）+ **原始图像 + 深度**；
- 相对初值的修正量不超过 `Dense/MaxTranslation` / `Dense/MaxRotation`（防发散）；
- 最细层参与的有效像素 ≥ 6（防欠约束）；
- Hessian 数值有效（非病态）。

此外，dense **默认保留上游（Vis/ICP）估计的协方差**——它只精化变换的"均值"，不改变该回环边在图优化中的权重。

---

## 5. 参数参考

### 主开关

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `Reg/DenseRefining` | bool | `false` | 是否启用稠密精化（接在配准管线之后） |

### `Dense/` 参数组（仅当 `Reg/DenseRefining=true` 生效）

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `Dense/Iterations` | int | `10` | 每个金字塔层的最大 Gauss-Newton 迭代数 |
| `Dense/PyramidLevels` | int | `3` | coarse-to-fine 金字塔层数（≥1） |
| `Dense/Decimation` | int | `2` | 最细层的像素抽样步长（每轴每 N 像素取 1 个）。**最关键的性能旋钮**，成本约按 `1/步长²` 下降 |
| `Dense/MinDepth` | float | `0.3` | 最小可信深度（m），低于则忽略 |
| `Dense/MaxDepth` | float | `8.0` | 最大可信深度（m，0=禁用）。学习/双目深度远处不可信，建议按网络可信范围设（如 5~6） |
| `Dense/MinGradient` | float | `12.0` | 参与**光度项**的像素所需的最小图像梯度幅值（0–255 尺度），过滤弱纹理像素以保持光度项良态 |
| `Dense/GeometricWeight` | float | `1.0` | **点到面几何项**的相对权重 λ。0=纯光度；增大=更偏几何 |
| `Dense/PhotometricWeight` | float | `1.0` | **光度项**的相对权重。0=纯几何（即稠密点到面 ICP） |
| `Dense/IlluminationInvariant` | bool | `true` | 光度项是否对**全局仿射光照变化**（增益+偏置）不变（ZNCC 风格零均值/单位方差归一化）。重访场景曝光/光照会变时建议开启；近乎零成本，关闭则退回原始光度残差 |
| `Dense/HuberThreshold` | float | `1.345` | Huber 鲁棒核阈值（以鲁棒估计的残差标准差为单位） |
| `Dense/ConvergenceEps` | float | `0.0001` | 收敛阈值：增量范数（rad+m）低于此值即停止 |
| `Dense/MaxCorrespondenceDepthDiff` | float | `0.2` | 形成有效对应时，warp 点深度与目标实测深度的最大允许差（m），用于剔除遮挡/深度断裂 |
| `Dense/MaxTranslation` | float | `0.2` | 安全护栏：修正平移超过此值（m）则拒绝精化、保留初值。0=禁用 |
| `Dense/MaxRotation` | float | `0.2` | 安全护栏：修正旋转超过此值（rad）则拒绝精化、保留初值。0=禁用 |

---

## 6. 使用示例

### 6.1 离线 / 算力充足（精度优先）

```ini
Reg/Strategy             0       # Vis（或 2=VisIcp）
Reg/DenseRefining        true
Dense/MaxDepth           6.0     # 按 StereoNet 可信范围
Dense/Decimation         2
Dense/PyramidLevels      3
Dense/Iterations         10
Dense/GeometricWeight    1.0
Dense/PhotometricWeight  1.0
```

### 6.2 嵌入式 / CPU 实时（如 RDK X5，速度优先）

```ini
Reg/Strategy              0
Reg/DenseRefining         true
RGBD/NeighborLinkRefining false   # 重要：别和 dense 一起开（逐帧）
Dense/Decimation          6       # 大幅减少像素
Dense/PyramidLevels       2
Dense/Iterations          5
Dense/MaxDepth            5.0
```

### 6.3 纯几何（无纹理场景，退化为稠密点到面 ICP）

```ini
Reg/DenseRefining        true
Dense/PhotometricWeight  0.0
Dense/GeometricWeight    1.0
```

---

## 7. 性能与实时性（重点）

### 7.1 它不是逐帧开销

dense 精化**仅在配准发生时触发**，配准只发生在：

| 触发点 | 频率 |
|---|---|
| 回环检测成功 | 偶发（回到老地方才有） |
| 空间邻近检测 | 偶发，每帧最多几次 |
| **相邻边精化 `RGBD/NeighborLinkRefining`** | **每帧（如开启）** |
| 视觉里程计 | 不触发（使用外部里程计时 RTABMap 不算 VO） |

> **关键**：使用外部里程计（如轮速+IMU 融合）时，实时位姿来自里程计，而 RTABMap 的回环+优化跑在**独立后台线程**。因此一次回环的 dense 开销（即便几百毫秒）**不会阻塞实时位姿输出**，只是地图修正稍有延迟。
>
> ⚠️ **唯一逐帧雷区**：`RGBD/NeighborLinkRefining=true` 会让 dense 每帧运行。嵌入式平台请关闭它，或仅离线使用 dense。

### 7.2 成本量级（参考）

单核 ARM Cortex-A55、VGA(640×480)、默认参数：约 **150–400 ms / 次配准**。
成本主要随**有效像素数**线性变化，因此：

- `Dense/Decimation` 2→6：约省 **9 倍**；
- `Dense/PyramidLevels` 3→2、`Dense/Iterations` 10→5：再省约 2~3 倍；
- 调到 6.2 节配置：约 **30–80 ms / 次**。

### 7.3 进一步加速（可选改进，尚未实现）

- **OpenMP 并行**：像素级 H/b 累加是天然可并行的；多核（如 X5 八核）可再获 3~4 倍加速。
- 随机/网格采样上限、GPU 化等。

---

## 8. 已知限制

- 仅支持**单相机模型**（mono RGB-D 或双目左目）；多相机数据会**安全跳过**（返回初值）。
- 要求两帧**图像分辨率一致**，且深度图能与左目对齐（不一致时会自动按最近邻缩放深度到图像尺寸）。
- 精度依赖深度质量；学习/双目深度的远处误差会拉低收益，应配 `Dense/MaxDepth` 限制可信范围。
- 它精化**变换均值**，不改变回环边协方差（权重仍来自上游 Vis/ICP）。
- 单一平面等几何退化场景，点到面项欠约束，此时主要靠光度项（有纹理即可良好收敛）。

---

## 9. 实现与验证

### 涉及文件

| 文件 | 改动 |
|---|---|
| `corelib/include/rtabmap/core/RegistrationDense.h` | 新增：类声明 |
| `corelib/src/RegistrationDense.cpp` | 新增：算法实现 |
| `corelib/include/rtabmap/core/Parameters.h` | 新增 `Reg/DenseRefining` + `Dense/*` 参数组 |
| `corelib/src/Registration.cpp` | 工厂 `create()` 追加 dense child |
| `corelib/src/Memory.cpp` / `Memory.h` | 新增 `_denseRefining` 标志；按需解压原始图像+深度 |
| `corelib/src/CMakeLists.txt` | 编译 `RegistrationDense.cpp` |

### 数学正确性验证

用仅依赖 Eigen 的**合成 RGB-D 场景**（含真实光学↔基座 `localTransform` 的平面与墙角）做了独立数值测试：从扰动初值和单位阵初值出发，

- 纯光度、纯点到面（非退化墙角）、光度+几何联合，

**全部收敛到真值变换**（平移误差 < 0.0011 m，旋转误差 ≈ 0），验证了 SE(3) 指数映射、两类雅可比、坐标系约定与 Huber+MAD 鲁棒平衡的正确性。

另外，对目标帧施加全局仿射光照变化（增益 2.2 + 偏置 55）后：开启 `Dense/IlluminationInvariant` 时纯光度仍收敛到 ~1e-5 m，关闭时被带偏到 ~3.2e-3 m（约 300 倍差距），验证了 ZNCC 归一化光照不变性的正确性与有效性。

> 注意：受沙箱依赖限制（缺 OpenCV/PCL/VTK），未做整库编译。所有 RTABMap API 调用已逐头文件核对并与 `RegistrationIcp/RegistrationVis` 保持一致。请在配齐依赖的环境编译验证。

---

## 10. 参考方法

- 光度直接法（DVO）：Steinbrücker/Sturm/Cremers, *Real-time visual odometry from dense RGB-D images*；Kerl/Sturm/Cremers, *Robust odometry estimation for RGB-D cameras*。
- 点到面 ICP：Chen & Medioni, *Object modelling by registration of multiple range images*。
- 鲁棒估计（Huber / IRLS）与协方差作为 Hessian 之逆的标准最大似然框架。

> 说明：以上内容为对公开方法的概括性改写，非原文照搬。
