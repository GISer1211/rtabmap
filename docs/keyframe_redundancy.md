# 共视关键帧冗余消减（Covisibility-based Keyframe Redundancy Reduction）

> 对应实现：`corelib/src/Rtabmap.cpp`（`Rtabmap::process` 度量段）
> 开关参数：`Mem/CovisibilityRedundancyRatio`

---

## 1. 这是什么

在建图过程中，从**关键帧的角度减少冗余**：把"场景内容已被前一关键帧充分覆盖"的新节点**降级为中间节点（intermediate node）**——它的里程计/邻接链接仍保留在位姿图中（局部精度不丢），但它被排除出回环检测、图像数据被释放。

目标是：**在不损失回环召回、不破坏里程计约束的前提下，减少回环数据库里的近重复关键帧**，从而降低内存/计算，并有机会**提升回环精度**。

---

## 2. 背景：RTABMap 已有的冗余控制与缺口

| 机制 | 参数 | 判据 | 局限 |
|---|---|---|---|
| 固定距离采样 | `RGBD/LinearUpdate` / `AngularUpdate` | 位移 | 只看距离，重叠多的地方照样建满 |
| 外观复诵合并 | `Mem/RehearsalSimilarity` | 相邻帧外观相似度 | 只比**相邻**帧 |
| 中间节点 | `Rtabmap/CreateIntermediateNodes` | 检测频率（时间） | 由 `DetectionRate` 驱动，与场景内容无关 |
| 图缩减 | `Mem/ReduceGraph` | 回环时合并 | 较粗 |

**缺口**：没有"这个关键帧的场景**是否已被其它关键帧覆盖**"这一全局判据。本功能补上它。

---

## 3. 方法

### 3.1 冗余判据：复用 RTAB-Map 的共视相似度

每生成一个新地图节点，用 RTAB-Map **自带的相似度函数** `Signature::compareTo()`（与 rehearsal 用的是同一个）计算它相对**最后保留的关键帧（锚点 anchor）**的共视相似度：

```
covisibility = compareTo(current, anchor) = 匹配词数 / max(|words_current|, |words_anchor|)   ∈ [0,1]
```

`covisibility > Mem/CovisibilityRedundancyRatio`（如 0.8）→ 判定为冗余。

> 复用 `compareTo()` 而非重写：与 rehearsal 语义一致、正确处理无效词、并自动支持全局描述子（若启用）。

### 3.2 与 rehearsal 互补（这才是增量价值所在）

RTAB-Map 的 rehearsal 也用 `compareTo()` 把相似节点合并，但 `Memory::rehearsalMerge()` 里有一道 **`!isMoving` 闸门**——只在运动小于更新阈值（`RGBD/LinearUpdate`/`AngularUpdate`）时才合并。也就是说：

- **rehearsal / smallDisplacement** → 处理"**几乎不动**且相似"的帧；
- **本功能（`!smallDisplacement` 时触发）** → 处理"**已经在动**但与锚点高度重叠"的帧（如长走廊里慢速前进，每帧重叠 90%）。

这些"在动但重叠"的帧位移超过阈值，现有机制不会动它们、照样建完整关键帧——正是本功能消减的冗余来源。

### 3.3 为什么内建了独特性保护（B）

`compareTo` 以 `max(words)` 归一化：**带来大量新内容的帧，其词数大 → 分母大 → 相似度低 → 被保留**。所以"独特地点保护"是判据内建的，无需额外规则。（该归一化比"按当前帧归一化"更保守，更不易误删。）

### 3.4 抗感知混叠：单锚点设计

锚点恒为**紧邻的上一关键帧**（通过里程计链相连，空间上相邻）。当前帧只与它比较——因此"高相似度"必然意味着**真实的局部重叠**，而不是"两段长得像但物理不同的走廊"那种全局混叠。混叠需要与**远处**关键帧比较，而本功能从不这样做。
（更激进的"对空间近邻关键帧并集算覆盖"可消减更多冗余，但会引入混叠风险、需要里程计空间闸门；为优先保证召回，本版采用更保守的单锚点。）

### 3.5 锚点 = 最后保留的**完整**关键帧（自限制，且这是正确性要点）

覆盖率始终对**最后一个被保留的完整关键帧（weight≥0）**计算，**绝不**对上一个节点计算：

- 实现上 `_lastKeyframeId` **只在节点被保留时更新**，降级时保持不变 → 锚点恒指向最后一个完整关键帧；
- **为什么必须如此**：`convertToIntermediate()` 在默认 `Mem/SaveIntermediateNodeData=false` 时会 `removeAllWords()`，把降级帧的词清空。若错误地把锚点指向"上一个节点"，长走廊里 N+1 降级后词为空，N+2 与空词比 → 相似度≈0 → 误判"场景变了"→ 每隔一帧才降一个，压缩率腰斩。本实现以"最后保留关键帧"为锚点从根本上避免该问题（并有 `!anchor->getWords().empty()` 兜底）；
- 只要当前视野仍被锚点覆盖（相似度高），就持续降级；相对锚点场景变化够大（相似度跌破阈值）则保留当前帧为新关键帧并**更新锚点**；
- 这天然**限制了降级链长度**——降级的每一帧都相对同一个"仍然覆盖它"的锚点冗余。

### 3.6 降级动作

冗余帧调用 `Memory::convertToIntermediate()`：

- `weight = -1` → 被回环检测统一跳过；
- `disableWordsRef()` → 从词典索引移除（不再参与回环似然）；
- 释放图像/原始数据；
- **不修改任何链接** → 里程计/邻接边保留，位姿图约束完整。

这与 RTABMap 既有的 `Rtabmap/CreateIntermediateNodes` 中间节点是同一套机制，只是触发条件从"检测频率"换成"共视覆盖率"。

### 3.7 安全网

`Mem/CovisibilityMaxIntermediateNodes`（默认 0=无限）：限制**连续**降级的节点数，超过则强制保留一个关键帧，给回环关键帧间隔加一道上限。

---

## 4. 在 RTABMap 中的接入位置

注入点在 `Rtabmap::process()` 的"Metric"段、`smallDisplacement`/`tooFastMovement` 处理之后、**回环检测之前**（回环检测以 `signature->getWeight() >= 0` 为门控，约 `Rtabmap.cpp:2069`）。

因此被降级的帧（`weight=-1`）在**本轮**就会被回环检测、邻接边精化等环节统一跳过，行为与 `DetectionRate` 产生的中间节点完全一致。

执行顺序（每个新节点）：
```
建签名(含BoW词) → 度量(位移/速度) → [本功能: 共视覆盖率判定 → 冗余则 convertToIntermediate]
                                  → 回环检测(跳过 weight=-1) → 图优化(用所有邻接边)
```

锚点状态（`_lastKeyframeId` / 连续降级计数）在 `close()` / `resetMemory()` / `triggerNewMap()` 处复位；若锚点已被转入 LTM 取不到，则安全回退（保留当前帧为新关键帧）。

---

## 5. 参数参考

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `Mem/CovisibilityRedundancyRatio` | float | `0.0` | 0=禁用。>0 时，新节点相对最后保留关键帧的共视覆盖率超过此值则降级为中间节点（保留里程计边、排除回环、释放数据）。推荐 0.8。仅建图模式（`Mem/IncrementalMemory=true`）生效 |
| `Mem/CovisibilityMaxIntermediateNodes` | int | `0` | 0=无限。连续降级达到此数则强制保留一个关键帧，限制回环关键帧间隔的安全网。可设 10~20 |

---

## 6. 对回环召回与精度的影响（精确表述）

**召回不是无条件不变的——它取决于保留锚点的密度/覆盖**。精确的不变量：

> 只要**保留锚点对某地点的覆盖**，在重访时仍然成立（重访帧与某个保留锚点的相似度 ≥ 回环检测所需），该地点的回环召回就不下降。

- 因为降级帧（`weight=-1`）本身既不发起回环查询、也不作候选（被 `getWeight()>=0` 门控跳过，约 `Rtabmap.cpp:2069`），重访时回环只在**保留的锚点**上尝试。
- 首次建图时，相邻两个锚点 A1、A2 之间被降级的帧，每一帧都与 A1（或 A2）相似度 > 阈值 → A1、A2 **合起来覆盖了整段轨迹**。因此从**相近视角**重访时，重访帧必与覆盖该点的锚点相似 → 回环可被检测，召回不降。
- **风险**：若重访视角与建图差异很大，稀疏锚点恰好没有合适视角的帧 → 召回可能下降（这是所有关键帧稀疏化的共性，被去冗放大）。
- **控制旋钮**：`Mem/CovisibilityMaxIntermediateNodes` 限制连续降级数 = **限制锚点最大间距**，是召回的安全网；越小越保守（锚点越密、召回越稳、压缩越少）。把它当作"召回↔压缩"的折中旋钮。

可能的提升：
- **精度可能升**：近重复关键帧减少 → 贝叶斯后验不再被一堆相似候选稀释、词典更干净、更具区分度。
- **更快**：回环检索集合更小、图更小、优化更快（省下的算力可用于更强的优化）。
- **里程计不丢**：降级帧保留邻接边，位姿图约束完整。

---

## 7. 使用示例

### 7.1 启用（推荐起步）
```ini
Mem/CovisibilityRedundancyRatio       0.8
Mem/CovisibilityMaxIntermediateNodes  0      # 或 10~20 作安全网
```

### 7.2 更激进减冗余（重叠多、内存紧张）
```ini
Mem/CovisibilityRedundancyRatio       0.7
Mem/CovisibilityMaxIntermediateNodes  20
```

### 7.3 与现有机制配合
- 可与 `RGBD/LinearUpdate`、`Mem/RehearsalSimilarity` 同时使用，三者互补（距离 / 相邻外观-静止 / 在动时的共视）。
- **不要**与 `Mem/ReduceGraph=true` 同时使用（不支持中间节点；同时开会被自动禁用并告警）。
- 验证阶段可设 `Mem/SaveIntermediateNodeData=true` 以便可逆/调试，稳定后关闭省内存。
- 若同时用 `Rtabmap/CreateIntermediateNodes`，注意 `Mem/RehearsalIdUpdatedToNewOne` 应保持 false。

---

## 8. 已知限制与兼容性

- **与 `Mem/ReduceGraph` 互斥**：图缩减不支持中间节点（源码 `Memory::moveSignatureToWMFromSTM` 明确 "Graph reduction with intermediate nodes is not supported"）。本功能会降级出中间节点，故二者不能同时开；若检测到同时开启，会打印告警并**自动禁用本功能**。
- **可逆性 / 调试**：`Memory::convertToIntermediate()` 是破坏性的（清特征、从倒排索引移除、默认删词）。若需先验证再省内存，可设 `Mem/SaveIntermediateNodeData=true`：降级节点仍被排除出回环检测，但其词被保留（可恢复/可调试），稳定后再关掉以释放内存。**测试陷阱**：用 `SaveIntermediateNodeData=true`（不删词）时，即使锚点逻辑有误也可能"凑合工作"，一旦上线改回默认 `false`（删词）行为会突变——务必按**上线配置 `false`** 做最终验证。
- 共视用 `compareTo()`（BoW 词匹配，量化级），廉价但粒度受词典影响；阈值的绝对值会随特征数/词典轻微漂移，跨数据集建议据实标定（可参考 `Mem/RehearsalSimilarity` 的取值，本阈值应取得比它更严格）。后续可升级为相对/分位数自适应阈值，或 TF-IDF 加权的稀有词覆盖率。
- 仅在**建图（incremental）模式**生效；定位模式不降级。
- **单锚点**（最后保留关键帧）是保守近似：只消减相对紧邻关键帧冗余的帧，不会因感知混叠误删远处真地点；代价是漏掉"对更早关键帧冗余"的帧（这些被安全保留）。多锚点（空间近邻并集 + 里程计空间闸门）可更激进，属后续工作。
- **决策时机**为节点 admission（建帧时），此时当前帧尚无回环边，因此本逻辑**只降级当前新帧、从不回溯删除过去节点** → 已携带回环/路标边的历史关键帧绝不会被误降。若想消减"事后才发现冗余"的历史节点，可在节点离开 STM（`moveSignatureToWMFromSTM`）时做回溯式剔除并加回环边保护，属后续工作。
- 默认关闭（`ratio=0`），需显式开启。

---

## 9. 实现与文件

| 文件 | 改动 |
|---|---|
| `corelib/include/rtabmap/core/Parameters.h` | 新增 `Mem/CovisibilityRedundancyRatio`、`Mem/CovisibilityMaxIntermediateNodes` |
| `corelib/include/rtabmap/core/Rtabmap.h` | 锚点状态成员（`_covisRedundancyRatio` / `_covisMaxIntermediateNodes` / `_lastKeyframeId` / `_consecutiveIntermediateNodes`） |
| `corelib/src/Rtabmap.cpp` | 参数解析、复位、`process()` 中的共视冗余判定 + 降级 + 锚点维护 |

> 说明：受沙箱依赖限制（缺 OpenCV/PCL/VTK），未做整库编译。所有 API（`Memory::getSignature/isIncremental/convertToIntermediate`、`Signature::getWords/getWeight`、参数宏访问器）已对照头文件核对，并与既有中间节点代码路径保持一致。请在配齐依赖的环境编译验证。

---

## 10. 参考思路

- 关键帧共视/冗余剔除：ORB-SLAM 的关键帧筛选与冗余关键帧剔除（基于地图点共视）。
- RTABMap 的内存管理与中间节点机制（weight、工作记忆/长期记忆转移）。

> 以上为对公开方法的概括性改写，非原文照搬。
