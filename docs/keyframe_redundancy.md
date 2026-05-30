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

### 3.1 冗余判据：共视覆盖率

每生成一个新地图节点，计算它相对**最后保留的关键帧（锚点 anchor）**的**共视覆盖率**：

```
coverage = |words_current ∩ words_anchor| / |unique(words_current)|
```

- `words_*` 为 BoW 视觉词 ID 集合（RTABMap 在建签名时已算好，取唯一词 ID）。
- 含义：**当前帧所见内容中，有多大比例已经被锚点覆盖**。

`coverage > Mem/CovisibilityRedundancyRatio`（如 0.8）→ 判定为冗余。

### 3.2 为什么用"相对当前帧"的覆盖率（而非 shared/min）

这是同时实现 A（减冗余）与 B（回环价值保护）的关键：

- 用 `shared / |current|`，**带来大量新内容的帧覆盖率自然低 → 自动保留**；
- 独特地点（即使与锚点有大量重叠，但引入了新结构）不会被误删；
- 因此"距离区分性保护"是**内建**的，无需额外参数。

### 3.3 锚点 = 最后保留的关键帧（自限制）

覆盖率始终对**最后一个被保留的关键帧**计算，而不是上一帧：

- 只要当前视野仍被锚点覆盖（coverage 高），就持续降级；
- 一旦相对锚点的场景变化够大（coverage 跌破阈值），保留当前帧为新关键帧并**更新锚点**；
- 这天然**限制了降级链的长度**——降级的每一帧都相对同一个"仍然覆盖它"的锚点冗余。

### 3.4 降级动作

冗余帧调用 `Memory::convertToIntermediate()`：

- `weight = -1` → 被回环检测统一跳过；
- `disableWordsRef()` → 从词典索引移除（不再参与回环似然）；
- 释放图像/原始数据；
- **不修改任何链接** → 里程计/邻接边保留，位姿图约束完整。

这与 RTABMap 既有的 `Rtabmap/CreateIntermediateNodes` 中间节点是同一套机制，只是触发条件从"检测频率"换成"共视覆盖率"。

### 3.5 安全网

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

## 6. 为什么不伤回环、且可能提升

- **召回不降**：冗余帧（>阈值被锚点覆盖）所在地点已被锚点覆盖；该帧能形成的回环，锚点同样能形成。
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
- 可与 `RGBD/LinearUpdate`、`Mem/RehearsalSimilarity` 同时使用，三者互补（距离 / 相邻外观 / 全局共视）。
- 若同时用 `Rtabmap/CreateIntermediateNodes`，注意 `Mem/RehearsalIdUpdatedToNewOne` 应保持 false（与中间节点相关的既有约束）。

---

## 8. 已知限制

- 共视用 **BoW 词 ID 交集**（量化级），廉价但粒度受词典影响；可后续升级为 **TF-IDF 加权的稀有词覆盖率**（更强调独特性）。
- 仅在**建图（incremental）模式**生效；定位模式不降级。
- 锚点为"最后保留的关键帧"，是单锚点近似；多锚点（图邻域并集覆盖）会更严格，但更复杂，本版未实现。
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
