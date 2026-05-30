# XFeat learned features for loop closure (CPU)

> Script: `corelib/src/python/rtabmap_xfeat.py`
> Plugs into RTAB-Map's existing **PyDetector** interface (`Kp/DetectorStrategy=15`). No C++ changes.

## 1. What it does

Replaces the hand-crafted local feature (GFTT/ORB) used for loop-closure
feature matching (and for the BoW vocabulary) with **XFeat** ("Accelerated
Features", Potje et al., CVPR 2024) — a lightweight learned detector/descriptor
that runs in real time **on CPU**. Better, more viewpoint/illumination-robust
correspondences → more inliers → loops that BoW-quantized matching missed get
verified, and the loop transform is more accurate.

It outputs, per image, `N×3` keypoints `[x, y, response]` and `N×64`
L2-normalized float descriptors — exactly the format RTAB-Map's `PyDetector`
expects.

## 2. Install (CPU)

```bash
pip install "torch>=1.10" torchvision numpy opencv-python
```

The model (`xfeat.pt`, ~6 MB, Apache-2.0) is fetched **automatically** from the
official repo `verlab/accelerated_features` via `torch.hub` on first use and
cached under `~/.cache/torch/hub`.

**Offline device (e.g. robot with no internet):**
- Option A — pre-fetch on a connected machine, then copy the cache:
  ```bash
  python3 -c "import torch; torch.hub.load('verlab/accelerated_features','XFeat',pretrained=True,trust_repo=True)"
  # copy ~/.cache/torch/hub to the device's ~/.cache/torch/hub
  ```
- Option B — local clone + weights:
  ```bash
  git clone https://github.com/verlab/accelerated_features
  export RTABMAP_XFEAT_REPO=/path/to/accelerated_features
  export RTABMAP_XFEAT_WEIGHTS=/path/to/accelerated_features/weights/xfeat.pt
  ```

## 3. Enable in RTAB-Map (CPU, NN matching)

```
--Kp/DetectorStrategy 15 \
--Vis/FeatureType 15 \
--PyDetector/Path "/<path-to>/rtabmap/corelib/src/python/rtabmap_xfeat.py" \
--PyDetector/Cuda false \
--Vis/CorType 0 --Vis/CorNNType 1 --Kp/NNStrategy 1
```

- `DetectorStrategy=15` / `FeatureType=15` = PyDetector (this script) for both the
  vocabulary and the visual registration.
- `CorType=0` + `CorNNType=1` = features matching with FLANN KD-tree nearest
  neighbor on the 64-d descriptors (CPU). `CorNNType=3` (brute force) also works.
- `PyDetector/Cuda false` keeps it on CPU (the script also forces CPU by default;
  set env `RTABMAP_XFEAT_CUDA=1` only if you want GPU).

## 4. Tuning (X5 / CPU speed)

Environment variables (read by the script):

| Var | Default | Meaning |
|---|---|---|
| `RTABMAP_XFEAT_TOPK` | `2048` | Max features XFeat extracts. Lower (e.g. `1024`/`512`) = faster on CPU. |
| `RTABMAP_XFEAT_CUDA` | `0` | `1` to allow GPU (default CPU-only). |
| `RTABMAP_XFEAT_REPO` | - | Local clone of `verlab/accelerated_features` (offline). |
| `RTABMAP_XFEAT_WEIGHTS` | - | Local `xfeat.pt` path (offline). |

RTAB-Map then further caps features with `Kp/MaxFeatures` / `Vis/MaxFeatures`.

## 5. Scope (what this does and does not change)

- ✅ Upgrades the **feature extraction + matching** used for the loop-closure
  transform (and the BoW vocabulary descriptors).
- ❌ Does **not** replace the loop **detection** algorithm itself (still BoW +
  Bayes filter). A learned global place-recognition descriptor (VPR) would be a
  separate change via `GlobalDescriptorExtractor`.
- Downstream geometry (PnP with StereoNet depth, optional dense refinement) is
  unchanged and benefits from the better correspondences.

## 6. Validation

End-to-end on CPU (torch 2.8.0+cpu): `init()` loads XFeat on CPU; `detect()` on a
480×640 grayscale image returns `pts (2048, 3) float32` and `desc (2048, 64)
float32`, matching `PyDetector.cpp`'s expected contract (kpts `N×3` float32,
descriptors `N×D` float32, rows aligned). The pure I/O-formatting function is
also unit-tested for shape/dtype/contiguity, including the empty-detection case.

> Note: this validates the Python script + model on CPU. Building RTAB-Map
> itself requires its usual dependencies (OpenCV, PCL, pybind11) and Python
> support enabled (`WITH_PYTHON`).

## 7. Attribution / license

XFeat — "XFeat: Accelerated Features for Lightweight Image Matching", Potje,
Cadar, Araujo, Martins, Nascimento, CVPR 2024. Code & weights:
`https://github.com/verlab/accelerated_features` (Apache-2.0). The weights are
fetched from there at runtime and are not redistributed in this repository.
