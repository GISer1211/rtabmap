#! /usr/bin/env python3
#
# XFeat ("Accelerated Features", Potje et al., CVPR 2024) local feature
# detector/descriptor for RTAB-Map's PyDetector interface, running on CPU.
#
# XFeat is a lightweight learned local feature that runs in real time on CPU,
# with accuracy comparable to (or better than) heavier deep features. Here it
# replaces the hand-crafted detector/descriptor (GFTT/ORB) used for loop
# closure feature matching (and for the BoW vocabulary).
#
# --------------------------------------------------------------------------
# Setup (CPU)
# --------------------------------------------------------------------------
#   pip install "torch>=1.10" torchvision numpy opencv-python
#
# The model is fetched automatically from the official Apache-2.0 repository
# (verlab/accelerated_features) via torch.hub on first use (needs internet
# once; it is then cached under ~/.cache/torch/hub). For an OFFLINE device:
#   1) On a machine with internet, run once:
#        python3 -c "import torch; torch.hub.load('verlab/accelerated_features','XFeat',pretrained=True)"
#      then copy ~/.cache/torch/hub to the device, OR
#   2) git clone https://github.com/verlab/accelerated_features and set:
#        export RTABMAP_XFEAT_REPO=/path/to/accelerated_features
#        export RTABMAP_XFEAT_WEIGHTS=/path/to/accelerated_features/weights/xfeat.pt
#
# --------------------------------------------------------------------------
# Use with RTAB-Map (CPU; features matching with nearest-neighbor)
# --------------------------------------------------------------------------
#   --Kp/DetectorStrategy 15 --Vis/FeatureType 15 \
#   --PyDetector/Path "<rtabmap>/corelib/src/python/rtabmap_xfeat.py" \
#   --PyDetector/Cuda false \
#   --Vis/CorType 0 --Vis/CorNNType 1 --Kp/NNStrategy 1
#
# Optional environment variables:
#   RTABMAP_XFEAT_TOPK     max features XFeat extracts (default 2048)
#   RTABMAP_XFEAT_CUDA     set to "1" to allow GPU (default CPU-only)
#   RTABMAP_XFEAT_REPO     local path to a clone of verlab/accelerated_features
#   RTABMAP_XFEAT_WEIGHTS  local path to xfeat.pt weights
#
# Returns to RTAB-Map (per detect() call):
#   pts : float32 array N x 3  -> [x, y, response]
#   desc: float32 array N x 64 -> XFeat descriptors (L2-normalized)
# --------------------------------------------------------------------------

import os

# Target CPU by default. This must happen BEFORE torch is imported so that
# torch.cuda.is_available() returns False and XFeat selects the CPU device.
if os.environ.get("RTABMAP_XFEAT_CUDA", "0") != "1":
    os.environ["CUDA_VISIBLE_DEVICES"] = ""

import numpy as np
import torch

torch.set_grad_enabled(False)

_xfeat = None
_device = "cpu"
_top_k = int(os.environ.get("RTABMAP_XFEAT_TOPK", "2048"))


def _load_xfeat(top_k):
    """Load an XFeat instance, trying a local clone first then torch.hub."""
    repo = os.environ.get("RTABMAP_XFEAT_REPO", "")
    weights = os.environ.get("RTABMAP_XFEAT_WEIGHTS", "")

    # 1) Local clone (offline-friendly): <repo> on sys.path, weights from file.
    if repo and os.path.isdir(repo):
        import sys
        if repo not in sys.path:
            sys.path.insert(0, repo)
        from modules.xfeat import XFeat  # noqa: E402
        if weights and os.path.isfile(weights):
            return XFeat(weights=weights, top_k=top_k)
        return XFeat(top_k=top_k)

    # 2) torch.hub (auto-downloads code + pretrained weights, cached locally).
    #    trust_repo=True avoids the interactive trust prompt on newer torch.
    return torch.hub.load(
        "verlab/accelerated_features", "XFeat",
        pretrained=True, top_k=top_k, trust_repo=True
    )


def init(cuda):
    """Called once by RTAB-Map's PyDetector. `cuda` is 1 (GPU) or 0 (CPU)."""
    global _xfeat, _device

    use_cuda = bool(cuda) and (os.environ.get("RTABMAP_XFEAT_CUDA", "0") == "1") \
        and torch.cuda.is_available()
    _device = torch.device("cuda" if use_cuda else "cpu")

    _xfeat = _load_xfeat(_top_k)

    # Force the selected device on the underlying network (robust across versions).
    try:
        _xfeat.dev = _device
        if hasattr(_xfeat, "net") and _xfeat.net is not None:
            _xfeat.net = _xfeat.net.to(_device)
    except Exception as e:  # best effort
        print("rtabmap_xfeat: warning, could not pin device: %s" % e)

    print("rtabmap_xfeat: XFeat initialized on %s (top_k=%d)" % (_device, _top_k))


def _format_output(keypoints, scores, descriptors):
    """Build RTAB-Map's expected (pts Nx3 float32, desc NxD float32) arrays.

    Kept as a pure function (no torch) so the I/O contract can be unit-tested.
      keypoints  : array-like (N, 2) -> [x, y]
      scores     : array-like (N,)
      descriptors: array-like (N, D)
    """
    keypoints = np.asarray(keypoints, dtype=np.float32).reshape(-1, 2)
    scores = np.asarray(scores, dtype=np.float32).reshape(-1, 1)
    descriptors = np.asarray(descriptors, dtype=np.float32)
    if descriptors.ndim == 1:
        descriptors = descriptors.reshape(keypoints.shape[0], -1)

    n = keypoints.shape[0]
    if n == 0:
        # Return well-shaped empty arrays so the C++ side handles them gracefully.
        dim = descriptors.shape[1] if descriptors.ndim == 2 and descriptors.shape[1] > 0 else 64
        return (np.zeros((0, 3), dtype=np.float32), np.zeros((0, dim), dtype=np.float32))

    pts = np.concatenate([keypoints, scores], axis=1).astype(np.float32)
    # contiguous copies so the C++ side reads correctly ordered memory
    return np.ascontiguousarray(pts), np.ascontiguousarray(descriptors.astype(np.float32))


def detect(imageBuffer):
    """Called by RTAB-Map per image. `imageBuffer` is an H x W uint8 grayscale array."""
    global _xfeat

    image = np.asarray(imageBuffer)
    if image.ndim != 2:
        image = image.reshape(image.shape[0], image.shape[1])

    # XFeat's detectAndCompute accepts a HxWxC uint8 numpy image (like cv2 images)
    # and handles permute + /255 + RGB->gray internally. Replicate gray to 3 channels.
    img3 = np.repeat(image[:, :, None], 3, axis=2)

    output = _xfeat.detectAndCompute(img3, top_k=_top_k)[0]

    keypoints = output["keypoints"].detach().cpu().numpy()      # (N, 2)
    scores = output["scores"].detach().cpu().numpy()            # (N,)
    descriptors = output["descriptors"].detach().cpu().numpy()  # (N, 64)

    return _format_output(keypoints, scores, descriptors)


if __name__ == "__main__":
    # Smoke test (requires torch + XFeat available).
    init(0)
    pts, desc = detect((np.random.rand(480, 640) * 255).astype(np.uint8))
    print("pts", pts.shape, pts.dtype, "desc", desc.shape, desc.dtype)
