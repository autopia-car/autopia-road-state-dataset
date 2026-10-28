"""Loader for ARSD labeled ROI image patches."""

import os
from pathlib import Path

import numpy as np

LABELS = ["even", "uneven", "manhole", "pothole", "speedbump"]


class ROIDataset:
    """Load labeled 240x360 image patches from the ROIs directory.

    The expected layout is::

        rois_root/
            even/        (150 122 images)
            uneven/      (126 921 images)
            manhole/     (  5 478 images)
            pothole/     (    573 images)
            speedbump/   (     68 images)

    Example::

        ds = ROIDataset("/path/to/ROIs")
        print(ds)                       # per-label counts
        img, label = ds[0]              # numpy BGR image, label string
        for img, label in ds.iter_label("pothole"):
            ...
    """

    def __init__(self, rois_root, labels=None):
        self.root = Path(rois_root)
        self.labels = labels or LABELS
        self._samples = []
        for label in self.labels:
            label_dir = self.root / label
            if not label_dir.is_dir():
                continue
            for fname in sorted(os.listdir(label_dir)):
                if fname.lower().endswith((".jpg", ".jpeg", ".png")):
                    self._samples.append((label_dir / fname, label))

    def __len__(self):
        return len(self._samples)

    def __getitem__(self, idx):
        import cv2

        path, label = self._samples[idx]
        img = cv2.imread(str(path), cv2.IMREAD_COLOR)
        return img, label

    def iter_label(self, label):
        """Yield *(image, label)* for every sample of a given label."""
        for path, lbl in self._samples:
            if lbl == label:
                import cv2

                yield cv2.imread(str(path), cv2.IMREAD_COLOR), lbl

    def label_counts(self):
        """Return dict mapping label -> count."""
        counts = {l: 0 for l in self.labels}
        for _, label in self._samples:
            counts[label] += 1
        return counts

    def __repr__(self):
        counts = self.label_counts()
        total = sum(counts.values())
        lines = [f"ROIDataset({self.root}, {total} samples)"]
        for label, count in counts.items():
            pct = 100 * count / total if total else 0
            lines.append(f"  {label:<12} {count:>8}  ({pct:5.1f}%)")
        return "\n".join(lines)
