def suppress_nested(dets, thresh=0.8):
    kept = []
    for d in sorted(dets, key=lambda d: d["conf"], reverse=True):
        x1, y1, x2, y2 = d["box"]
        area = max(x2 - x1, 0) * max(y2 - y1, 0)
        nested = False
        for k in kept:
            kx1, ky1, kx2, ky2 = k["box"]
            iw = min(x2, kx2) - max(x1, kx1)
            ih = min(y2, ky2) - max(y1, ky1)
            if iw <= 0 or ih <= 0:
                continue
            k_area = max(kx2 - kx1, 0) * max(ky2 - ky1, 0)
            if iw * ih / max(min(area, k_area), 1) > thresh:
                nested = True
                break
        if not nested:
            kept.append(d)
    return kept


def unique_path(path):
    from pathlib import Path

    path = Path(path)
    if not path.exists():
        return path
    n = 1
    while (path.with_name(f"{path.stem}_{n}{path.suffix}")).exists():
        n += 1
    return path.with_name(f"{path.stem}_{n}{path.suffix}")
