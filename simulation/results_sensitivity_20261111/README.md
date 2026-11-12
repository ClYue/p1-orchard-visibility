# 敏感性实验数据索引（2026-11-11）

全部由 `../run_study.py` 生成，每组 60 条策略 × 30 seeds × 2 场景 = 3600 条样本。
`study.json` 里的 `horizon` / `pos_weight` / `speed` 字段标明该组的参数。

| 目录 | horizon | pos_weight | speed | 说明 |
|---|---|---|---|---|
| `horizon1/` | 1 s | 0.2 | 3.0 m/s | **基准组**，与 `../results/study.json` 逐位相同（母本，勿改） |
| `horizon2/` | 2 s | 0.2 | 3.0 m/s | 可达 6 m |
| `horizon3/` | 3 s | 0.2 | 3.0 m/s | 可达 9 m |
| `horizon4/` | 4 s | 0.2 | 3.0 m/s | 可达 12 m，全部六档半径可用 |
| `posw0.05/` | 1 s | 0.05 | 3.0 m/s | 位置项只占航向项 19% |
| `posw0.1/` | 1 s | 0.10 | 3.0 m/s | 39% |
| `posw0.4/` | 1 s | 0.40 | 3.0 m/s | 99%，两项等权 |
| `posw0.8/` | 1 s | 0.80 | 3.0 m/s | 179%，位置项主导 |
| `speed1.5/` | 1 s | 0.2 | 1.5 m/s | 可达 1.5 m |
| `speed6.0/` | 1 s | 0.2 | 6.0 m/s | 可达 6 m，与 `horizon2/` 同可达半径 |
| `k0.5/` | 1 s | 0.2 | 3.0 m/s | 消光系数 k=0.5 m⁻¹ |
| `k2.0/` | 1 s | 0.2 | 3.0 m/s | k=2.0 m⁻¹ |
| `k3.0/` | 1 s | 0.2 | 3.0 m/s | k=3.0 m⁻¹（`horizon1/` 即 k=1.0） |

## 复现

```bash
python run_study.py --horizon 3 --output results_sensitivity_20261111/horizon3
python run_study.py --pos-weight 0.4 --output results_sensitivity_20261111/posw0.4
python run_study.py --speed 6 --output results_sensitivity_20261111/speed6.0
python run_study.py --extinction 0.5 --output results_sensitivity_20261111/k0.5
```

单组约 22 秒。默认参数（`--horizon 1 --pos-weight 0.2 --speed 3`）必须与 `../results/study.json` 逐位一致，
这是改动的回归基准。

## 注意

- `horizon1/` 与 `speed6.0/` 的可达半径不同（3 m vs 6 m），**不是**重复实验。
  真正重叠的是 `speed6.0/`（1 s × 6 m/s = 6 m）与 `horizon2/`（2 s × 3 m/s = 6 m）：
  两者可达集相同，实测航向差 0.011–0.026°，说明可达半径是这两组变化的主因，
  但每步位移不同仍有次级效应。**不要把 E1 和 E3 当作两组独立证据。**
- `pos_weight` 只作用于自适应策略，48 个固定站跨 5 档逐位相同（max|Δ| = 0）。
- `speed` 会同时改变固定站轨迹（移动速度变了），跨 3 档 max|Δ| = 12.99 m（center）。
