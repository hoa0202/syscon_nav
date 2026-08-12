#!/usr/bin/env python3
"""
Teach & Repeat 주행 결과 일괄 분석
- teach 경로(paths/) vs 실제 주행(results/) CTE 비교
- 사용법:
    python3 analyze_results.py
    python3 analyze_results.py --teach paths/latest_path.csv --results results/
    python3 analyze_results.py --plot
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from glob import glob
from pathlib import Path


ROOT = Path(os.path.expanduser('~/roboro_nav'))
DEFAULT_TEACH = ROOT / 'paths' / 'latest_path.csv'
DEFAULT_RESULTS = ROOT / 'results'


def load_xy(path: Path):
    """CSV에서 (x,y[,t]) 로드. # 주석/헤더 스킵."""
    pts = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or line.lower().startswith('x,'):
                continue
            parts = line.split(',')
            if len(parts) < 2:
                continue
            x, y = float(parts[0]), float(parts[1])
            t = float(parts[2]) if len(parts) >= 3 else None
            pts.append((x, y, t))
    return pts


def path_length(pts):
    return sum(
        math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        for i in range(1, len(pts))
    )


def point_to_segment_dist(px, py, ax, ay, bx, by):
    """점 P와 세그먼트 AB 최근접 거리 + 부호있는 횡방향 오차(대략)."""
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-12:
        return math.hypot(apx, apy), 0.0
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab2))
    cx, cy = ax + t * abx, ay + t * aby
    dx, dy = px - cx, py - cy
    dist = math.hypot(dx, dy)
    # 부호: AB 기준 좌(+)/우(-)
    cross = abx * dy - aby * dx
    signed = dist if cross >= 0 else -dist
    return dist, signed


def cross_track_errors(actual, teach, stride=1):
    """각 actual 점에 대해 teach polyline 최근접 거리."""
    if len(teach) < 2 or len(actual) < 1:
        return [], []

    abs_err = []
    signed_err = []
    for i in range(0, len(actual), stride):
        px, py, _ = actual[i]
        best_d = float('inf')
        best_s = 0.0
        for j in range(len(teach) - 1):
            d, s = point_to_segment_dist(
                px, py,
                teach[j][0], teach[j][1],
                teach[j + 1][0], teach[j + 1][1],
            )
            if d < best_d:
                best_d = d
                best_s = s
        abs_err.append(best_d)
        signed_err.append(best_s)
    return abs_err, signed_err


def percentile(vals, p):
    if not vals:
        return float('nan')
    s = sorted(vals)
    k = (len(s) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return s[int(k)]
    return s[f] * (c - k) + s[c] * (k - f)


def analyze_run(actual_path: Path, teach, stride=1):
    actual = load_xy(actual_path)
    if len(actual) < 2:
        return None

    abs_err, signed_err = cross_track_errors(actual, teach, stride=stride)
    if not abs_err:
        return None

    duration = None
    if actual[0][2] is not None and actual[-1][2] is not None:
        duration = actual[-1][2] - actual[0][2]

    start_err = math.hypot(
        actual[0][0] - teach[0][0], actual[0][1] - teach[0][1])
    end_err = math.hypot(
        actual[-1][0] - teach[-1][0], actual[-1][1] - teach[-1][1])

    return {
        'file': actual_path.name,
        'path': str(actual_path),
        'points': len(actual),
        'length_m': path_length(actual),
        'duration_s': duration,
        'avg_m': sum(abs_err) / len(abs_err),
        'rmse_m': math.sqrt(sum(e * e for e in abs_err) / len(abs_err)),
        'max_m': max(abs_err),
        'p95_m': percentile(abs_err, 95),
        'mean_signed_m': sum(signed_err) / len(signed_err),
        'start_err_m': start_err,
        'end_err_m': end_err,
        'actual': actual,
        'abs_err': abs_err,
    }


def find_result_files(results_dir: Path):
    files = sorted(results_dir.glob('actual_path_*.csv'))
    # 심볼릭 링크 제외 (실파일만)
    return [f for f in files if f.is_file() and not f.is_symlink()]


def print_table(rows):
    headers = [
        ('#', 3), ('file', 32), ('pts', 5), ('len_m', 7),
        ('dur_s', 7), ('avg_m', 8), ('rmse_m', 8),
        ('max_m', 8), ('p95_m', 8), ('end_m', 8),
    ]
    line = ' '.join(h.ljust(w) if h == 'file' else h.rjust(w) for h, w in headers)
    print(line)
    print('-' * len(line))

    for i, r in enumerate(rows, 1):
        dur = f"{r['duration_s']:.1f}" if r['duration_s'] is not None else '-'
        vals = [
            str(i).rjust(3),
            r['file'][:32].ljust(32),
            str(r['points']).rjust(5),
            f"{r['length_m']:.2f}".rjust(7),
            dur.rjust(7),
            f"{r['avg_m']:.4f}".rjust(8),
            f"{r['rmse_m']:.4f}".rjust(8),
            f"{r['max_m']:.4f}".rjust(8),
            f"{r['p95_m']:.4f}".rjust(8),
            f"{r['end_err_m']:.4f}".rjust(8),
        ]
        print(' '.join(vals))


def save_summary_csv(rows, out_path: Path):
    fields = [
        'file', 'points', 'length_m', 'duration_s',
        'avg_m', 'rmse_m', 'max_m', 'p95_m',
        'mean_signed_m', 'start_err_m', 'end_err_m',
    ]
    with open(out_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow({k: r[k] for k in fields})
    print(f'\n요약 CSV: {out_path}')


def maybe_plot(rows, teach, out_path: Path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib 없음 — 플롯 스킵 (pip install matplotlib)')
        return

    n = len(rows)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # XY overlay
    ax = axes[0]
    tx = [p[0] for p in teach]
    ty = [p[1] for p in teach]
    ax.plot(tx, ty, 'k-', linewidth=2, label='teach', zorder=10)
    for r in rows:
        ax.plot(
            [p[0] for p in r['actual']],
            [p[1] for p in r['actual']],
            alpha=0.7, label=r['file'].replace('actual_path_', '')[:15],
        )
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Path overlay')
    ax.legend(fontsize=7, loc='best')

    # Error bars
    ax = axes[1]
    labels = [f"#{i}" for i in range(1, n + 1)]
    avgs = [r['avg_m'] * 100 for r in rows]
    maxs = [r['max_m'] * 100 for r in rows]
    x = range(n)
    ax.bar([i - 0.15 for i in x], avgs, width=0.3, label='avg')
    ax.bar([i + 0.15 for i in x], maxs, width=0.3, label='max')
    ax.set_xticks(list(x), labels)
    ax.set_ylabel('CTE [cm]')
    ax.set_title('Cross-track error')
    ax.grid(True, axis='y', alpha=0.3)
    ax.legend()

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f'플롯: {out_path}')


def main():
    ap = argparse.ArgumentParser(description='Teach&Repeat 결과 일괄 분석')
    ap.add_argument('--teach', type=Path, default=DEFAULT_TEACH,
                    help='teach 경로 CSV (default: paths/latest_path.csv)')
    ap.add_argument('--results', type=Path, default=DEFAULT_RESULTS,
                    help='주행 결과 폴더 (default: results/)')
    ap.add_argument('--plot', action='store_true', help='overlay/오차 플롯 저장')
    ap.add_argument('--stride', type=int, default=1,
                    help='오차 샘플 간격 (기본 1=전점)')
    args = ap.parse_args()

    teach_path = Path(os.path.expanduser(str(args.teach))).resolve()
    results_dir = Path(os.path.expanduser(str(args.results))).resolve()

    if teach_path.is_symlink():
        teach_path = Path(os.path.realpath(teach_path))

    if not teach_path.exists():
        print(f'❌ teach 경로 없음: {teach_path}', file=sys.stderr)
        sys.exit(1)

    results_dir.mkdir(parents=True, exist_ok=True)
    files = find_result_files(results_dir)
    if not files:
        print(f'❌ 결과 파일 없음: {results_dir}/actual_path_*.csv')
        sys.exit(1)

    teach = load_xy(teach_path)
    if len(teach) < 2:
        print(f'❌ teach 포인트 부족: {teach_path}', file=sys.stderr)
        sys.exit(1)

    print(f'Teach : {teach_path} ({len(teach)} pts, {path_length(teach):.2f}m)')
    print(f'Results: {results_dir} ({len(files)} runs)\n')

    rows = []
    for f in files:
        r = analyze_run(f, teach, stride=args.stride)
        if r:
            rows.append(r)
        else:
            print(f'⚠️ 스킵 (데이터 부족): {f.name}')

    if not rows:
        print('❌ 분석 가능한 결과가 없음')
        sys.exit(1)

    print_table(rows)

    n = len(rows)
    mean_avg = sum(r['avg_m'] for r in rows) / n
    mean_max = sum(r['max_m'] for r in rows) / n
    mean_rmse = sum(r['rmse_m'] for r in rows) / n
    peak_max = max(r['max_m'] for r in rows)
    best_avg = min(rows, key=lambda r: r['avg_m'])
    worst_avg = max(rows, key=lambda r: r['avg_m'])

    print('\n=== Overall ===')
    print(f'runs           : {n}')
    print(f'mean(avg CTE)  : {mean_avg:.4f} m  ({mean_avg*100:.2f} cm)')
    print(f'mean(rmse CTE) : {mean_rmse:.4f} m  ({mean_rmse*100:.2f} cm)')
    print(f'mean(max CTE)  : {mean_max:.4f} m  ({mean_max*100:.2f} cm)')
    print(f'peak max CTE   : {peak_max:.4f} m  ({peak_max*100:.2f} cm)')
    print(f'best avg       : {best_avg["avg_m"]:.4f} m  ({best_avg["file"]})')
    print(f'worst avg      : {worst_avg["avg_m"]:.4f} m  ({worst_avg["file"]})')

    summary_path = results_dir / 'summary.csv'
    save_summary_csv(rows, summary_path)

    if args.plot:
        maybe_plot(rows, teach, results_dir / 'analysis.png')


if __name__ == '__main__':
    main()
