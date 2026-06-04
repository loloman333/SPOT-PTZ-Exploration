#!/usr/bin/env python3
"""
Offline plotter for collector CSV data.

Supports single CSV and batch folder modes with unified axes. Generates individual
time-series plots and a summary figure with alternating y-axes. In batch mode, also
generates a final-value bar comparison chart across all runs.
"""

import argparse
import csv
import math
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory
from matplotlib.ticker import FuncFormatter, LogLocator

# Plot configuration
PLOT_DPI = 180
SINGLE_PLOT_WIDTH = 9.0
SINGLE_PLOT_HEIGHT = 4.5
SUMMARY_PLOT_WIDTH = 14.0
SUMMARY_PLOT_HEIGHT = 10.0
COMPARISON_PLOT_WIDTH_PER_METRIC = 4.2
COMPARISON_PLOT_HEIGHT = 4.8
COMPARISON_LINE_STYLES = ('-', '--', ':', '-.')

# Data padding and spacing
AXIS_PADDING_PERCENT = 0.05
AXIS_PADDING_MIN = 0.1
BAR_VALUE_PADDING_PERCENT = 0.12
BAR_VALUE_PADDING_MIN = 0.15

# Font sizes
FONT_SIZE_SUPTITLE = 20
FONT_SIZE_LABEL = 16
FONT_SIZE_LEGEND = 14
FONT_SIZE_TICK = 14
FONT_SIZE_BAR_VALUE = 14

# Metric definitions (config name -> CSV field, label, color)
METRIC_SPECS = {
    'time': ('elapsed_s', 'Time passed [s]', 'tab:gray'),
    'distance': ('distance_m_cum', 'Distance traveled [m]', 'tab:blue'),
    'rotation': ('rotation_rad_cum', 'Rotation [rad]', 'tab:green'),
    'battery': ('battery_discharge', 'Battery Discharge [%]', 'tab:brown'),
    'area': ('projected_map_known_area_m2', 'Explored Area [m²]', 'tab:orange'),
    'landmarks': ('landmarks_count', 'Landmark Count', 'tab:red'),
    'confidence': ('mean_confidence', 'Mean Detection Confidence [%]', 'tab:purple'),
    'uncertainty': ('mean_covariance_trace', 'Mean Covariance Trace [m²]', 'tab:pink'),
    'error': ('geometric_error', 'Mean Geometric Error [m]', 'tab:brown'),
}

DEFAULT_PLOTS_CONFIG_PATH = Path(get_package_share_directory('ptz_exploration_core')) / 'config' / 'plots.yaml'

REQUIRED_CSV_COLUMNS = [
    'elapsed_s',
    'distance_m_cum',
    'rotation_rad_cum',
    'landmarks_count',
    'mean_confidence',
    'projected_map_known_area_m2',
]

OPTIONAL_CSV_COLUMNS = [
    'mean_covariance_trace',
    'geometric_error',
    'battery_percentage',
]


def load_plot_groups(config_path: Path) -> Dict[str, object]:
    """Load grouped plot definitions and settings from YAML."""
    with config_path.open('r', newline='') as config_file:
        config = yaml.safe_load(config_file) or {}

    plot_groups: Dict[str, object] = {}
    section_names = {
        'per_run': ('per_run',),
        'bar_comparison': ('bar_comparison',),
        'chart_comparison': ('chart_comparison',),
    }

    for canonical_name, aliases in section_names.items():
        section = next((config.get(alias) for alias in aliases if isinstance(config.get(alias), dict)), {})
        raw_groups = section.get('plots', []) if isinstance(section, dict) else []
        if not isinstance(raw_groups, list):
            raise ValueError(f"'{canonical_name}.plots' must be a list")

        normalized_groups: List[List[str]] = []
        for group in raw_groups:
            if not isinstance(group, list):
                raise ValueError(f"Each group in '{canonical_name}.plots' must be a list of metric names")
            normalized_groups.append([str(metric_name) for metric_name in group])

        plot_groups[canonical_name] = normalized_groups

        raw_settings = section.get('settings', {}) if isinstance(section, dict) else {}
        settings = raw_settings if isinstance(raw_settings, dict) else {}
        plot_groups[f'{canonical_name}_settings'] = {
            'sync_x_axis': bool(settings.get('sync_x_axis', False)),
            'sync_y_axis': bool(settings.get('sync_y_axis', False)),
        }

    return plot_groups


def _resolve_metric_spec(metric_name: str) -> Optional[Tuple[str, str, str]]:
    """Map a config metric name to the CSV field, display label, and color."""
    return METRIC_SPECS.get(metric_name)


def _group_has_visible_data(rows: List[Dict[str, float]], metric_names: List[str]) -> bool:
    for metric_name in metric_names:
        spec = _resolve_metric_spec(metric_name)
        if not spec:
            continue
        metric_key, _, _ = spec
        if any(math.isfinite(float(row.get(metric_key, math.nan))) for row in rows):
            return True
    return False


def _group_slug(metric_names: List[str]) -> str:
    """Create a stable filename-safe slug from a metric group."""
    cleaned = []
    for metric_name in metric_names:
        safe = ''.join(ch if ch.isalnum() or ch == '_' else '_' for ch in str(metric_name).strip().lower())
        if safe:
            cleaned.append(safe)
    return '_'.join(cleaned) if cleaned else 'group'


def _metric_specs_for_run(rows: List[Dict[str, float]], run_stem: Optional[str] = None):
    """Return metric specs to plot for a run.

    Include a metric only if there is at least one finite value in the run
    and the *last* row contains a finite value for that metric. This avoids
    deciding based on the CSV filename.
    """
    specs = []
    if not rows:
        return specs

    last_row = rows[-1]
    for metric_name in METRIC_SPECS:
        spec = _resolve_metric_spec(metric_name)
        if spec is None:
            continue
        metric_key, ylabel, color = spec

        # Skip metrics with no finite values at all in the run.
        if not any(math.isfinite(float(r.get(metric_key, math.nan))) for r in rows):
            continue

        # Only include the metric if the last row contains a finite value.
        last_val = float(last_row.get(metric_key, math.nan))
        if not math.isfinite(last_val):
            continue

        specs.append((metric_key, ylabel, color))

    return specs


def _to_float(value: str, default: float = 0.0) -> float:
    """Convert value to float with fallback to default on error."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def load_collector_csv(csv_path: Path) -> List[Dict[str, float]]:
    """Load and validate collector CSV, converting all fields to float."""
    rows: List[Dict[str, float]] = []
    with csv_path.open('r', newline='') as f:
        reader = csv.DictReader(f)

        if reader.fieldnames is None:
            raise ValueError('CSV has no header.')

        missing = [c for c in REQUIRED_CSV_COLUMNS if c not in reader.fieldnames]
        if missing:
            raise ValueError(f'Missing required columns: {missing}')

        for row in reader:
            rows.append({
                'elapsed_s': _to_float(row.get('elapsed_s')),
                'distance_m_cum': _to_float(row.get('distance_m_cum')),
                'rotation_rad_cum': _to_float(row.get('rotation_rad_cum')),
                'landmarks_count': _to_float(row.get('landmarks_count')),
                'mean_confidence': _to_float(row.get('mean_confidence')),
                'mean_covariance_trace': _to_float(row.get('mean_covariance_trace'), math.nan),
                'projected_map_known_area_m2': _to_float(row.get('projected_map_known_area_m2')),
                'geometric_error': _to_float(row.get('geometric_error'), math.nan),
                'battery_percentage': _to_float(row.get('battery_percentage'), math.nan),
            })

    if not rows:
        raise ValueError('CSV has no data rows.')

    _add_battery_discharge(rows)

    return rows


def _add_battery_discharge(rows: List[Dict[str, float]]) -> None:
    """Add battery discharge since start as a derived metric in percentage points."""
    baseline = next((row['battery_percentage'] for row in rows if math.isfinite(row.get('battery_percentage', math.nan))), math.nan)
    for row in rows:
        battery_percentage = row.get('battery_percentage', math.nan)
        if math.isfinite(baseline) and math.isfinite(battery_percentage):
            row['battery_discharge'] = max(0.0, baseline - battery_percentage)
        else:
            row['battery_discharge'] = math.nan


def normalize_elapsed_time(rows: List[Dict[str, float]]) -> List[Dict[str, float]]:
    """Sort rows by elapsed time and rebase so each run starts at t=0."""
    sorted_rows = sorted(rows, key=lambda row: row['elapsed_s'])
    if not sorted_rows:
        return sorted_rows

    t0 = sorted_rows[0]['elapsed_s']
    for row in sorted_rows:
        row['elapsed_s'] = max(0.0, row['elapsed_s'] - t0)

    return sorted_rows


def save_single_plot(
    plt,
    t: List[float],
    y: List[float],
    title: str,
    ylabel: str,
    output_path: Path,
    ylim: Optional[Tuple[float, float]] = None,
    xlim: Optional[Tuple[float, float]] = None,
    color: Optional[str] = None,
) -> None:
    """Create and save a single line plot."""
    fig, ax = plt.subplots(figsize=(SINGLE_PLOT_WIDTH, SINGLE_PLOT_HEIGHT))
    ax.plot(t, y, linewidth=2, color=color)
    ax.set_title(title, fontsize=FONT_SIZE_LABEL)
    ax.set_xlabel('time [s]', fontsize=FONT_SIZE_LABEL)
    ax.set_ylabel(ylabel, fontsize=FONT_SIZE_LABEL)
    ax.grid(True, alpha=0.35)
    ax.tick_params(axis='both', labelsize=FONT_SIZE_TICK)
    if ylim:
        ax.set_ylim(ylim)
    if xlim:
        ax.set_xlim(xlim)
    fig.tight_layout()
    fig.savefig(output_path, dpi=PLOT_DPI)
    plt.close(fig)


def main() -> None:
    """Entry point for plotter CLI."""
    parser = argparse.ArgumentParser(
        description='Create time-series plots from collector CSV output. '
        'Pass a single CSV file or a folder to batch-process multiple CSVs with unified axes.'
    )
    parser.add_argument(
        'input',
        type=Path,
        help='Path to collector CSV file or folder containing CSV files.',
    )
    parser.add_argument(
        '--out-dir',
        type=Path,
        default=None,
        help='Directory for generated plots (default: <input_dir>/plots or <csv_dir>/plots_<csv_stem> for single file).',
    )
    parser.add_argument(
        '--plots-config',
        type=Path,
        default=DEFAULT_PLOTS_CONFIG_PATH,
        help=f'Path to plot grouping config YAML (default: {DEFAULT_PLOTS_CONFIG_PATH}).',
    )
    args = parser.parse_args()

    input_path = args.input.expanduser().resolve()
    plots_config_path = args.plots_config.expanduser().resolve()
    plot_groups = load_plot_groups(plots_config_path)

    if input_path.is_file():
        _process_single_csv(input_path, args.out_dir, plot_groups)
    elif input_path.is_dir():
        _process_folder(input_path, args.out_dir, plot_groups)
    else:
        raise FileNotFoundError(f'Input path not found: {input_path}')


def _process_single_csv(csv_path: Path, out_dir_arg: Optional[Path] = None, plot_groups: Optional[Dict[str, object]] = None) -> None:
    """Process a single CSV file and generate plots."""
    if not csv_path.exists():
        raise FileNotFoundError(f'CSV file not found: {csv_path}')

    out_dir = (
        out_dir_arg.expanduser().resolve()
        if out_dir_arg
        else (csv_path.parent / f'plots_{csv_path.stem}')
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = normalize_elapsed_time(load_collector_csv(csv_path))
    t = [r['elapsed_s'] for r in rows]

    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise ImportError(
            'matplotlib is required for plotting. Install it (e.g. pip install matplotlib).'
        ) from exc

    _generate_per_run_figures(plt, t, rows, out_dir, csv_path.stem, plot_groups or {})


def _compute_axis_limits(all_data: Dict[str, Dict], metric_keys: List[str]) -> Dict[str, Tuple[float, float]]:
    """Compute global min/max axis limits with padding for each metric."""
    axis_limits: Dict[str, Tuple[float, float]] = {}

    for metric in metric_keys:
        all_values = []
        for data in all_data.values():
            all_values.extend([float(r.get(metric, math.nan)) for r in data['rows'] if math.isfinite(float(r.get(metric, math.nan)))])

        if all_values:
            vmin = min(all_values)
            vmax = max(all_values)
            padding = (vmax - vmin) * AXIS_PADDING_PERCENT if vmax > vmin else AXIS_PADDING_MIN
            axis_limits[metric] = (vmin - padding, vmax + padding)

    return axis_limits


def _collect_metric_fields(plot_groups: List[List[str]]) -> List[str]:
    """Collect unique CSV fields referenced by a list of grouped metric names."""
    metric_fields: List[str] = []
    seen_fields = set()

    for group in plot_groups:
        for metric_name in group:
            spec = _resolve_metric_spec(metric_name)
            if spec is None:
                continue
            metric_field, _, _ = spec
            if metric_field not in seen_fields:
                seen_fields.add(metric_field)
                metric_fields.append(metric_field)

    return metric_fields


def _metric_series(rows: List[Dict[str, float]], metric_key: str) -> List[float]:
    """Return a metric series while treating zero as missing for selected metrics."""
    zero_is_missing = metric_key in {'mean_covariance_trace', 'mean_confidence'}
    series: List[float] = []

    for row in rows:
        value = float(row.get(metric_key, math.nan))
        if zero_is_missing and math.isfinite(value) and value == 0.0:
            series.append(math.nan)
        else:
            series.append(value)

    return series


def _log_axis_limits(values: List[float]) -> Optional[Tuple[float, float]]:
    """Compute tidy positive log-axis limits from finite metric values."""
    positive_values = [value for value in values if math.isfinite(value) and value > 0.0]
    if not positive_values:
        return None

    lower = 10 ** math.floor(math.log10(min(positive_values)))
    upper = 10 ** math.ceil(math.log10(max(positive_values)))
    if lower == upper:
        lower /= 10.0
        upper *= 10.0

    return lower, upper


def _apply_log_yaxis(ax, values: List[float], axis_limit: Optional[Tuple[float, float]] = None) -> None:
    """Apply log scaling to a y-axis while keeping tick labels as plain numbers."""
    limits = axis_limit
    if not limits or limits[0] <= 0.0 or limits[1] <= 0.0:
        limits = _log_axis_limits(values)

    if not limits:
        return

    lower, upper = limits
    ax.set_yscale('log')
    ax.set_ylim(lower, upper)
    ax.yaxis.set_major_locator(LogLocator(base=10.0))
    ax.yaxis.set_major_formatter(FuncFormatter(lambda value, _: f'{value:g}'))


def _apply_percentage_yaxis(ax, values: List[float], axis_limit: Optional[Tuple[float, float]] = None) -> None:
    """Apply percentage formatting to a y-axis while keeping values in fractional units."""
    finite_values = [value for value in values if math.isfinite(value)]
    if not finite_values:
        return

    lower = 0.5
    upper = max(finite_values)
    if axis_limit:
        lower = max(lower, axis_limit[0])
        upper = max(upper, axis_limit[1])

    ax.set_ylim(lower, upper)
    ax.yaxis.set_major_formatter(FuncFormatter(lambda value, _: f'{value * 100:.0f}'))


def _resolve_visible_metrics(groups: List[List[str]], rows: List[Dict[str, float]]) -> List[Tuple[str, str, str]]:
    """Resolve plot metrics that have at least one finite value in the provided rows."""
    visible_metrics: List[Tuple[str, str, str]] = []

    for group in groups:
        if not isinstance(group, list):
            continue
        for metric_name in group:
            spec = _resolve_metric_spec(metric_name)
            if spec is None:
                continue
            metric_key, metric_label, color = spec
            metric_values = _metric_series(rows, metric_key)
            if any(math.isfinite(value) for value in metric_values):
                visible_metrics.append((metric_key, metric_label, color))

    return visible_metrics


def _resolve_comparison_visible_metrics(all_data: Dict[str, Dict], groups: List[List[str]]) -> List[Tuple[str, str, str]]:
    """Resolve comparison metrics that have at least one finite value across all runs."""
    visible_metrics: List[Tuple[str, str, str]] = []

    for group in groups:
        if not isinstance(group, list):
            continue
        for metric_name in group:
            spec = _resolve_metric_spec(metric_name)
            if spec is None:
                continue
            metric_key, metric_label, color = spec
            if any(
                math.isfinite(value)
                for data in all_data.values()
                for value in _metric_series(data['rows'], metric_key)
            ):
                visible_metrics.append((metric_key, metric_label, color))

    return visible_metrics


def _render_grouped_timeseries_figure(
    plt,
    t: List[float],
    rows: List[Dict[str, float]],
    groups: List[List[str]],
    output_path: Path,
    title: str,
    axis_limits: Optional[Dict[str, Tuple[float, float]]] = None,
    xlim: Optional[Tuple[float, float]] = None,
    run_stem: Optional[str] = None,
) -> None:
    """Render a grouped time-series figure with vertically stacked metric panels."""
    if not groups:
        return

    visible_metrics = _resolve_visible_metrics(groups, rows)

    if not visible_metrics:
        return

    fig, axes = plt.subplots(
        len(visible_metrics),
        1,
        figsize=(SUMMARY_PLOT_WIDTH, len(visible_metrics) * SINGLE_PLOT_HEIGHT * 0.5),
        squeeze=True,
        sharex=True,
    )
    if len(visible_metrics) == 1:
        axes = [axes]

    for row_index, (metric_key, metric_label, color) in enumerate(visible_metrics):
        ax = axes[row_index]
        metric_values = _metric_series(rows, metric_key)
        ax.plot(t, metric_values, linewidth=1.8, color=color)
        ax.set_ylabel(metric_label, fontsize=FONT_SIZE_LABEL, color=color, weight='bold')
        ax.grid(True, alpha=0.35, axis='y')
        if row_index % 2 == 1:
            ax.yaxis.tick_right()
            ax.yaxis.set_label_position('right')

        if row_index == len(visible_metrics) - 1:
            ax.set_xlabel('time [s]', fontsize=FONT_SIZE_LABEL)

    fig.suptitle(title, fontsize=FONT_SIZE_SUPTITLE)
    fig.subplots_adjust(hspace=0.0, left=0.08, right=0.92, top=1.15, bottom=0.08)
    fig.savefig(output_path, dpi=PLOT_DPI, bbox_inches='tight')
    plt.close(fig)


def _render_overlaid_timeseries_figure(
    plt,
    all_data: Dict[str, Dict],
    groups: List[List[str]],
    output_path: Path,
    title: str,
    axis_limits: Optional[Dict[str, Tuple[float, float]]] = None,
    xlim: Optional[Tuple[float, float]] = None,
) -> None:
    """Render grouped time-series figures with all runs overlaid per metric."""
    if not groups:
        return

    visible_metrics = _resolve_comparison_visible_metrics(all_data, groups)
    if not visible_metrics:
        return

    fig, axes = plt.subplots(
        len(visible_metrics),
        1,
        figsize=(SUMMARY_PLOT_WIDTH, len(visible_metrics) * SINGLE_PLOT_HEIGHT * 0.5),
        squeeze=True,
        sharex=True,
    )
    if len(visible_metrics) == 1:
        axes = [axes]

    run_items = list(all_data.items())
    global_end_time = xlim[1] if xlim else max(
        (max(data['t']) for _, data in run_items if data.get('t')),
        default=0.0,
    )
    legend_handles = []
    legend_labels = []

    from matplotlib.lines import Line2D

    for run_index, (run_name, _) in enumerate(run_items):
        line_style = COMPARISON_LINE_STYLES[run_index % len(COMPARISON_LINE_STYLES)]
        legend_handles.append(Line2D([0], [0], color='black', linewidth=1.8, linestyle=line_style))
        legend_labels.append(re.sub(r'^\d+_', '', run_name))

    for row_index, (metric_key, metric_label, color) in enumerate(visible_metrics):
        ax = axes[row_index]
        for run_index, (run_name, data) in enumerate(run_items):
            metric_values = _metric_series(data['rows'], metric_key)
            if not any(math.isfinite(value) for value in metric_values):
                continue

            series_t = list(data['t'])
            if not series_t:
                continue

            finite_indices = [idx for idx, value in enumerate(metric_values) if math.isfinite(value)]
            if not finite_indices:
                continue

            last_finite_index = finite_indices[-1]
            plot_t = series_t[:last_finite_index + 1]
            plot_y = metric_values[:last_finite_index + 1]
            last_finite_value = plot_y[-1]

            if plot_t[-1] < global_end_time:
                plot_t.append(global_end_time)
                plot_y.append(last_finite_value)

            line_style = COMPARISON_LINE_STYLES[run_index % len(COMPARISON_LINE_STYLES)]
            ax.plot(
                plot_t,
                plot_y,
                linewidth=1.8,
                color=color,
                linestyle=line_style,
                label=run_name,
            )

        ax.set_ylabel(metric_label, fontsize=FONT_SIZE_LABEL, color=color, weight='bold')
        ax.grid(True, alpha=0.35, axis='y')
        ax.tick_params(axis='both', labelsize=FONT_SIZE_TICK)
        if metric_key == 'mean_covariance_trace':
            covariance_values = [
                value
                for _, data in run_items
                for value in _metric_series(data['rows'], metric_key)
            ]
            _apply_log_yaxis(ax, covariance_values, axis_limits.get(metric_key) if axis_limits else None)
        elif metric_key == 'mean_confidence':
            confidence_values = [
                value
                for _, data in run_items
                for value in _metric_series(data['rows'], metric_key)
            ]
            _apply_percentage_yaxis(ax, confidence_values, axis_limits.get(metric_key) if axis_limits else None)
        elif axis_limits and metric_key in axis_limits:
            ax.set_ylim(axis_limits[metric_key])
        if xlim:
            ax.set_xlim(xlim)

        if row_index % 2 == 1:
            ax.yaxis.tick_right()
            ax.yaxis.set_label_position('right')

        if row_index == len(visible_metrics) - 1:
            ax.set_xlabel('time [s]', fontsize=FONT_SIZE_LABEL)

    if legend_handles:
        fig.legend(
            legend_handles,
            legend_labels,
            loc='upper center',
            bbox_to_anchor=(0.5, 0.99),
            ncol=min(4, len(legend_handles)),
            frameon=False,
            fontsize=FONT_SIZE_LEGEND,
        )

    if title:
        fig.suptitle(title, fontsize=FONT_SIZE_SUPTITLE)
        fig.subplots_adjust(hspace=0.0, left=0.08, right=0.92, top=0.92, bottom=0.08)
    else:
        fig.subplots_adjust(hspace=0.0, left=0.08, right=0.92, top=0.96, bottom=0.0)

    fig.savefig(output_path, dpi=PLOT_DPI, bbox_inches='tight')
    plt.close(fig)


def _render_grouped_comparison_figure(
    plt,
    all_data: Dict[str, Dict],
    groups: List[List[str]],
    output_path: Path,
    title: str,
) -> None:
    """Render grouped bar charts for final values across multiple runs."""
    if not groups:
        return

    visible_metrics: List[Tuple[str, str, str, List[Tuple[str, float]]]] = []
    for group in groups:
        if not isinstance(group, list):
            continue
        for metric_name in group:
            spec = _resolve_metric_spec(metric_name)
            if spec is None:
                continue

            metric_key, metric_label, color = spec
            # Use last-row presence to decide which runs provide a valid final
            # value for this metric (do not rely on run name prefixes).
            valid_items = [
                (run_name, float(data['rows'][-1].get(metric_key, math.nan)), data.get('subfolder'))
                for run_name, data in all_data.items()
                if math.isfinite(float(data['rows'][-1].get(metric_key, math.nan)))
            ]

            if valid_items:
                visible_metrics.append((metric_key, metric_label, color, valid_items))

    if not visible_metrics:
        return

    n_metrics = len(visible_metrics)
    if n_metrics > 3:
        cols = 2
        rows = math.ceil(n_metrics / cols)
    else:
        cols = n_metrics
        rows = 1

    fig, axes = plt.subplots(
        rows,
        cols,
        figsize=(COMPARISON_PLOT_WIDTH_PER_METRIC * cols, COMPARISON_PLOT_HEIGHT * rows),
        squeeze=False,
    )
    axes_flat = axes.flatten()

    for ax, (metric_key, metric_label, color, valid_items) in zip(axes_flat, visible_metrics):
        # Group by subfolder if present
        grouped_items = {}
        for item in valid_items:
            # item is (run_name, value, subfolder)
            run_name, value, subfolder = item
            group_key = subfolder if subfolder else run_name
            # remove number_ prefix (e.g. "12_folder" -> "folder")
            display_name = re.sub(r'^\d+_', '', group_key)
            if display_name not in grouped_items:
                grouped_items[display_name] = []
            grouped_items[display_name].append(value)
            
        metric_group_names = list(grouped_items.keys())
        metric_final_values = [sum(vals)/len(vals) for vals in grouped_items.values()]
        positions = list(range(len(metric_group_names)))

        ax.bar(
            positions,
            metric_final_values,
            color=color,
            width=0.8,
            edgecolor='black',
            linewidth=0.8,
            align='center',
        )
        ax.set_title(metric_label, fontsize=FONT_SIZE_LABEL)
        ax.grid(False)
        ax.margins(x=0.05)
        ax.set_xlim(-0.5, max(len(metric_group_names) - 0.5, 0.5))

        for xpos, value in zip(positions, metric_final_values):
            value_str = f'{value:.2f}'
            ax.text(
                xpos,
                value,
                value_str,
                ha='center',
                va='bottom',
                fontsize=FONT_SIZE_BAR_VALUE,
                color='black',
            )

        vmax = max(metric_final_values) if metric_final_values else 0.0
        top_padding = max(vmax * BAR_VALUE_PADDING_PERCENT, BAR_VALUE_PADDING_MIN)
        ax.set_ylim(0, vmax + top_padding)

        ax.set_yticks([])
        ax.spines['left'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.set_xticks(positions)
        ax.set_xticklabels(metric_group_names, rotation=45, ha='right', fontsize=FONT_SIZE_TICK)

    for i in range(n_metrics, len(axes_flat)):
        fig.delaxes(axes_flat[i])

    fig.suptitle(title, fontsize=14)
    fig.subplots_adjust(wspace=0.3, hspace=1.2, left=0.08, right=0.95, bottom=0.22, top=0.88)
    fig.savefig(output_path, dpi=PLOT_DPI, bbox_inches='tight')
    plt.close(fig)


def _comparison_group_has_visible_data(all_data: Dict[str, Dict], metric_name: str) -> bool:
    spec = _resolve_metric_spec(metric_name)
    if spec is None:
        return False

    metric_key, _, _ = spec
    if metric_key == 'geometric_error':
        return any(
            run_name.startswith('run_sim_') and math.isfinite(float(data['rows'][-1].get(metric_key, math.nan)))
            for run_name, data in all_data.items()
        )

    return any(math.isfinite(float(data['rows'][-1].get(metric_key, math.nan))) for data in all_data.values())


def _comparison_group_has_visible_timeseries_data(all_data: Dict[str, Dict], metric_name: str) -> bool:
    spec = _resolve_metric_spec(metric_name)
    if spec is None:
        return False

    metric_key, _, _ = spec
    return any(
        math.isfinite(value)
        for data in all_data.values()
        for value in _metric_series(data['rows'], metric_key)
    )


def _generate_per_run_figures(
    plt,
    t: List[float],
    rows: List[Dict[str, float]],
    out_dir: Path,
    name_stem: str,
    plot_groups: Dict[str, object],
    axis_limits: Optional[Dict[str, Tuple[float, float]]] = None,
    xlim: Optional[Tuple[float, float]] = None,
) -> None:
    groups = plot_groups.get('per_run', [])
    if not isinstance(groups, list):
        return
    if not groups:
        return

    for group_index, group in enumerate(groups, start=1):
        if not isinstance(group, list) or not group:
            continue

        if not _group_has_visible_data(rows, group):
            continue

        slug = _group_slug(group)
        output_path = out_dir / f'{name_stem}_per_run_{group_index:02d}_{slug}.png'
        _render_grouped_timeseries_figure(
            plt,
            t,
            rows,
            [group],
            output_path,
            '',
            axis_limits=axis_limits,
            xlim=xlim,
            run_stem=name_stem,
        )


def _generate_comparison_figure(
    plt,
    all_data: Dict[str, Dict],
    out_dir: Path,
    plot_groups: Dict[str, object],
    chart_axis_limits: Optional[Dict[str, Tuple[float, float]]] = None,
    chart_xlim: Optional[Tuple[float, float]] = None,
) -> None:
    bar_groups = plot_groups.get('bar_comparison', [])
    if isinstance(bar_groups, list) and bar_groups:
        for group_index, group in enumerate(bar_groups, start=1):
            if not isinstance(group, list) or not group:
                continue

            if not any(_comparison_group_has_visible_data(all_data, metric_name) for metric_name in group):
                continue

            slug = _group_slug(group)
            output_path = out_dir / f'bar_comparison_{group_index:02d}_{slug}.png'
            _render_grouped_comparison_figure(
                plt,
                all_data,
                [group],
                output_path,
                '',
            )

    chart_groups = plot_groups.get('chart_comparison', [])
    if not isinstance(chart_groups, list):
        return
    if not chart_groups:
        return

    for group_index, group in enumerate(chart_groups, start=1):
        if not isinstance(group, list) or not group:
            continue

        if not any(_comparison_group_has_visible_timeseries_data(all_data, metric_name) for metric_name in group):
            continue

        slug = _group_slug(group)
        output_path = out_dir / f'chart_comparison_{group_index:02d}_{slug}.png'
        _render_overlaid_timeseries_figure(
            plt,
            all_data,
            [group],
            output_path,
            '',
            axis_limits=chart_axis_limits,
            xlim=chart_xlim,
        )


def _process_folder(folder_path: Path, out_dir_arg: Optional[Path] = None, plot_groups: Optional[Dict[str, object]] = None) -> None:
    """Process all CSV files in a folder with unified y-axis ranges."""
    subfolders = [d for d in folder_path.iterdir() if d.is_dir()]
    
    csv_files_info = []
    if subfolders:
        for subfolder in sorted(subfolders):
            for csv_file in sorted(subfolder.glob('*.csv')):
                csv_files_info.append((csv_file, subfolder.name))
    else:
        for csv_file in sorted(folder_path.glob('*.csv')):
            csv_files_info.append((csv_file, None))

    if not csv_files_info:
        raise FileNotFoundError(f'No CSV files found in {folder_path} or its subfolders')

    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise ImportError(
            'matplotlib is required for plotting. Install it (e.g. pip install matplotlib).'
        ) from exc

    # Load all CSVs
    all_data: Dict[str, Dict] = {}
    max_time = 0.0
    for csv_file, subfolder_name in csv_files_info:
        try:
            rows = normalize_elapsed_time(load_collector_csv(csv_file))
            t_values = [r['elapsed_s'] for r in rows]
            run_key = f"{subfolder_name}_{csv_file.stem}" if subfolder_name else csv_file.stem
            all_data[run_key] = {
                't': t_values,
                'rows': rows,
                'subfolder': subfolder_name,
            }
            if t_values:
                max_time = max(max_time, max(t_values))
        except Exception as e:
            print(f'Warning: Skipped {csv_file.name}: {e}')

    if not all_data:
        raise ValueError('No valid CSV files found in folder.')

    plot_groups = plot_groups or {}
    per_run_groups = plot_groups.get('per_run', [])
    if not isinstance(per_run_groups, list):
        per_run_groups = []
    per_run_settings = plot_groups.get('per_run_settings', {})
    if not isinstance(per_run_settings, dict):
        per_run_settings = {}

    chart_groups = plot_groups.get('chart_comparison', [])
    if not isinstance(chart_groups, list):
        chart_groups = []

    sync_x_axis = bool(per_run_settings.get('sync_x_axis', False))
    sync_y_axis = bool(per_run_settings.get('sync_y_axis', False))

    # Compute global min/max for the metrics used in per-run plots only when requested.
    if sync_y_axis:
        per_run_metric_fields = _collect_metric_fields(per_run_groups)
        axis_limits = _compute_axis_limits(all_data, per_run_metric_fields) if per_run_metric_fields else {}
    else:
        axis_limits = None

    chart_metric_fields = _collect_metric_fields(chart_groups)
    chart_axis_limits = _compute_axis_limits(all_data, chart_metric_fields) if chart_metric_fields else {}

    covariance_values = [
        float(row.get('mean_covariance_trace', math.nan))
        for data in all_data.values()
        for row in data['rows']
    ]
    covariance_axis_limits = _log_axis_limits(covariance_values)
    if covariance_axis_limits:
        if axis_limits is None:
            axis_limits = {}
        else:
            axis_limits = dict(axis_limits)
        axis_limits['mean_covariance_trace'] = covariance_axis_limits

        chart_axis_limits = dict(chart_axis_limits)
        chart_axis_limits['mean_covariance_trace'] = covariance_axis_limits

    # Set global x-axis limit across runs only when requested.
    xlim = (0, max_time * 1.02) if sync_x_axis and max_time > 0 else None
    chart_xlim = (0, max_time * 1.02) if max_time > 0 else None

    # Output directory
    out_dir = (
        out_dir_arg.expanduser().resolve()
        if out_dir_arg
        else (folder_path.parent / 'plots')
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    # Generate plots for each CSV with unified axes
    for stem, data in all_data.items():
        _generate_per_run_figures(
            plt,
            data['t'],
            data['rows'],
            out_dir,
            stem,
            plot_groups,
            axis_limits=axis_limits,
            xlim=xlim,
        )

    _generate_comparison_figure(
        plt,
        all_data,
        out_dir,
        plot_groups,
        chart_axis_limits=chart_axis_limits,
        chart_xlim=chart_xlim,
    )

    print(f'Generated plots for {len(all_data)} runs in: {out_dir}')


if __name__ == '__main__':
    main()
