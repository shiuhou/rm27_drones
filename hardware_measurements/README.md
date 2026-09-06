# Hardware Measurements (M4-B data ingestion)

Raw bench measurements for replacing M4-A engineering estimates with real
RM27 hardware data. **Raw CSVs here are never modified or overwritten** —
`analyze_hardware_measurements.py` only reads them and writes derived
output to `analysis/`.

## Expected files

| file | content |
|---|---|
| `thrust_2s_2in.csv` | 1103 9500 KV + 2.0" prop on 2S, one motor |
| `thrust_2s_2p5in.csv` | 1103 9500 KV + 2.5" prop on 2S |
| `thrust_3s_2in.csv` | 1103 9500 KV + 2.0" prop on 3S |
| `thrust_3s_2p5in.csv` | 1103 9500 KV + 2.5" prop on 3S |
| `airframe_measurements.csv` | AUW, wheelbase, COM, component masses |

Missing files (or header-only templates) are reported as
`NEEDS_MEASUREMENT` — never fabricated.

## Thrust CSV schema

```csv
throttle,voltage_v,current_a,thrust_gf,rpm
0.00,7.40,0.02,0.0,
0.10,7.39,0.35,12.4,
...
```

- `throttle`: command fraction 0–1 **or** percent 0–100 (auto-normalized;
  ambiguous ranges are rejected).
- `voltage_v`: measured pack voltage **under load** at each step [V].
- `current_a`: measured motor current [A].
- `thrust_gf`: measured thrust in gram-force (converted to N via
  1 gf = 0.00980665 N exactly).
- `rpm`: column required, values may be blank.
- Rows should sweep from low to high throttle; duplicates are averaged,
  at least 2 distinct settings required.

Bench protocol suggestion: hold each throttle step ~2–3 s, record the
settled values, note prop direction and battery charge state in a
`notes` column you add **after** `rpm` (extra trailing columns are
ignored by the analyzer).

## Airframe CSV schema

```csv
parameter,value,unit,notes
auw_kg,0.180,kg,"scale with battery + props"
wheelbase_m,0.120,m,"diagonal motor-to-motor"
```

`auw_kg` unlocks thrust-to-weight and hover throttle/current estimates;
without it those fields stay `NEEDS_MEASUREMENT`.

## Running the analysis

```bash
python analyze_hardware_measurements.py
```

Outputs (in `hardware_measurements/analysis/`):

- `hardware_analysis.json` — machine-readable summary per combination:
  max thrust, throttle/current/power at max, gf/W and N/A efficiency,
  thrust-to-weight (if AUW known), hover throttle/current from the fitted
  monotonic thrust curve (PAVA isotonic fit), and the curve itself.
- `thrust_<combo>_curve.png` — measured vs fitted curve plus
  current/power panels.

These results do **not** feed the MuJoCo plant yet; M4-B will consume
`hardware_analysis.json` after review.
