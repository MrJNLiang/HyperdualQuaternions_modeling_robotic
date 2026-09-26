# DQ/HDQ feedback design and offline validation

Read [analysis.md](analysis.md) for the Chinese derivation, assumptions, fair comparisons, numerical results and limitations. This is a separate research experiment; it does not change or import the robot's actuator interface.

The tested candidate family is:

- DQ-I: geometric filtered error plus constant-disturbance internal model.
- DQ-IM: DQ-I plus harmonic internal models at 0.6, 1.2, 1.8 rad/s.
- C2+I / C2+IM: essential ablations showing the benefit shared by an extended C2.

The geometry and controller are in `controller.py`. The full rigid-body torque plant is in `experiment.py`; it uses the B601 URDF already in this repository, synthetic armature, smooth friction and model uncertainties. The conclusions are not hardware predictions.

## Tested environment

Python 3.8.20, NumPy 1.24.4, SciPy 1.10.1, Matplotlib 3.7.5, Pinocchio 2.7.0. This machine already provides that environment at:

`/home/liang/miniconda3/envs/dq_hinf/bin/python`

`requirements.txt` records the four direct package versions; native Pinocchio dependencies must also be available. The executable below refers to this existing environment, so no package installation is needed on the current machine.

## Reproduce from the repository root

```bash
DQ_PYTHON=/home/liang/miniconda3/envs/dq_hinf/bin/python
DQ_DIR=docs/dq_feedback_design_20260920
"$DQ_PYTHON" "$DQ_DIR/verify.py"
"$DQ_PYTHON" "$DQ_DIR/verify_internal_model.py"
"$DQ_PYTHON" "$DQ_DIR/experiment.py" --seeds 3
"$DQ_PYTHON" "$DQ_DIR/experiment.py" --laws DQ-IM C2+IM --out "$DQ_DIR/results_im"
"$DQ_PYTHON" "$DQ_DIR/sensitivity.py"
"$DQ_PYTHON" "$DQ_DIR/plot_results.py"
```

A shorter initial run is available with `experiment.py --quick --seeds 1 --out /tmp/dq-feedback-quick`. It intentionally covers only three cases and three laws.

Main runs use fixed parameters lambda=3, kappa=12, k0=16, harmonic gain=16, normalized translation length=1 m. C2 has Kp=36, Kd=15. C1 has local rotation stiffness matched to C2. The integral-free DQ variant has the same local PD gains; enabling the internal model changes the local closed-loop dynamics.

No optimization selected a best gain from every result row. The main parameters are shared across all main cases. The IM extension was motivated by the DQ-I friction limitation; seeds 1 and 2 and longer/time-refinement cases are follow-up checks, not a large held-out benchmark. Three harmonics use the known reference frequency. The sensitivity script varies C2 gains, timing and internal-model frequency explicitly.

## Outputs

- `results/`: 120 main physical runs, traces for seed 0, mathematical verification and frequency data.
- `results_im/`: 48 physical runs adding DQ-IM and C2+IM.
- `sensitivity_results/`: 39 physical follow-up runs.
- `summary.json`: aggregates with failures excluded from normal RMS comparisons.
- `figures/`: standalone static tracking, gain/effort tradeoff and certificate plots.
- `run.log`, `run_im.log`, `sensitivity.log`: original execution logs.

Physical trace columns: t; true TCP position error; true orientation angle; DQ translation-error norm; minimum singular value; limit flag; q[6]; qdot[6]; applied torque[6]; constant-bias estimate[6]. Harmonic states are internal controller variables and are not included in these physical traces. Desired trajectories are reconstructible from `reference()`.

Late-error metrics use 70–100% of the declared run duration. Torque norm RMS, torque-rate norm RMS and sampled joint-jerk norm RMS use the whole run. For terminated runs, raw metrics describe only the saved prefix and must not be compared as completed-run performance. `summary.json` reports failure counts instead. The original 120-run metrics predate the additional explicit `metrics_scope` field, but their `failed` flags carry the same meaning.

Main failures are intentionally retained: strong pulse: 1/3 failures per law; saturation stress: 3/3. Continuous-time proofs exclude the conditional antiwindup gate, saturation, sensor-noise dynamics and discrete implementation. The detailed report explains the model-error budget needed before extending the certificate to physical torque control.
