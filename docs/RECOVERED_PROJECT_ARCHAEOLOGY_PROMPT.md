You are acting as a senior research engineer and research scientist performing a **Project Archaeology / Technical Audit** of an existing FMCW-LiDAR small-UAV detection project.

This is NOT yet an idea-generation task.

Do NOT immediately propose new neural networks, manifold methods, novel modules, or paper contributions.

Your first responsibility is to thoroughly understand:

1. what this repository currently does,
2. what data are actually available,
3. what the current detection/tracking pipeline is,
4. what has already been implemented and tested,
5. where the system currently fails,
6. which bottlenecks are caused by the sensor/data and which are caused by the algorithm,
7. what information is still missing before research directions can be responsibly proposed.

The eventual research goal is small-UAV detection/tracking using FMCW LiDAR under extremely sparse point-cloud observations. A small UAV may produce only a few valid points in a frame. FMCW LiDAR may additionally provide information such as radial velocity/Doppler, intensity, timestamp, SNR, etc., depending on the actual dataset.

There is a possible future research hypothesis that UAV trajectories may possess exploitable low-dimensional geometric/manifold structure. However:

**Do NOT assume that manifold learning is the correct solution.**

Do not bias your diagnosis toward Grassmann manifolds, Riemannian methods, deep learning, tracking, clustering, or any predetermined technique.

The purpose of this audit is to determine what the real problem is before selecting a method.

---

# PHASE 0 — Repository safety

Before doing anything:

* Do not delete files.
* Do not overwrite existing experiment results.
* Do not modify the original dataset.
* Do not perform large-scale training.
* Do not install unnecessary dependencies.
* Do not refactor the repository during this audit.
* Do not silently fix bugs before documenting them.
* Do not change algorithm behavior simply to make the code run.

You may run lightweight diagnostic commands, inspect configuration files, inspect source code, examine a small number of samples, and execute existing evaluation/demo scripts when computationally reasonable.

If something cannot be executed, record exactly why.

Preserve the current project state.

---

# PHASE 1 — Build a repository map

Systematically inspect the repository.

Identify:

* directory structure,
* README/documentation,
* main entry points,
* training scripts,
* inference scripts,
* evaluation scripts,
* preprocessing scripts,
* visualization scripts,
* dataset loaders,
* configuration files,
* model definitions,
* clustering/detection modules,
* tracking modules,
* calibration files,
* experiment logs,
* checkpoints,
* result folders,
* notebooks,
* shell scripts,
* third-party code.

Produce a concise repository tree containing only scientifically relevant files.

For every major file/module, explain:

* its purpose,
* its inputs,
* its outputs,
* which other modules call it,
* whether it is currently used,
* whether it appears experimental/deprecated.

Then reconstruct the main execution path.

For example:

raw FMCW-LiDAR data
→ preprocessing
→ ROI filtering
→ background removal
→ point filtering
→ clustering
→ UAV candidate generation
→ tracking/data association
→ final detection
→ evaluation

Do NOT assume this example is correct.

Recover the actual pipeline from the code.

---

# PHASE 2 — Reconstruct the data model

This is one of the most important parts of the audit.

Determine exactly what one raw LiDAR observation contains.

Inspect the dataset loader and several representative raw samples.

Determine whether each point contains some subset of:

$$
x,\ y,\ z,\ range,\ azimuth,\ elevation,\ intensity,\ reflectivity,\ SNR,\ radial\ velocity,\ Doppler,\ timestamp
$$

or other attributes.

Report the exact representation.

For example:

```text
Point:
[x, y, z, intensity, radial_velocity, timestamp]
```

but report ONLY what is actually present.

Determine:

* coordinate system,
* units,
* frame rate,
* timestamp precision,
* sensor pose availability,
* calibration information,
* whether ego-motion exists,
* whether point-level radial velocity is available,
* whether radial velocity is signed,
* whether velocity ambiguity/wrapping exists,
* whether intensity/SNR/confidence exists.

Also determine dataset organization:

* number of sequences,
* number of frames,
* total duration,
* training/validation/test split if any,
* number of UAVs,
* UAV types,
* distance ranges,
* environments,
* stationary vs moving sensor,
* stationary vs moving UAV,
* single-UAV vs multi-UAV sequences.

If labels exist, determine exactly what is labeled:

* point labels,
* bounding boxes,
* UAV center,
* track IDs,
* trajectory,
* presence/absence,
* manually annotated candidate points,
* timestamps.

Explicitly distinguish:

**what is directly measured**

from

**what is derived or estimated by code**.

---

# PHASE 3 — Quantify the sparsity problem

Do not merely say that the point cloud is sparse.

Quantify it whenever the existing labels/data allow.

Calculate or recover statistics such as:

### UAV points per frame

$$
N_{\mathrm{UAV}}(t)
$$

Report:

* mean,
* median,
* standard deviation,
* minimum,
* maximum,
* quartiles,
* histogram if practical.

Determine approximately how often the UAV contains:

* 0 points,
* 1 point,
* 2 points,
* 3–5 points,
* 6–10 points,
* > 10 points.

If distance information is available, analyze:

$$
N_{\mathrm{UAV}}
\text{ vs. range}
$$

For example, determine whether the UAV becomes dramatically sparser with increasing distance.

Also inspect whether sparsity varies with:

* aspect angle,
* UAV type,
* radial velocity,
* flight direction,
* background,
* sequence.

If ground-truth point identity is unavailable, explain what prevents this calculation rather than inventing statistics.

---

# PHASE 4 — Reconstruct the current baseline

Identify the algorithm currently used by the previous developer/researcher.

Describe the complete baseline mathematically and algorithmically.

Possible components may include:

* ROI filtering,
* ground removal,
* static-background subtraction,
* velocity thresholding,
* DBSCAN,
* Euclidean clustering,
* connected components,
* CFAR,
* handcrafted thresholds,
* Kalman filter,
* EKF/UKF,
* IMM,
* Hungarian matching,
* JPDA,
* nearest-neighbor association,
* trajectory smoothing,
* neural networks.

Do not assume any of them are present.

For every actual component, determine:

### Input

What information enters the module?

### Operation

What transformation/algorithm is performed?

### Parameters

What thresholds/hyperparameters are used?

### Output

What does it produce?

### Dependency

What downstream components depend on it?

Represent the complete baseline as both:

1. a textual pipeline;
2. pseudocode.

Where useful, include mathematical equations.

---

# PHASE 5 — Reproduce the current evaluation

Locate existing reported results.

Determine:

* which experiment produced them,
* configuration used,
* checkpoint used,
* dataset split,
* evaluation metrics,
* whether the result can be reproduced.

Possible metrics include:

* Precision,
* Recall,
* F1,
* detection probability,
* false-alarm rate,
* localization error,
* MOTA,
* MOTP,
* ID switches,
* trajectory RMSE,
* latency.

Report only metrics that actually exist.

If possible, run the existing evaluation pipeline without retraining.

Record the exact command.

Example:

```bash
python evaluate.py --config ...
```

Report whether the result matches previous logs.

If not, investigate the discrepancy.

Do not hide reproducibility problems.

---

# PHASE 6 — Trace where detections are lost

This phase is critical.

For a true UAV target, determine at which stage it can disappear.

For example:

```text
raw return exists
↓
ROI filtering
↓
background filtering
↓
velocity filtering
↓
clustering
↓
candidate threshold
↓
association
↓
final track
```

Estimate, if possible, how many detections are lost at each stage.

Try to distinguish:

$$
\text{sensor miss}
$$

from

$$
\text{algorithmic miss}.
$$

Examples:

### Sensor-limited failure

No valid UAV return exists in the frame.

### Preprocessing failure

UAV points exist but are removed by filtering.

### Clustering failure

UAV returns survive preprocessing but do not form a cluster.

### Classification failure

A valid cluster exists but is rejected.

### Association failure

The UAV is detected but assigned to the wrong trajectory.

### Track-management failure

Temporary missing observations cause the entire track to terminate.

Do the same for false positives:

Where do clutter points become false UAV detections?

---

# PHASE 7 — Construct a Failure Atlas

Inspect representative failure cases from multiple sequences.

Do not select only visually convenient examples.

Try to establish categories such as:

* extreme point sparsity,
* UAV completely missing in a frame,
* long-range detection,
* background clutter,
* vegetation,
* buildings,
* ground reflections,
* moving clutter,
* radial velocity close to zero,
* tangential motion,
* radial motion,
* hovering,
* acceleration,
* sharp turning,
* crossing trajectories,
* fragmented clusters,
* temporal association failure,
* sensor noise,
* multipath,
* parameter-threshold sensitivity.

These are examples only.

Derive the actual categories from the data.

Create a table:

| Failure category | Observable symptom | Likely pipeline stage | Approx. frequency | Representative examples |
| ---------------- | ------------------ | --------------------- | ----------------: | ----------------------- |

If frequency cannot currently be measured, mark it as:

`unknown — requires annotation`

rather than estimating it.

Identify the **top 3–5 dominant failure modes**.

---

# PHASE 8 — Analyze the role of FMCW radial velocity

Determine whether the existing pipeline actually uses FMCW/Doppler/radial-velocity information.

If radial velocity is available, determine:

$$
v_r
$$

definition and sign convention.

Inspect its statistical behavior for:

* UAV points,
* stationary clutter,
* moving clutter,
* different flight directions.

Pay special attention to the geometric limitation:

If UAV velocity is approximately tangential to the sensor line of sight,

$$
v_r \approx 0
$$

even when the UAV itself moves quickly.

Determine whether this causes failure in the existing pipeline.

Check whether current code relies on a threshold such as:

$$
|v_r| > \tau
$$

and whether that may incorrectly suppress tangentially moving UAVs.

Also determine whether radial velocity is currently used only for filtering or whether it is incorporated into:

* clustering,
* temporal association,
* state estimation,
* trajectory prediction.

Do NOT propose a new method yet.

Just diagnose how much sensor information is currently being exploited or discarded.

---

# PHASE 9 — Analyze temporal information

Determine how many frames are currently processed jointly.

Is detection:

$$
P_t \rightarrow D_t
$$

purely single-frame?

Or does it use:

$$
\{P_{t-K},...,P_t\}
\rightarrow D_t?
$$

Determine whether the project currently performs:

* multi-frame accumulation,
* motion compensation,
* track-before-detect,
* temporal clustering,
* tracking after detection,
* trajectory smoothing.

Very importantly, distinguish:

### detect-then-track

$$
\text{frame detection}
\rightarrow
\text{association}
\rightarrow
\text{track}
$$

from

### track-before-detect / temporal evidence accumulation

$$
\text{weak observations across frames}
\rightarrow
\text{trajectory hypothesis}
\rightarrow
\text{detection}.
$$

Determine which paradigm the current system implements.

---

# PHASE 10 — Analyze trajectory information

Inspect any available ground-truth or estimated UAV trajectories.

Determine:

* trajectory sampling frequency,
* trajectory duration,
* smoothness,
* typical speed,
* acceleration,
* turning behavior,
* missing observations.

If data allow, inspect basic quantities:

$$
\|\mathbf v_t\|,
\qquad
\|\mathbf a_t\|,
\qquad
\Delta \theta_t
$$

and temporal continuity.

Do NOT claim that the trajectory forms a manifold.

Instead answer:

> Is there empirical evidence that UAV trajectories occupy a constrained/structured subset of the full state space?

If possible, inspect:

* PCA dimensionality,
* local PCA,
* intrinsic-dimensionality estimates,
* covariance spectrum,
* trajectory-state visualization.

Use these only as diagnostics.

Do not turn them into a proposed contribution.

---

# PHASE 11 — Inspect software and experimental reliability

Audit the project for scientific reproducibility.

Identify:

* random seeds,
* hard-coded paths,
* hard-coded thresholds,
* undocumented preprocessing,
* data leakage,
* inconsistent train/test preprocessing,
* hidden manual steps,
* stale checkpoints,
* unused code,
* duplicate implementations,
* metric bugs,
* coordinate-frame inconsistencies,
* unit inconsistencies,
* timestamp misalignment,
* velocity-sign inconsistencies.

Separate findings into:

### Critical

May invalidate experiments.

### Important

May materially affect performance.

### Minor

Engineering cleanliness/reproducibility issue.

Do not fix them silently.

Document evidence with file paths and line/function references.

---

# PHASE 12 — Identify parameter sensitivity

List important manually selected parameters.

Examples may include:

$$
\epsilon_{\mathrm{DBSCAN}},
\quad
\text{minPts},
\quad
v_{\min},
\quad
R_{\max},
\quad
K_{\text{frames}},
\quad
\text{association threshold}.
$$

Determine:

* where they are defined,
* whether they vary across sequences,
* whether they were empirically tuned,
* whether results appear highly sensitive to them.

If existing logs contain sweeps, summarize them.

Do not start a large hyperparameter sweep.

---

# PHASE 13 — Determine the actual scientific bottlenecks

Only after completing the previous analysis, summarize the system into approximately 3–5 bottlenecks.

Each bottleneck should follow:

## Bottleneck X

### Observation

What does the code/data show?

### Evidence

Which files, results, statistics, or failure cases support it?

### Mechanism

Why does the existing method fail?

### Research question

Turn the engineering failure into a scientific question.

Example format:

> Observation:
> UAVs frequently generate fewer than three valid returns per frame.

> Mechanism:
> DBSCAN requires spatial density within a single frame and therefore cannot reliably form a target cluster.

> Research question:
> How can weak spatial evidence be accumulated across time without simultaneously amplifying clutter?

This is the level of abstraction required.

Do NOT propose the solution yet.

---

# PHASE 14 — Separate facts from hypotheses

Create three sections:

## Established from the repository/data

Statements directly supported by evidence.

## Plausible but unverified

Statements that appear likely but require experiments.

## Unknown

Important information that is currently unavailable.

For example:

```text
Established:
The detector uses |vr| > 0.3 m/s.

Plausible:
Tangential UAV motion may cause false negatives.

Unknown:
How many false negatives occur specifically when |vr| < 0.3 m/s.
```

This distinction is mandatory.

---

# FINAL DELIVERABLE

Produce a report named:

```text
FMCW_LiDAR_Project_Archaeology.md
```

The report should contain the following structure:

# 1. Executive Summary

Approximately 1–2 pages.

Explain:

* what the system currently does,
* current performance if known,
* major limitations,
* top scientific bottlenecks,
* missing information.

Do not propose new research methods.

---

# 2. Repository Architecture

Important folders, files, and execution flow.

---

# 3. Dataset and Sensor Representation

Describe exactly what data exist.

Include a table such as:

| Attribute       | Available | Unit | Source | Used by current algorithm |
| --------------- | --------- | ---- | ------ | ------------------------- |
| x,y,z           |           |      |        |                           |
| intensity       |           |      |        |                           |
| radial velocity |           |      |        |                           |
| timestamp       |           |      |        |                           |
| SNR             |           |      |        |                           |

---

# 4. Dataset Statistics

Especially UAV point sparsity and range dependence.

---

# 5. Current Algorithm Pipeline

Provide:

* pipeline diagram in text,
* pseudocode,
* important parameters.

---

# 6. Current Experimental Results

Existing metrics and reproducibility status.

---

# 7. Failure Atlas

Include representative failure categories.

---

# 8. FMCW-Specific Information Usage

Especially radial velocity/Doppler.

---

# 9. Temporal and Trajectory Information Usage

Explain what current system does and does not exploit.

---

# 10. Code and Experimental Reliability Audit

Critical / Important / Minor issues.

---

# 11. Dominant Bottlenecks

Rank approximately 3–5 bottlenecks by importance.

The ranking should reflect evidence from the project, NOT perceived novelty.

---

# 12. Scientific Questions Emerging from the Audit

For each bottleneck, formulate research questions.

Do NOT provide proposed methods yet.

Examples of acceptable questions:

* How can a UAV be detected when a single frame contains too few points for spatial clustering?
* How can radial velocity and geometry jointly distinguish UAV returns from clutter?
* How can temporal information be accumulated without increasing false alarms?
* How should intermittent observations be associated into a physically plausible trajectory?

These are examples only.

Generate questions based on the actual project.

---

# 13. Missing Information

List everything required before reliable research ideation.

For each missing item state:

* why it matters,
* how to obtain it,
* whether it blocks future research decisions.

---

# 14. Recommended Next Diagnostic Experiments

This section may contain ONLY diagnostic experiments.

Examples:

* plot UAV-point count versus distance,
* inspect recall versus radial velocity,
* measure which preprocessing stage removes true UAV points,
* visualize consecutive 20-frame sequences,
* evaluate DBSCAN success versus target point count.

Do NOT propose a new model.

Limit this section to approximately 5–10 high-value diagnostics.

---

# 15. Research Readiness Assessment

End the report by answering:

### A. Do we currently understand why the baseline fails?

Yes / Partially / No

Explain why.

### B. Do we currently have enough information to start literature-driven idea generation?

Yes / Partially / No

### C. What are the 3 most important missing pieces of evidence?

### D. What should be investigated next before proposing a research method?

---

# IMPORTANT BEHAVIOR

Throughout this task:

* Be skeptical.
* Do not invent missing dataset information.
* Do not infer algorithm behavior solely from filenames.
* Trace actual code paths.
* Cite filenames/functions when describing implementation.
* Distinguish active code from abandoned experiments.
* Distinguish measured facts from interpretations.
* Quantify whenever possible.
* Prefer evidence over intuition.
* Report negative findings.
* Report inconsistencies.
* Report unsuccessful reproduction attempts.
* Do not optimize for making the project look good.
* Optimize for understanding why the current system works or fails.

Most importantly:

**Do not generate a paper idea yet.**

The next research stage will use this audit together with targeted literature review to generate and evaluate candidate research directions.

For now, the goal is simply:

> Understand the project deeply enough that subsequent research questions are driven by real sensor and algorithmic bottlenecks rather than by fashionable methods.
