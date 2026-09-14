# Experiment 2 - final reports and LaTeX sources

Repository: https://github.com/ccx8kg5qpd-debug/robomaster-ep-fixed-point-grasping

The repository is private and shared by the five-person group. The five reports
reference the same group project and distinguish each member's assigned scope.

## Final PDFs

- `24020036050_Qi_Yongyue_Experiment_2.pdf` - 10 pages, QYY personal template.
- `24020036049_Ning_Bo_Experiment_2.pdf` - 6 pages, simulation-blueprint design.
- `24020036027_Huang_Junbiao_Experiment_2.pdf` - 6 pages, control-journal design.
- `24020036072_Xue_Yutong_Experiment_2.pdf` - 6 pages, hardware field-note design.
- `24020036004_Bao_Lintai_Experiment_2.pdf` - 6 pages, operations-manual design.

All reports are in English and include the repository link plus a commit-history
image. The image records the first five material-upload commits and states that
the archive was assembled after the experiment; it does not fabricate historical
development commits.

## Compile

Use Tectonic from the relevant source directory.

For Yongyue Qi:

```text
tectonic QYY_Experiment_2_Report.tex
```

For the other reports, compile the selected student `.tex` file from
`LaTeX/Four_Individual_Reports/` so that the shared preamble, personal style,
assets and supporting data resolve by relative path.

## Outstanding submission item

The actual RoboMaster EP hardware video has not yet been provided. The repository
contains the complete simulation recording and a clearly labelled pending marker
for the hardware recording. Add the hardware video in a later commit rather than
replacing the simulation video or describing unrecorded measurements.

## Verification

`QA/summary.json` records the final page counts and checks for the correct names,
student IDs, repository identifier, commit evidence, 5/5 result and empty pages.
All final pages were rendered and visually reviewed after compilation.
