# Paper draft: Optimisation-Based Manipulation Planning in Convex Decompositions of the Composite Configuration Space

IEEE conference format (`IEEEtran`, two-column), 8-page target. No experimental results yet. Every gap is marked inline.

## Build

```bash
latexmk -pdf main.tex
```

To check the length without the placeholder notes, set `\showtodosfalse` in `main.tex`.

## Placeholder conventions

| Macro | Colour | Meaning |
|---|---|---|
| `\todo{...}` | red | Missing content or a decision you need to make |
| `\tocheck{...}` | orange | Written from notes, an older pipeline or memory; verify before submission |
| `\tbd` | red | Missing number |

## Layout

```
main.tex                 preamble, macros, title/authors, section includes
sections/abstract.tex
sections/introduction.tex
sections/related_work.tex
sections/problem.tex     composite space, CG/CP, transit/transfer, problem statement
sections/method.tex      decomposition, grasp polytopes (NLP sampling + null space),
                         mode sets, MIQP, A* (draft), GCS, guarantees
sections/experiments.tex setup (tasks T1-T4, implementation, baselines, metrics) + results
sections/discussion.tex
sections/conclusion.tex
references.bib
figures/                 copied from the Year-1 annual review handout (originals untouched)
```

## Sources used

- **MInf2 report**: `~/Downloads/MINF2.pdf`. Problem statement, the MIQP and its constraints, GCS stage, 3-DOF task definition.
- **Year-1 annual review report**: `~/PhD-Literature/Presentations/21.09.26 - Annual Review Year 1/`. Sections 5.1 and 6.1.1, Table 5.1 (low-DOF results), the three grasp-polytope approaches, and the bibliography.
- **IPAB workshop notes**: `~/PhD-Literature/Presentations/13.08.26 - IPAB Workshop - MInf + NLP Sampling/Notes for Content.md`. Basis for the abstract.
- **This repo**:
  - `origin/main`: `algorithms/nlp_sampling/` (sampler and docs), `standalone_test.py` (grasp constraints h and g), `data/generation/full_arm_c_free.ipynb` (scene and obstacles), `data/cfree/cfree_full_98coverage.yaml` (18 regions).
  - Branch `origin/nlp-sampling/quality-metrics`: `tc_space_investigation_summary.md` and `wrist_axis_grasp_polytope.py` (null-space grasp polytope, wrist-axis invariance), `msts_metric_reference.md`.
  - Local `main` is behind `origin/main` (the IRIS-ZO/clique-cover port and the shared `ManipulationPlanner` module are only on origin). Nothing was pulled or changed; files were read with `git show`.

## Main open items

1. **A\* sequencing (Sec. IV-E)**: written as a draft formulation, since no implementation exists yet. Settle the search state, edge cost and heuristic, and choose between greedy entry points and a GCS*-style expansion.
2. **Mode constraints inside GCS (Sec. IV-F)**: the text says linear transit and transfer constraints are imposed on all Bézier control points. The MInf version did not do this; confirm or implement.
3. **MIQP failure on 10-DOF**: attributed to big-M and thin polytopes. Confirm.
4. **C-IRIS certification via TC-space vertex mapping**: the back-mapped hull is not guaranteed to stay certified. Fix, call it approximate, or drop it.
5. **Grasp-polytope parameters and validation numbers**: ε, η, counts, volumes. HR vs. mRRT mixing numbers.
6. **Experiments**: baselines (HPP or a manipulation-graph PRM, OMPL constrained planners), the 10-DOF results table, re-running Table I with the final pipeline, T3 (keep or drop), obstacle sizes and start/goal configurations.
7. **Novelty claim** (intro and related work): re-check against recent GCS manipulation work.
8. **Authors, emails, grant number, venue**: if the venue is double-blind, anonymise the MInf self-citation.
9. **Bibliography**: written from the annual review and from memory. Verify every entry against DBLP or the publisher.

## Length budget

With placeholders hidden, the draft is exactly 8 pages, but the 10-DOF results and grasp-polytope result subsections are still empty. Expect to free about 0.75–1 page for results. Candidate cuts:
- Related work: drop the legged mixed-integer refs, Kurtz/INSAT, Tournassoud.
- Algorithm 2: fold it into the text.
- Problem-formulation bullets: compress them.
- Discussion: compress the alternatives paragraph.

The TC-space paragraph in the discussion is already disabled with `\iffalse`.
