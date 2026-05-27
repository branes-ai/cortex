<!-- markdownlint-disable-file MD041 -->
<!--
PR title MUST be a Conventional Commits message — type(scope): subject —
with the subject starting lowercase (commitlint enforces this). e.g.
  feat(sdk): camera updaters with MSCKF null-space projection

(A PR template intentionally opens with author guidance, not a top-level
heading — hence the markdownlint MD041 exemption above.)
-->

## Summary

<!-- What this PR does and why, in a few lines. -->

## Changes

- `path/to/file` — what changed

## Test Results

<!-- Build/test both compilers for C++ changes; delete the row that doesn't apply. -->

| Target | gcc build | gcc test | clang build | clang test |
|--------|-----------|----------|-------------|------------|
| <target> | OK | PASS | OK | PASS |

## Checklist

- [ ] PR title is a Conventional Commits message (lowercase subject).
- [ ] Built and tested locally (gcc **and** clang for C++ changes).
- [ ] Layering respected — no new cross-layer dependency (`math/` stays middleware-free; `sdk/` operators don't branch on the build target). See [Layering Invariants](https://branes-ai.github.io/cortex/architecture/layering/).
- [ ] New/changed algorithms are clean-room with a paper citation (no GPL), and any test oracle is independent.
- [ ] **Docs updated.** If this PR adds or changes a subsystem, benchmark, or public API, the corresponding page under `docs-site/` is updated in this PR (and any benchmark gate it moves is reflected). If docs are not affected, say so below.

<!-- Docs note (why no docs change was needed, or what was updated): -->

## Linked issue

Resolves #<!-- issue number; use "Relates to #" for partial work, and never "Resolves" for an Epic -->
