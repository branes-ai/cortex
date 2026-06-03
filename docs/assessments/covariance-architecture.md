# Covariance versions of MSCKF

Resolution plan — #187 square-root covariance MSCKF

Root design goal. Add a square-root-form MSCKF that carries the Cholesky factor S of the covariance (P = S·Sᵀ) instead of P, with a QR/Householder array
update replacing the Joseph form — selectable alongside MsckfBackend<T> without touching the front end. Acceptance: poses match the full-cov backend within
numerical tolerance on the synthetic end-to-end test, with S well-conditioned over a long run.

The math is fully worked out (all clean-room from textbook array algorithms — Kailath/Sayed/Hassibi Linear Estimation, Mourikis MSCKF — not from any GPL
source, per the clean-room rule). Every covariance operation reduces to one primitive: a Householder QR returning the upper-triangular R.
- Predict P←FPFᵀ+Q: retriangularize [F·S | √Q].
- Augment clone: retriangularize G·S (G = clone selector) — naturally handles the singular clone block.
- Marginalize: drop the clone's rows from S, retriangularize.
- Update: array algorithm on [[√R, H·S],[0, S]] → yields innovation factor Rₑ, gain block, and updated S' in one QR; δx = K̄·(Rₑ⁻¹r) via triangular solves.
- Gate: γ = ‖Rₑ⁻¹r‖² where Rₑ factors H·P·Hᵀ+R — no explicit inverse.

The genuinely shared, covariance-independent code is large: IMU mean strapdown, triangulation, projection/Jacobians, null-space projection, and the entire
backend feature-management policy (~700 lines). Only the ~5 covariance operations differ. 
