// SPDX-License-Identifier: MIT
//
// branes/tools/observability_ledger.hpp — where a running MSCKF picks up
// information it cannot have: the per-stage ledger of spurious yaw and
// global-position information (issue #480, follow-up to #457 and #212).
//
// The body-frame VIO state has four unobservable directions at its
// linearization point x̂: global translation and yaw about gravity, the columns
// of N(x̂). A consistent filter never gains information along them. The S6 bench
// showed that a single camera update cannot add any: S6 evaluates every Jacobian
// at one estimate, so H·N(x̂) = 0 by construction (#457). Whatever spurious
// information the filter collects must come from the stages disagreeing about
// the linearization point.
//
// This tap measures that. It carries the unobservable basis through the run the
// way the filter's own linear system carries it,
//
//   S2   T ← Φ T              (Φ = the propagator's own transition)
//   S3   T ← [I; J] T         (the clone copies the IMU pose rows)
//   S6   T unchanged          (an update moves the mean, not the error basis)
//   S9   T ← T without the marginalized clone's rows
//
// starting from T = N(x̂) at the first boundary it sees. In that linear system
// the camera's Fisher information about the starting unobservable directions is
// Σ_updates Tᵀ Hᵀ R⁻¹ H T. A consistent filter keeps it at 0; anything else is
// information the filter believes it has about yaw and global position.
//
// T drifts away from the current N(x̂) only by defects that individual stages
// introduce, so T = N(x̂) + Σ_stage D_stage exactly:
//
//   S2_propagation      Φ N(x̂⁻) − N(x̂⁺): Φ, linearized against the mean's Euler
//                       step, does not carry the basis at the old mean onto the
//                       basis at the new mean (a higher-order Φ leaks more, #471)
//   S3_augmentation     [I; J] N(x̂⁻) − N(x̂⁺): zero, a clone is an exact copy
//   S6_mean_shift       N(x̂⁻) − N(x̂⁺): the update moves the linearization point
//                       under a basis that stays put
//   S9_marginalization  zero, row deletion commutes with N
//   untapped            a state change no tap reported (should stay zero)
//
// Each defect is then transported by the later stages like T. At an update,
// R^-½ H T = R^-½ H N(x̂) + Σ R^-½ H D_stage, and the first term is S6's own
// projection residual (S6_projection, roundoff by #457). The spurious
// information splits exactly into per-stage shares, Σ (R^-½ H T)·(R^-½ H D),
// which sum to the total. A share can be negative (a stage that cancels part of
// another's leak). The "alone" numbers, ‖R^-½ H D‖², say what one stage would
// leak if it were the only one.
//
// The information ignores process noise, which only forgets it, so it bounds
// the spurious information from above. The cross-check is the filter's own yaw
// σ: with prior yaw information 1/σ₀², the ledger predicts a yaw σ no smaller
// than 1/√(1/σ₀² + I_yaw) from the camera's spurious information alone.
//
// A self-check guards the bookkeeping: the directly transported T and
// N(x̂) + Σ D must agree to roundoff (`closure_max`).
//
// Header-only, C++20. Full-covariance states only (as the stage recorder).

#ifndef BRANES_TOOLS_OBSERVABILITY_LEDGER_HPP
#define BRANES_TOOLS_OBSERVABILITY_LEDGER_HPP

#include <branes/sdk/msckf/stage_tap.hpp>
#include <branes/sdk/msckf/stages/s6_msckf_update.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <ostream>
#include <string>
#include <string_view>
#include <vector>

namespace branes::tools {

/// Where a defect between the transported basis and N(x̂) came from.
enum class LeakSource : std::size_t {
    S2_propagation = 0,
    S3_augmentation,
    S6_projection,
    S6_mean_shift,
    S9_marginalization,
    untapped,
    kCount
};

inline constexpr std::size_t kLeakSources = static_cast<std::size_t>(LeakSource::kCount);

[[nodiscard]] constexpr std::string_view leak_source_name(LeakSource s) noexcept {
    constexpr std::array<std::string_view, kLeakSources> names{
        "S2_propagation", "S3_augmentation", "S6_projection", "S6_mean_shift", "S9_marginalization", "untapped"};
    return names[static_cast<std::size_t>(s)];
}

/// The 4-D unobservable basis of the body-frame error state at `s` (right
/// perturbation R ← R·Exp(δθ)): columns 0–2 global translation (δp = eᵢ on the
/// IMU and every clone), column 3 yaw about world ẑ (δθ = Rᵀẑ, δp = ẑ×p,
/// δv = ẑ×v; per clone δθ_c = R_cᵀẑ, δp_c = ẑ×p_c). Bias and calibration rows
/// are zero. The same basis as the S6 benches' `s6::unobservable_basis`.
template <class St>
[[nodiscard]] sdk::msckf::DynMat<double> unobservable_basis(const St& s) {
    sdk::msckf::DynMat<double> n(s.dim(), 4);
    auto put = [&](std::size_t row, std::size_t col, double x, double y, double z) {
        n(row, col) = x;
        n(row + 1, col) = y;
        n(row + 2, col) = z;
    };
    // ẑ × a = (−a_y, a_x, 0); Rᵀẑ is the third row of R.
    auto yaw_rows = [&](std::size_t th, std::size_t pos, const auto& R, const auto& p) {
        const auto M = R.matrix();
        put(th, 3, static_cast<double>(M(2, 0)), static_cast<double>(M(2, 1)), static_cast<double>(M(2, 2)));
        put(pos, 3, -static_cast<double>(p[1]), static_cast<double>(p[0]), 0.0);
    };
    for (std::size_t i = 0; i < 3; ++i)
        n(St::kPos + i, i) = 1.0;
    yaw_rows(St::kTheta, St::kPos, s.R, s.p);
    put(St::kVel, 3, -static_cast<double>(s.v[1]), static_cast<double>(s.v[0]), 0.0);
    for (std::size_t c = 0; c < s.clones.size(); ++c) {
        const std::size_t off = s.clone_offset(c);
        for (std::size_t i = 0; i < 3; ++i)
            n(off + 3 + i, i) = 1.0;
        yaw_rows(off, off + 3, s.clones[c].R, s.clones[c].p);
    }
    return n;
}

template <math::Scalar T, class Cov = sdk::msckf::FullCovariance<T>>
class ObservabilityLedger final : public sdk::msckf::StageTap<T, Cov> {
public:
    using Base = sdk::msckf::StageTap<T, Cov>;
    using St = typename Base::St;
    using Vec3 = typename Base::Vec3;
    using Mat = sdk::msckf::DynMat<double>;

    /// Spurious information along yaw (rad⁻²) and global translation (m⁻²,
    /// summed over the three axes), split by the stage that introduced it.
    struct Split {
        double yaw = 0.0;
        double pos = 0.0;
    };

    struct Totals {
        Split info{};                             ///< Σ ‖R^-½ H T‖² over applied updates
        std::array<Split, kLeakSources> share{};  ///< additive: Σ share = info
        std::array<Split, kLeakSources> alone{};  ///< one stage's defect on its own
        std::size_t s2_steps = 0, s3_clones = 0, s6_applied = 0, s6_skipped = 0, s9_marginalized = 0;
        std::size_t restarts = 0;       ///< the state was re-seeded (dimension jump); the ledger restarted
        std::size_t prep_mismatch = 0;  ///< the backend applied a track this replay would not (or vice versa)
        double closure_max = 0.0;       ///< max |T − N(x̂) − Σ D| (bookkeeping self-check)
        double yaw_sigma_start = 0.0;   ///< the filter's world-yaw σ at the first boundary (rad)
        double yaw_sigma_end = 0.0;     ///< … at the last boundary
        double yaw_sigma_min = std::numeric_limits<double>::infinity();

        /// The yaw σ (rad) the spurious information alone allows: 1/√(1/σ₀² + I_yaw).
        [[nodiscard]] double yaw_sigma_floor() const {
            if (!(yaw_sigma_start > 0.0))
                return 0.0;
            return 1.0 / std::sqrt(1.0 / (yaw_sigma_start * yaw_sigma_start) + info.yaw);
        }
    };

    /// One row per camera frame: the running totals at that frame.
    struct Row {
        double t = 0.0;
        Split info{};
        std::array<Split, kLeakSources> share{};
        double yaw_sigma = 0.0;
    };

    [[nodiscard]] bool wants(sdk::msckf::TapStage stage) const override {
        using S = sdk::msckf::TapStage;
        return stage == S::S2_propagation || stage == S::S3_augmentation || stage == S::S6_msckf_update ||
               stage == S::S9_marginalization;
    }

    void on_frame(double t) override {
        if (!started_)
            return;
        Row r;
        r.t = t;
        r.info = tot_.info;
        r.share = tot_.share;
        r.yaw_sigma = tot_.yaw_sigma_end;
        rows_.push_back(r);
    }

    void on_s2(const St& before,
               const sdk::msckf::Propagator<T>& prop,
               const Vec3& gyro,
               const Vec3& accel,
               T dt,
               const St& after) override {
        enter(before);
        if (dt > T{0}) {
            const auto tr = prop.transition(before, gyro, accel, dt);
            Mat F(tr.F.rows, tr.F.cols);
            for (std::size_t i = 0; i < F.d.size(); ++i)
                F.d[i] = static_cast<double>(tr.F.d[i]);
            T_ = sdk::msckf::mul(F, T_);
            for (auto& D : D_)
                D = sdk::msckf::mul(F, D);
            add_defect(LeakSource::S2_propagation, sdk::msckf::mul(F, unobservable_basis(before)), after);
            ++tot_.s2_steps;
        }
        leave(after);
    }

    void on_s3(const St& before,
               double /*t*/,
               const sdk::msckf::stages::s3_augmentation::Diagnostics& /*diag*/,
               const St& after) override {
        enter(before);
        auto clone = [&](const Mat& M) {
            Mat out(M.rows + St::kCloneDim, M.cols);
            std::copy(M.d.begin(), M.d.end(), out.d.begin());
            for (std::size_t j = 0; j < M.cols; ++j)
                for (std::size_t i = 0; i < 3; ++i) {
                    out(M.rows + i, j) = M(St::kTheta + i, j);
                    out(M.rows + 3 + i, j) = M(St::kPos + i, j);
                }
            return out;
        };
        T_ = clone(T_);
        for (auto& D : D_)
            D = clone(D);
        add_defect(LeakSource::S3_augmentation, clone(unobservable_basis(before)), after);
        ++tot_.s3_clones;
        leave(after);
    }

    void on_s6(const St& before,
               const sdk::msckf::CameraUpdater<T>& upd,
               std::uint64_t /*feature_id*/,
               const sdk::msckf::FeatureTrack<T>& track,
               const sdk::msckf::stages::s6_msckf_update::Diagnostics<T>& diag,
               const St& after) override {
        enter(before);
        if (diag.accepted()) {
            const auto prep = sdk::msckf::stages::s6_msckf_update::prepare(before, upd, track);
            if (!prep.ready) {
                ++tot_.prep_mismatch;
            } else {
                const auto& Hs = prep.measurement.H;
                Mat H(Hs.rows, Hs.cols);
                const double w = 1.0 / std::sqrt(static_cast<double>(upd.measurement_variance()));
                for (std::size_t i = 0; i < H.d.size(); ++i)
                    H.d[i] = w * static_cast<double>(Hs.d[i]);
                credit(H, unobservable_basis(before));
                ++tot_.s6_applied;
            }
            // The update moved the mean; the basis it leaves behind is T's.
            add_defect(LeakSource::S6_mean_shift, unobservable_basis(before), after);
        } else {
            ++tot_.s6_skipped;
        }
        leave(after);
    }

    void on_s9(const St& before,
               std::size_t clone_index,
               const sdk::msckf::stages::s9_marginalization::Diagnostics& /*diag*/,
               const St& after) override {
        enter(before);
        const std::size_t off = before.clone_offset(clone_index);
        auto drop = [&](const Mat& M) {
            Mat out(M.rows - St::kCloneDim, M.cols);
            std::size_t r = 0;
            for (std::size_t i = 0; i < M.rows; ++i) {
                if (i >= off && i < off + St::kCloneDim)
                    continue;
                for (std::size_t j = 0; j < M.cols; ++j)
                    out(r, j) = M(i, j);
                ++r;
            }
            return out;
        };
        T_ = drop(T_);
        for (auto& D : D_)
            D = drop(D);
        add_defect(LeakSource::S9_marginalization, drop(unobservable_basis(before)), after);
        ++tot_.s9_marginalized;
        leave(after);
    }

    [[nodiscard]] const Totals& totals() const noexcept {
        return tot_;
    }
    [[nodiscard]] const std::vector<Row>& rows() const noexcept {
        return rows_;
    }
    [[nodiscard]] bool started() const noexcept {
        return started_;
    }
    /// The transported basis T and one stage's transported defect (for tests).
    [[nodiscard]] const Mat& transported() const noexcept {
        return T_;
    }
    [[nodiscard]] const Mat& defect(LeakSource s) const noexcept {
        return D_[static_cast<std::size_t>(s)];
    }

    /// The filter's world-frame yaw σ (rad) of the IMU orientation at `s`:
    /// √(ẑᵀ R P_θθ Rᵀ ẑ), with δθ the right perturbation.
    [[nodiscard]] static double yaw_sigma(const St& s) {
        const auto P = s.covariance();
        const auto M = s.R.matrix();
        double g[3];  // Rᵀẑ
        for (std::size_t i = 0; i < 3; ++i)
            g[i] = static_cast<double>(M(2, i));
        double v = 0.0;
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j)
                v += g[i] * static_cast<double>(P(St::kTheta + i, St::kTheta + j)) * g[j];
        return std::sqrt(std::max(v, 0.0));
    }

    /// The one-screen summary vio_pipeline prints.
    void report(std::ostream& os) const {
        const auto& t = tot_;
        os << "  observability ledger (#480): spurious information along the unobservable directions\n";
        if (!started_) {
            os << "    the filter never initialized; nothing to report\n";
            return;
        }
        os << "    boundaries      : " << t.s2_steps << " S2, " << t.s3_clones << " S3, " << t.s6_applied
           << " S6 applied (" << t.s6_skipped << " not), " << t.s9_marginalized << " S9";
        if (t.restarts)
            os << ", " << t.restarts << " restart(s)";
        if (t.prep_mismatch)
            os << ", " << t.prep_mismatch << " replay mismatch(es)";
        os << "\n    closure check   : " << t.closure_max << "  (T vs N(x) + sum D; roundoff expected)\n";
        os << "    spurious info   : yaw " << t.info.yaw << " rad^-2, position " << t.info.pos
           << " m^-2  (no process-noise forgetting: upper bounds)\n";
        os << "    filter yaw sigma: start " << t.yaw_sigma_start << ", min " << t.yaw_sigma_min << ", end "
           << t.yaw_sigma_end << " rad; the leak alone could pull it to " << t.yaw_sigma_floor() << " rad\n";
        os << "    stage                share yaw   share pos   alone yaw   alone pos\n";
        // A position total at roundoff has no meaningful split.
        const double pos_whole = t.info.pos > 1e-12 * std::max(1.0, t.info.yaw) ? t.info.pos : 0.0;
        for (std::size_t s = 0; s < kLeakSources; ++s) {
            const auto name = leak_source_name(static_cast<LeakSource>(s));
            os << "    " << name << std::string(21 - std::min<std::size_t>(name.size(), 20), ' ')
               << pct(t.share[s].yaw, t.info.yaw) << pct(t.share[s].pos, pos_whole) << sci(t.alone[s].yaw)
               << sci(t.alone[s].pos) << "\n";
        }
    }

    /// Per-frame CSV: t, cumulative yaw/pos information, each stage's yaw share, yaw σ.
    /// Full round-trip precision: EuRoC frame times are Unix-epoch seconds (~1.4e9).
    void write_csv(std::ostream& os) const {
        const auto flags = os.flags();
        const auto prec = os.precision(std::numeric_limits<double>::max_digits10);
        os << "t,info_yaw,info_pos";
        for (std::size_t s = 0; s < kLeakSources; ++s)
            os << ",share_yaw_" << leak_source_name(static_cast<LeakSource>(s));
        os << ",yaw_sigma\n";
        for (const auto& r : rows_) {
            os << r.t << ',' << r.info.yaw << ',' << r.info.pos;
            for (const auto& sh : r.share)
                os << ',' << sh.yaw;
            os << ',' << r.yaw_sigma << '\n';
        }
        os.precision(prec);
        os.flags(flags);
    }

private:
    /// At the start of a boundary: start, restart on a re-seeded state, and book
    /// any change no tap reported since the last boundary.
    void enter(const St& before) {
        if (!started_ || before.dim() != last_N_.rows) {
            if (started_)
                ++tot_.restarts;
            started_ = true;
            T_ = unobservable_basis(before);
            for (auto& D : D_)
                D = Mat(T_.rows, T_.cols);
            last_N_ = T_;
            if (tot_.restarts == 0)
                tot_.yaw_sigma_start = yaw_sigma(before);
            return;
        }
        const Mat N = unobservable_basis(before);
        auto& D = D_[static_cast<std::size_t>(LeakSource::untapped)];
        for (std::size_t i = 0; i < D.d.size(); ++i)
            D.d[i] += last_N_.d[i] - N.d[i];
    }

    /// At the end of a boundary: record N(x̂⁺) for the next one, check closure.
    void leave(const St& after) {
        last_N_ = unobservable_basis(after);
        double worst = 0.0;
        for (std::size_t i = 0; i < T_.d.size(); ++i) {
            double s = last_N_.d[i];
            for (const auto& D : D_)
                s += D.d[i];
            worst = std::max(worst, std::abs(T_.d[i] - s));
        }
        tot_.closure_max = std::max(tot_.closure_max, worst);
        const double ys = yaw_sigma(after);
        tot_.yaw_sigma_end = ys;
        tot_.yaw_sigma_min = std::min(tot_.yaw_sigma_min, ys);
    }

    /// D_source += carried − N(x̂⁺): the part of the carried basis the new mean's
    /// basis does not reproduce.
    void add_defect(LeakSource source, const Mat& carried, const St& after) {
        const Mat N = unobservable_basis(after);
        auto& D = D_[static_cast<std::size_t>(source)];
        for (std::size_t i = 0; i < D.d.size(); ++i)
            D.d[i] += carried.d[i] - N.d[i];
    }

    /// Book one applied update's information. `Hw` is R^-½ H.
    void credit(const Mat& Hw, const Mat& N_before) {
        const Mat c = sdk::msckf::mul(Hw, T_);
        auto split = [](const Mat& a, const Mat& b) {
            Split s;
            for (std::size_t i = 0; i < a.rows; ++i) {
                for (std::size_t j = 0; j < 3; ++j)
                    s.pos += a(i, j) * b(i, j);
                s.yaw += a(i, 3) * b(i, 3);
            }
            return s;
        };
        auto add = [](Split& into, const Split& x) {
            into.yaw += x.yaw;
            into.pos += x.pos;
        };
        add(tot_.info, split(c, c));
        for (std::size_t s = 0; s < kLeakSources; ++s) {
            const Mat cs = s == static_cast<std::size_t>(LeakSource::S6_projection) ? sdk::msckf::mul(Hw, N_before)
                                                                                    : sdk::msckf::mul(Hw, D_[s]);
            add(tot_.share[s], split(c, cs));
            add(tot_.alone[s], split(cs, cs));
        }
    }

    static std::string pct(double part, double whole) {
        char buf[16];
        if (!(whole > 0.0))
            std::snprintf(buf, sizeof buf, "%10s  ", "-");
        else
            std::snprintf(buf, sizeof buf, "%9.1f%%  ", 100.0 * part / whole);
        return buf;
    }
    static std::string sci(double x) {
        char buf[16];
        std::snprintf(buf, sizeof buf, "%10.3g  ", x);
        return buf;
    }

    bool started_ = false;
    Mat T_;                            ///< the transported basis
    std::array<Mat, kLeakSources> D_;  ///< per-stage transported defects (S6_projection stays 0)
    Mat last_N_;                       ///< N(x̂) after the last boundary
    Totals tot_{};
    std::vector<Row> rows_;
};

}  // namespace branes::tools

#endif  // BRANES_TOOLS_OBSERVABILITY_LEDGER_HPP
