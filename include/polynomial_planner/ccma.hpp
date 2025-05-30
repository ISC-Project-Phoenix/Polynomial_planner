/***********************************************************************************************
 *  CURVATURE-CORRECTED MOVING-AVERAGE  ( C C M A )
 *
 *  A **model-free** path-smoothing algorithm for 2-D / 3-D data that avoids the typical
 *  inward-bending artefact of a plain moving average by *explicit* curvature compensation.
 *
 *  This header is the literal C++17 translation of the official Python reference
 *  implementation (Steinecker & Wuensche, IV 2023) – **but** every relevant line is
 *  annotated with two kinds of comments:
 *
 *      WHAT → explains *what* the statement does.
 *      WHY  → explains *why* that design/implementation decision was taken.
 *
 *  Build test-program (with Eigen in include-path):
 *      g++ -std=c++17 -O3 example.cpp -I /usr/include/eigen3
 *
 **********************************************************************************************/
#ifndef CCMA_HPP
#define CCMA_HPP

/*───────────────────────────────────────────────────────────────────────────────*
 *  WHAT:  Standard library & Eigen headers.                                     *
 *  WHY :  • Eigen supplies vector/matrix math without external libs.            *
 *         • <cmath>  : math functions (sin, cos, log, etc.).                    *
 *         • <numeric>: std::accumulate for kernel normalisation.                *
 *───────────────────────────────────────────────────────────────────────────────*/
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <opencv2/core/types.hpp>

namespace ccma {
/*=============================================================================*
 *                               Helper utilities                              *
 *============================================================================*/
namespace detail {
/* WHAT:  √2 constant – used in the error-function based normal CDF.            */
constexpr double kSQRT2 = 1.41421356237309504880168872420969808;

/*---------------------------------------------------------------------------*/
/*  Φ(x)  – Standard-normal cumulative distribution function.                 */
/*  WHAT:  wrapper around std::erfc so we avoid <random> heavyweight.         */
/*  WHY :  fastest portable implementation in <cmath>.                        */
inline double normal_cdf(double x) noexcept { return 0.5 * std::erfc(-x / kSQRT2); }

/*---------------------------------------------------------------------------*/
/*  unit(v) – returns v/‖v‖ (or v if ‖v‖==0).                                 *
 *  WHY:  frequent normalisations; inline avoids function-call overhead.      */
inline Eigen::Vector3d unit(const Eigen::Vector3d& v) noexcept {
    double n = v.norm();
    return n == 0.0 ? v : v / n;
}

/*---------------------------------------------------------------------------*/
/*  Manual *valid-length* 1-D convolution (kernel shorter than data).         *
 *  WHAT:  reproduces numpy.convolve(..., mode="valid") exactly.              *
 *  WHY :  Eigen lacks an easy 1-D convolution; writing loop is trivial &     *
 *         avoids another dependency.                                         */
inline std::vector<double> convolve_valid(const std::vector<double>& data, const std::vector<double>& kernel) {
    std::size_t n = data.size(), k = kernel.size();
    if (n < k) throw std::runtime_error("convolve_valid: kernel longer than data");

    std::vector<double> res(n - k + 1, 0.0);
    for (std::size_t i = 0; i <= n - k; ++i)
        for (std::size_t j = 0; j < k; ++j) res[i] += data[i + j] * kernel[j];
    return res;
}

}  // namespace detail

/*=============================================================================*
 *                                  CCMA class                                 *
 *============================================================================*/
class CCMA {
public:
    /*-------------------------------------------------------------------------*/
    /*  Constructor – stores hyper-parameters *and* pre-computes kernels.       *
     *  WHY:  weights are independent of data → compute once, reuse often.      */
    CCMA(int w_ma = 5,                   /* half-window for moving avg  */
         int w_cc = 3,                   /* half-window for curvature  */
         std::string distrib = "pascal", /* default kernel family      */
         std::string distrib_ma = "",    /* override for MA (opt)      */
         std::string distrib_cc = "",    /* override for CC (opt)      */
         double rho_ma = 0.95,           /* truncation area (normal)   */
         double rho_cc = 0.95)
        : w_ma_(w_ma),
          w_cc_(w_cc),
          w_ccma_(w_ma + w_cc + 1) /* larger composite window     */
          ,
          distrib_ma_(distrib_ma.empty() ? distrib : distrib_ma),
          distrib_cc_(distrib_cc.empty() ? distrib : distrib_cc),
          rho_ma_(rho_ma),
          rho_cc_(rho_cc) {
        /* WHAT:  generate lookup tables of kernels for all window sizes ≤ w.   */
        weights_ma_ = generate_weights(w_ma_, distrib_ma_, rho_ma_);
        weights_cc_ = generate_weights(w_cc_, distrib_cc_, rho_cc_);
    }

    /*-------------------------------------------------------------------------*/
    /*  Public API: filter()                                                   */
    /*  WHAT:  accepts raw points, returns smoothed points.                    *
     *  WHY :  hides boundary handling + 2-D/3-D up-conversion details.        */
     std::vector<cv::Point2d> filter(const std::vector<cv::Point2d>& input, const std::string& mode = "padding",
                           bool cc_mode = true) const {
        /*———— Sanity checks ————*/
        if (mode != "none" && mode != "padding" && mode != "wrapping" && mode != "fill_boundary")
            throw std::invalid_argument("CCMA::filter: invalid mode");

        if (input.size() < 3) throw std::runtime_error("CCMA::filter: at least 3 points required");

        /* copy because we may extend/modify pts for boundary treatment        */
        Eigen::MatrixXd pts(3, input.size());

        for (int i = 0; i < input.size(); i++) {
            pts(0,i) = input.at(i).x;
            pts(1,i) = input.at(i).y;
            pts(2,i) = 0;
        }

        /*======================== 1. Boundary pre-processing =================*/
        // if (mode == "padding") {
        //     /* WHAT: replicate first/last sample to keep length unchanged.     *
        //      * WHY : simplest length-preserving strategy when data are *open*.*/
        //     int n_pad = cc_mode ? w_ccma_ : w_ma_;
        //     pts.conservativeResize(pts.rows() + 2 * n_pad, pts.cols());
        //     pts.block(0, 0, n_pad, pts.cols()) = pts.row(n_pad).replicate(n_pad, 1);
        //     pts.block(pts.rows() - n_pad, 0, n_pad, pts.cols()) = pts.row(pts.rows() - n_pad - 1).replicate(n_pad, 1);
        // } else if (mode == "wrapping") {
        //     /* WHAT: treat data as cyclic.                                      *
        //      * WHY : useful for closed curves (e.g. circle, track loop).       */
        //     int n_pad = cc_mode ? w_ccma_ : w_ma_;
        //     Eigen::MatrixXd tmp(pts.rows() + 2 * n_pad, pts.cols());
        //     tmp.block(0, 0, n_pad, pts.cols()) = pts.bottomRows(n_pad);
        //     tmp.block(n_pad, 0, pts.rows(), pts.cols()) = pts;
        //     tmp.block(n_pad + pts.rows(), 0, n_pad, pts.cols()) = pts.topRows(n_pad);
        //     pts.swap(tmp);
        // }

        /* ensure enough points remain after trimming (for “none” mode)        */
        if (pts.cols() < (cc_mode ? w_ccma_ * 2 + 1 : w_ma_ * 2 + 1))
            throw std::runtime_error("CCMA::filter: not enough points for given widths");

        /*======================== 2. 2-D → 3-D up-conversion =================*/                       // should never run
        // bool is2d = (pts.cols() == 2);
        // if (is2d) {
        //     /* WHY: core math implemented in 3-D; simply append z=0 column.    */
        //     Eigen::MatrixXd tmp(pts.rows(), 3);
        //     tmp.block(0, 0, pts.rows(), 2) = pts;
        //     tmp.col(2).setZero();
        //     pts.swap(tmp);
        // }

        /*======================== 3. Filter core =============================*/
        Eigen::MatrixXd out;

        if (mode != "fill_boundary") {
            /* full-size kernel everywhere                                     */
            out = _filter(pts, w_ma_, w_cc_, cc_mode);
        } else {
            /* cascading smaller kernels near edges                            */
            out = fill_boundary_mode(pts, cc_mode);
        }

        std::vector<cv::Point2d> ret;
        for (int i = 0; i < out.cols(); i++) {
            cv::Point2d toPush(out(0,i), out(1,i));
            ret.push_back(toPush);        
        }

        /*======================== 4. Return shape as input ===================*/
        return ret;
    }

    /*============================================================================*
 *                                  Internals                                 *
 *============================================================================*/
private:
    /*-------------------------------------------------------------------------*/
    /*  generate_weights() – produces *all* kernels up to window size w.       *
     *  WHY:  pre-computing dramatically speeds repeated filtering.            */
    static std::vector<std::vector<double>> generate_weights(int w, const std::string& distrib, double crit_z) {
        std::vector<std::vector<double>> weight_list;
        weight_list.reserve(w + 1);

        /*================ Normal (truncated) kernel ==========================*/
        if (distrib == "normal") {
            /* WHAT:  boundaries that enclose ‘rho’ area under N(0,1).         */
            crit_z = std::fabs(crit_z);
            double x_start = -crit_z;
            double x_end = crit_z;

            for (int wi = 0; wi <= w; ++wi) {
                int k = 2 * wi + 1; /* kernel length*/
                std::vector<double> weights(k, 0.0);

                /* integrate PDF over slices → exactly matches Python impl.    */
                std::vector<double> x(k + 1);
                for (int i = 0; i <= k; ++i) x[i] = x_start + (x_end - x_start) * static_cast<double>(i) / k;

                for (int i = 0; i < k; ++i) weights[i] = detail::normal_cdf(x[i + 1]) - detail::normal_cdf(x[i]);

                /* renormalise to sum≈1 (truncation removed mass ‘1-rho’)      */
                for (double& v : weights) v /= (1 - detail::normal_cdf(crit_z) * 2);
                weight_list.emplace_back(std::move(weights));
            }
        }
        /*================ Uniform kernel ====================================*/
        else if (distrib == "uniform") {
            for (int wi = 0; wi <= w; ++wi) {
                int k = 2 * wi + 1;
                weight_list.emplace_back(k, 1.0 / static_cast<double>(k));
            }
        }
        /*================ Pascal (binomial) kernel ==========================*/
        else if (distrib == "pascal") {
            auto pascal_row = [](int n) {
                /* WHAT: computes row ‘n’ of Pascal’s triangle iteratively.    */
                std::vector<double> row{1.0};
                for (int r = 1; r <= n; ++r) row.push_back(row.back() * static_cast<double>(n - r + 1) / r);
                return row;
            };

            for (int wi = 0; wi <= w; ++wi) {
                auto row = pascal_row(2 * wi); /* even row length  */
                double sum = std::accumulate(row.begin(), row.end(), 0.0);
                for (double& v : row) v /= sum; /* normalise        */
                weight_list.emplace_back(std::move(row));
            }
        }
        /*================ Hanning (raised-cosine) kernel ====================*/
        else if (distrib == "hanning") {
            for (int wi = 0; wi <= w; ++wi) {
                int k = 2 * wi + 1;
                int window_size = k + 2; /* match Python impl*/
                std::vector<double> kernel;
                kernel.reserve(k);

                for (int n = 1; n <= k; ++n) {
                    double v = 0.5 * (1.0 - std::cos(2.0 * M_PI * n / (window_size - 1)));
                    kernel.push_back(v);
                }
                double sum = std::accumulate(kernel.begin(), kernel.end(), 0.0);
                for (double& v : kernel) v /= sum;
                weight_list.emplace_back(std::move(kernel));
            }
        } else {
            throw std::invalid_argument("unknown kernel distribution '" + distrib + "'");
        }
        return weight_list;
    }

    /*-------------------------------------------------------------------------*/
    /*  ma_points() – convolution along each axis with chosen MA kernel.       *
     *  WHAT:  returns *valid* samples only → easy index bookkeeping later.    *
     *  WHY :  explicit loops avoid Eigen’s heavy convolution workaround.      */
    inline static Eigen::MatrixXd ma_points(const Eigen::MatrixXd& points, const std::vector<double>& weights) {
        int k = (weights.size());
        int n_out = points.rows() - k ;
        Eigen::MatrixXd out(n_out, 3);

        Eigen::VectorXd w = Eigen::Map<const Eigen::VectorXd>(weights.data(), k);

        for (int i = 0; i < n_out; ++i) {                                                                       // IF CRASHES PLEASE LOOK HERE
            out(0,i) = points.row(0).segment(i, k).dot(w);
            out(1,i) = points.row(1).segment(i, k).dot(w);
            out(2,i) = points.row(2).segment(i, k).dot(w);                                                        
        }
        return out;
    }

    /*-------------------------------------------------------------------------*/
    /*  curvature_vectors() – approximates signed curvature normal for each pt.*/
    /*  WHAT:  uses circumcircle of triplets (i-1,i,i+1).                       *
     *  WHY :  local curvature needed to correct systematic MA shrinkage.      */
    inline static Eigen::MatrixXd curvature_vectors(const Eigen::MatrixXd& pts) {
        Eigen::MatrixXd cv = Eigen::MatrixXd::Zero(pts.rows(), 3);

        for (int i = 1; i < pts.rows() - 1; ++i) {
            Eigen::Vector3d v1 = pts.row(i) - pts.row(i - 1);
            Eigen::Vector3d v2 = pts.row(i + 1) - pts.row(i);
            Eigen::Vector3d cross = v1.cross(v2);
            double cross_norm = cross.norm();

            double curvature = 0.0;
            if (cross_norm != 0.0) {
                double radius = v1.norm() * v2.norm() * (pts.row(i + 1) - pts.row(i - 1)).norm() / (2.0 * cross_norm);
                curvature = 1.0 / radius;
            }
            cv.row(i) = curvature * detail::unit(cross);
        }
        return cv;
    }

    /*-------------------------------------------------------------------------*/
    /*  alphas() – angle between consecutive chord vectors under constant curv.*/
    inline static std::vector<double> alphas(const Eigen::MatrixXd& pts, const std::vector<double>& curv) {
        int n = pts.rows();
        std::vector<double> a(n, 0.0);

        for (int i = 1; i < n - 1; ++i) {
            if (curv[i] != 0.0) {
                double R = 1.0 / curv[i];
                double d = (pts.row(i + 1) - pts.row(i - 1)).norm();
                a[i] = std::asin(0.5 * d / R);
            }
        }
        return a;
    }

    /*-------------------------------------------------------------------------*/
    /*  normalized_ma_radii() – Eq. (7) in the paper (figure-2 intuition).     *
     *  WHY:  converts angles to *radius shrink-factor* for curvature fix.     */
    inline static std::vector<double> normalized_ma_radii(const std::vector<double>& alphas, int w_ma,
                                                          const std::vector<double>& weights) {
        int n = static_cast<int>(alphas.size());
        std::vector<double> radii(n, 0.0);

        for (int i = 1; i < n - 1; ++i) {
            double radius = weights[w_ma]; /* central weight (k=0)            */
            for (int k = 1; k <= w_ma; ++k) radius += 2.0 * std::cos(alphas[i] * k) * weights[w_ma + k];
            radii[i] = std::max(0.35, radius); /* lower-bound to avoid overshoot */
        }
        return radii;
    }

    /*-------------------------------------------------------------------------*/
    /*  _filter() – workhorse for *one* window size (possibly with CC).         */
    Eigen::MatrixXd _filter(const Eigen::MatrixXd& pts, int w_ma, int w_cc, bool cc_mode) const {
        int w_ccma = w_ma + w_cc + 1;

        /*——- Step 1: plain moving-average over pts ————————————————*/
        Eigen::MatrixXd pts_ma = ma_points(pts, weights_ma_[w_ma]);

        if (!cc_mode) return pts_ma; /* user disabled CC                */

        /*——- Step 2: curvature + correction vectors ———————————————*/
        Eigen::MatrixXd cv = curvature_vectors(pts_ma);
        std::vector<double> curv(cv.rows());
        for (int i = 0; i < cv.rows(); ++i) curv[i] = cv.row(i).norm();

        std::vector<double> a = alphas(pts_ma, curv);
        std::vector<double> radii_ma = normalized_ma_radii(a, w_ma, weights_ma_[w_ma]);

        /*——- Step 3: reconstruct curvature-corrected points ————————*/
        int n_out = pts.rows() - 2 * w_ccma;
        Eigen::MatrixXd out(n_out, 3);

        for (int i = 0; i < n_out; ++i) {
            /* tangent vector (for cross-product reconstruction)               */
            Eigen::Vector3d unit_tangent = detail::unit(pts_ma.row(w_cc + i + 2) - pts_ma.row(w_cc + i));

            Eigen::Vector3d shift = Eigen::Vector3d::Zero();

            for (int j = 0; j < 2 * w_cc + 1; ++j) {
                int idx = i + j + 1;            /* corresponding MA index       */
                if (curv[idx] == 0.0) continue; /* straight line → no shift     */

                Eigen::Vector3d u_vec = detail::unit(cv.row(idx));
                double weight = weights_cc_[w_cc][j];
                double shift_mag = (1.0 / curv[idx]) * (1.0 / radii_ma[idx] - 1.0);
                shift += u_vec * weight * shift_mag;
            }
            /* reconstruct by rotating shift into tangent plane                */
            out.col(i) = pts_ma.col(i + w_cc + 1) + unit_tangent.cross(shift);
        }
        return out;
    }

    /*-------------------------------------------------------------------------*/
    /*  fill_boundary_mode() – gradually shrinking window near ends.           *
     *  WHY:  keeps *length* unchanged while avoiding heavy extrapolation.     */
    Eigen::MatrixXd fill_boundary_mode(const Eigen::MatrixXd& pts, bool cc_active) const {
        int dim = 3;
        Eigen::MatrixXd out = Eigen::MatrixXd::Zero(pts.rows(), dim);

        if (cc_active) /* CCMA with shrinking windows           */
        {
            /* Build descending sequence of (w_ma,w_cc) pairs, largest first. */
            std::vector<std::pair<int, int>> widths;
            int w_ma_cur = w_ma_, w_cc_cur = w_cc_;
            while (w_ma_cur > 0 || w_cc_cur > 0) {
                (w_cc_cur >= w_ma_cur) ? --w_cc_cur : --w_ma_cur;
                widths.emplace_back(w_ma_cur, w_cc_cur);
            }
            std::reverse(widths.begin(), widths.end());

            /* Unchanged first/last original points                            */
            out.row(0) = pts.row(0);
            out.row(pts.rows() - 1) = pts.row(pts.rows() - 1);

            /* Fully filtered *interior*                                       */
            out.block(w_ccma_, 0, pts.rows() - 2 * w_ccma_, dim) = _filter(pts, w_ma_, w_cc_, true);

            /* Ascending/descending partial windows                            */
            for (std::size_t w_idx = 0; w_idx < widths.size(); ++w_idx) {
                int wma = widths[w_idx].first;
                int wcc = widths[w_idx].second;
                int wccma = wma + wcc + 1;

                /* Design choice: if w_ma==0 use MA=1 but no curvature corr.   */
                bool use_ma1 = (wma == 0 && w_ma_ != 0);

                /* Start side                                                  */
                out.row(w_idx + 1) =
                    _filter(pts.topRows(w_idx + 1 + wccma + 1), use_ma1 ? 1 : wma, wcc, use_ma1 ? false : true).row(0);

                /* End side                                                    */
                out.row(out.rows() - w_idx - 2) =
                    _filter(pts.bottomRows(w_idx + 1 + wccma + 1), use_ma1 ? 1 : wma, wcc, use_ma1 ? false : true)
                        .row(0);
            }
        } else /* MA-only variant with width-cascade                            */
        {
            out.block(w_ma_, 0, pts.rows() - 2 * w_ma_, dim) = _filter(pts, w_ma_, 0, false);

            for (int w = 0; w < w_ma_; ++w) {
                out.row(w) = _filter(pts.topRows(2 * w + 1), w, 0, false).row(0);

                out.row(out.rows() - w - 1) = _filter(pts.bottomRows(2 * w + 1), w, 0, false).row(0);
            }
        }
        return out;
    }

    /*============================================================================*
 *                               Data members                                 *
 *============================================================================*/
    int w_ma_, w_cc_, w_ccma_;                                 /* window params   */
    std::string distrib_ma_, distrib_cc_;                      /* kernel families */
    double rho_ma_, rho_cc_;                                   /* truncation area */
    std::vector<std::vector<double>> weights_ma_, weights_cc_; /* kernel LUTs */
};

}  // namespace ccma
#endif /* CCMA_HPP */
