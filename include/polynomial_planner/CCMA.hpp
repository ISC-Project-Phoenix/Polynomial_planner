
#ifndef CCMA_HPP
#define CCMA_HPP

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace ccma {
namespace helpers {

constexpr double kSQRT2 = 1.41421356237309504880168872420969807856967187537694;
inline double normal_cdf(double z) { return 0.5 * std::erfc(-z / kSQRT2); }

inline Eigen::VectorXd unit(const Eigen::VectorXd& v) {
    double n = v.norm();
    return n == 0 ? v : v / n;
}

inline void printMatrixXd(const Eigen::MatrixXd& matrix) {
    for (int row = 0; row < matrix.rows(); row++) {
        for (int col = 0; col < matrix.cols(); col++) {
            std::cout << matrix(row, col) << ", ";
        }
        std::cout << std::endl;
    }
}
}  // namespace helpers
class CCMA {
public:
    inline CCMA(int w_ma = 5, int w_cc = 3, std::string distrib = "pascal", std::string distrib_ma = "",
                std::string distrib_cc = "", double crit_z_ma = 2, double crit_z_cc = 2)
        : w_ma_(w_ma),
          w_cc_(w_cc),
          w_ccma_(w_ma + w_cc + 1),
          distrib_ma_(distrib_ma.empty() ? distrib : distrib_ma),  // unholy initializer list
          distrib_cc_(distrib_cc.empty() ? distrib : distrib_cc),
          crit_z_ma_(fabs(crit_z_ma)),
          crit_z_cc_(fabs(crit_z_cc)) {
        weights_ma_ = generate_weights(w_ma_, distrib_ma_, crit_z_ma_);  // generates the weights algorithmically
        weights_cc_ = generate_weights(w_cc_, distrib_cc_, crit_z_cc_);
    };

    Eigen::MatrixXd filter(const Eigen::MatrixXd& input, const std::string& mode = "none",
                           bool cc_mode = true);  // the only public funciton

    Eigen::MatrixXd points_to_MatrixXd(const std::vector<cv::Point2d>& points);  // lies, deception ^
    std::vector<cv::Point2d> matrixXd_to_Points2d(const Eigen::MatrixXd& matrix);

private:
    /// <summary>
    /// Generates every kernel (vector of weights) up to the given width. Should likely use w_ma/cc when grabbing list of weights.
    /// GPT says it's faster on runtime instead of recalculating every run. Maybe we should only save the one we want since we aren't changing widths?
    /// </summary>
    /// <param name="width">: The width of the desired weight list</param>
    /// <param name="distribution">: The type of distribution. May be: normal uniform pascal AND NOT hanning i got bored
    /// </param>
    /// <param name="crit_z_val">: Desired z-val of the normal distribution. Only works if normal is picked, unused otherwise.</param>
    /// <returns> Vector of every kernel up to and including width, which will have 2(width) +-? 1 values, and will be stored at [width - 1]. </returns>
    static std::vector<std::vector<double>> generate_weights(int width, const std::string& distribution,
                                                             double crit_z_val);

    /// <summary>
    /// Moving average points. Convolution along all 3 axes with the provided weights.
    /// </summary>
    /// <param name="points">: The matrix of points.</param>
    /// <param name="weights">: The vector of weights.</param>
    /// <returns>The convoluted matrix. <strong>This will be a differently-sized matrix</strong>!</returns>
    Eigen::MatrixXd ma_points(const Eigen::MatrixXd& points, const std::vector<double>& weights) const;

    /// <summary>
    ///
    /// Curvatures of the circle inscribed by each set of 3 points.	Visual in comments
    /// look at function for proper structuring this is so frustratingly awful to get to linebreak
    ///
    /// </summary>
    /// <param name="points"></param>
    /// <returns></returns>
    static Eigen::MatrixXd curvature_vectors(const Eigen::MatrixXd& points);

    /*	[	0,	0,	0,	...	0
				0,	0,	0,	...	0
				0,	k1,	k2,	...	0	]*/

    /// <summary>
    /// Angles between adjacent points for each point. Starts with the second point and ends with the second to last point.
    /// </summary>
    /// <param name="points"></param>
    /// <param name="curvatures">, could probably just use a function call natively, not sure why it doesn't</param>
    /// <returns>angles between adjacent points</returns>
    static std::vector<double> alphas(const Eigen::MatrixXd& points, const std::vector<double>& curvatures);

    /// <summary>
    /// weights for the radii in a similar way that we do for moving average. slightly weirder though.
    /// </summary>
    /// <param name="alphas">angles between points</param>
    /// <param name="w_ma"></param>
    /// <param name="weights"></param>
    /// <returns></returns>
    static std::vector<double> normalized_ma_radii(const std::vector<double>& alphas, int w_ma,
                                                   const std::vector<double>& weights);

    /// <summary>
    /// one window-size filter. why not use this?
    /// </summary>
    /// <param name="points"></param>
    /// <param name="w_ma"></param>
    /// <param name="w_cc"></param>
    /// <param name="cc_mode">: whether we are using curvature correction</param>
    /// <returns>filtered matrix</returns>
    Eigen::MatrixXd _filter(const Eigen::MatrixXd& points, int w_ma, int w_cc, bool cc_mode) const;

    /// <summary>
    /// readjusts to regular size, with extrapolation reduction algorithms
    /// </summary>
    /// <param name="points"></param>
    /// <param name="cc_active"></param>
    /// <returns></returns>
    //	Eigen::MatrixXd fill_boundary_mode(const Eigen::MatrixXd& points, bool cc_active) const;

    // members
    int w_ma_, w_cc_, w_ccma_;             // width of the weights
    std::string distrib_ma_, distrib_cc_;  // type of distribution
    double crit_z_ma_,
        crit_z_cc_;  // instead of using crit p vals, we use crit z vals to get around needing prob functions
    std::vector<std::vector<double>> weights_ma_, weights_cc_;  // a vector of vectors of doubles for weights!
};
}  // namespace ccma

#endif
