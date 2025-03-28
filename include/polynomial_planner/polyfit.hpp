#include <Eigen/Dense>
#include <cassert>
#include <iostream>
#include <numeric>
#include <vector>

namespace polyfit {

// Fits a polynomial of the requested degree to the given x, y values and
// returns the coefficients for {x^0, ... x^{degree}}.
inline std::vector<double> FitPolynomial(const std::vector<double>& x, const std::vector<double>& y, int degree) {
    assert(x.size() == y.size());
    // This maps the doubles in x to something that behaves like a const
    // Eigen::VectorXd object so we can use Eigen functionality on the underlying
    // data.
    auto xvec = Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());

    // Create a matrix with one row for each observation where xs(i, j) = x^j.
    // For example, if x = {1, 2, 3, 4}, then:
    // xs =
    //   1  1  1  1
    //   1  2  4  8
    //   1  3  9 27
    //   1  4 16 64
    Eigen::MatrixXd xs(x.size(), degree + 1);
    xs.col(0).setOnes();
    for (int i = 1; i <= degree; ++i) {
        xs.col(i).array() = xs.col(i - 1).array() * xvec.array();
    }

    // Map y to an object ys that behaves like an Eigen::VectorXd.
    auto ys = Eigen::Map<const Eigen::VectorXd>(y.data(), y.size());

    std::vector<double> result(degree + 1);
    // Again we use Eigen::Map to enable treating a std::vector<double> as an
    // Eigen object (this time a non-const one since we need to write to it).
    auto result_map = Eigen::Map<Eigen::VectorXd>(result.data(), result.size());

    // Compute a decomposition of the matrix xs; in this case we are using a QR
    // decomposition computed via Householder reflections. There are other
    // decompositions that can be used here as well with differing accuracy and
    // performance characteristics. For a list, see
    // https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
    //  and
    // https://eigen.tuxfamily.org/dox/group__TopicLinearAlgebraDecompositions.html
    // Note that this step is expensive and if you may ever be trying to set the
    // same set of x values to different y values, then you would want to compute
    // this value once and reuse it. This would be the case if, for example, you
    // always sample some real world data at the same x points and want to fit
    // a polynomial based on those xs and the observed ys.
    auto decomposition = xs.householderQr();
    result_map = decomposition.solve(ys);
    return result;
}
}  // namespace polyfit