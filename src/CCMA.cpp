
#pragma once

#include "polynomial_planner/CCMA.hpp"

#include <numeric>
using namespace ccma;
using std::vector, std::string, Eigen::MatrixXd, Eigen::VectorXd;

/*	POINTS MATRIX LOOKS LIKE THIS: CHATGPT FORGETS THIS SOMETIMES AND MAKES IT VERTICAL BUT NO! IT IS LIKE THIS! I SWEAR!
* 	Z is along for the ride fyi 
*	[	x,	x,	x,	x,	x,	x,	x,	x,	x,	x,	x,	x,	x,	...
*		y,	y,	y,	y,	y,	y,	y,	y,	y,	y,	y,	y,	y,	...
*		z,	z,	z,	z,	z,	z,	z,	z,	z,	z,	z,	z,	z,	...	]
* 
*	WHEN WE WANT TO DO THE CONVOLUTION, WE TAKE SLICES THAT ARE THE WIDTH OF THE THING WE ARE DOTTING (AKA THE WEIGHTS)
* 
*	[	x1,	x2,	x3,	x4			[	w1
*		y1,	y2,	y3,	y4		*		w2
*		z1,	z2,	z3,	z4	]			w3
*									w4	]
*
*	SO WE GET
* 
*	[	xv*wv
*		yv*wv
*		zv*wv	]
*
*
*	
*	Type naming convention:
* 
*	MatrixXd << X dimension aka dynamic, d for double
*	Matrix3d << 3x3 of doubles
*	VectorXd << dynamic dimensions, doubles. one column.
*	There is a templated constructor needed
*	Transposes exist but there's a comment against using them
*
*
*
*/

// Convert vector<cv::Point2d> to Eigen::MatrixXd
Eigen::MatrixXd CCMA::points_to_MatrixXd(const std::vector<cv::Point2d>& points) {
    const size_t n = points.size();
    Eigen::MatrixXd mat(3, n);  // n rows, 3 columns (x, y, z)

    for (size_t i = 0; i < n; ++i) {
        const cv::Point2d& pt = points[i];
        mat(0, i) = pt.x;  // x-coordinate
        mat(1, i) = pt.y;  // y-coordinate
        mat(2, i) = 0.0;   // z-coordinate (always 0 for 2D points)
    }

    return mat;
}

// Convert Eigen::MatrixXd to vector<cv::Point2d>
std::vector<cv::Point2d> CCMA::matrixXd_to_Points2d(const Eigen::MatrixXd& matrix) {
    std::vector<cv::Point2d> points;
    points.reserve(matrix.cols());  // Number of columns = number of points

    for (int i = 0; i < matrix.cols(); ++i) {
        points.emplace_back(matrix(0, i), matrix(1, i));
    }
    return points;
}

vector<vector<double>> CCMA::generate_weights(int width, const string& distribution, double crit_z_val) {
    vector<vector<double>> weight_list;
    weight_list.reserve(width + 1);
    if (distribution == "normal") {
        crit_z_val = fabs(crit_z_val);  // assuring we don't get these backward by mistake
        double x_start = -crit_z_val;
        double x_end = crit_z_val;

        for (int wi = 0; wi <= width;
             ++wi) {             // loops through each width until width is reached, can probably be removed
            int k = 2 * wi + 1;  // kernel width
            vector<double> weights(k, 0.0);
            vector<double> x(k + 1);
            for (int i = 0; i <= k; i++) {
                x[i] = x_start + (x_end - x_start) * i / k;
            }

            for (int i = 0; i < k; i++) {
                weights[i] = helpers::normal_cdf(x[i + 1]) - helpers::normal_cdf(x[i]);
            }

            double sum = 0;
            for (int i = 0; i < weights.size(); i++) {
                sum += weights[i];
            }
            for (double& w : weights) w /= sum;
            weight_list.emplace_back(weights);
        }
    } else if (distribution == "uniform") {
        for (int wi = 0; wi <= width; wi++) {
            int k = 2 * wi + 1;
            weight_list.emplace_back(k, 1.0 / k);
        }
    } else if (distribution == "pascal") {
        auto pascal_row = [](int n) {
            vector<double> row{1};

            for (int r = 1; r <= n; r++) {
                row.push_back(row.back() * static_cast<double>(n - r + 1) / r);  // neat algorithm
            }

            return row;
        };

        for (int wi = 0; wi <= width; wi++) {
            auto row = pascal_row(2 * wi);
            double sum = std::accumulate(row.begin(), row.end(), 0.0);

            for (double& v : row) v /= sum;

            weight_list.emplace_back(std::move(row));
        }
    }

    else
        return (generate_weights(width, "pascal", crit_z_val));

    return weight_list;
};

MatrixXd CCMA::ma_points(const Eigen::MatrixXd& points, const vector<double>& weights) const {
    int kernel_width = weights.size();
    int last_index = points.cols() - kernel_width;

    // e.g. if we have a 3x3 matrix and we have 2 weights (for some reason),
    // we do one ma at col 1 (index 0, covers col 1 and 2)
    // and one at col 2 (index 1,	   covers col 2 and 3).

    Eigen::MatrixXd out(3, last_index + 1);
    VectorXd w = Eigen::Map<const VectorXd>(weights.data(), kernel_width);

    for (int column = 0; column < last_index + 1; column++) {  // v not off by one?
        for (int row = 0; row < points.rows(); row++) {
            out(row, column) = points.row(row).segment(column, kernel_width).dot(w);
        }
    }  // this had to be flipped from gpt implementation because dot products don't work the other way around and there's no transpose (without warning lmao)

    return out;
}

MatrixXd CCMA::curvature_vectors(
    const Eigen::MatrixXd& points) {  // this might not need to be MatrixXd, see if it can be replaced with a vector<double>
                               // this might not need to be curvature, see if it can be replaced with radii.
    Eigen::MatrixXd kv = Eigen::MatrixXd::Zero(3, points.cols());

    for (int i = 1; i < points.rows() - 1; i++) {  // don't index outermost points
        const Eigen::Vector3d& previous_point = points.col(i - 1);
        const Eigen::Vector3d& this_point = points.col(i);
        const Eigen::Vector3d& next_point =
            points.col(i + 1);  // i have no idea if this is more or less readable than just using indexes lmao

        const Eigen::Vector3d vector_to_previous = previous_point - this_point;
        const Eigen::Vector3d vector_to_next = next_point - this_point;
        const Eigen::Vector3d vector_between_others = next_point - previous_point;
        const Eigen::Vector3d prev_next_cross =
            vector_to_previous.cross(vector_to_next);  // hopefully makes vector math more obvious
        const double prev_next_cross_mag = prev_next_cross.norm();

        double k = 0.0;
        if (prev_next_cross_mag != 0.0) {
            double r = -vector_to_previous.norm() * vector_to_next.norm() * vector_between_others.norm() /
                       (2.0 * prev_next_cross_mag);
            // ^ radius can be negative, because we want curvature to be negative and we want signs in various other places. if you want actual r take mag.
            k = 1 / r;
        }
        kv(2, i) = k;
    }

    return kv;
}

vector<double> CCMA::alphas(const Eigen::MatrixXd& points, const vector<double>& curvatures) {
    int width = points.cols();
    vector<double> angles(width, 0.0);

    for (int i = 1; i < width - 1; i++) {
        if (curvatures[i] != 0) {
            double r = 1 / curvatures[i];
            double d = (points.col(i - 1) - points.col(i + 1)).norm();
            angles[i] = std::asin(0.5 * d / r);
        }
    }

    return angles;
}

vector<double> CCMA::normalized_ma_radii(const vector<double>& alphas, int w_ma, const vector<double>& weights) {
    int n = alphas.size();
    vector<double> radii(n, 0.0);

    for (int i = 1; i < n - 1; i++) {
        double r = weights[w_ma];

        for (int k = 1; k <= w_ma; k++) {
            r += 2 * std::cos(alphas[i] * k) * weights[w_ma + k];  // consult the magic paper for math
        }

        radii[i] = std::max(0.35, r);  // do we need this max function?
    }

    return radii;
}

Eigen::MatrixXd CCMA::_filter(const Eigen::MatrixXd& points, int w_ma, int w_cc, bool cc_mode) const {
    int w_ccma = w_ma + w_cc + 1;

    Eigen::MatrixXd points_ma = ma_points(points, weights_ma_[w_ma]);

    if (!cc_mode) return points_ma;

    Eigen::MatrixXd cv = curvature_vectors(points_ma);
    vector<double> curvatures(cv.cols());

    for (int i = 0; i < cv.cols(); i++) {
        curvatures[i] = cv.col(i).norm();  // shouldn't this just get the z coordinate??
    }

    vector<double> angles = alphas(points_ma, curvatures);
    vector<double> radii_ma = normalized_ma_radii(angles, w_ma, weights_ma_[w_ma]);

    int n_out = points.cols() - 2 * w_ccma;
    Eigen::MatrixXd out(3, n_out);

    for (int i = 0; i < n_out; i++) {
        Eigen::Vector3d unit_tangent = helpers::unit(
            points_ma.col(w_cc + i + 2) - points_ma.col(w_cc + i));  // adjacent columns to the w_cc+i th column
        Eigen::Vector3d shift = Eigen::Vector3d::Zero();

        for (int j = 0; j < 2 * w_cc + 1; j++) {  // iterates through all of the cc weights

            int idx = i + j + 1;

            if (curvatures[idx] == 0) continue;  // skip if straight line

            Eigen::Vector3d curvature_unit_vector = helpers::unit(cv.col(idx));
            double weight = weights_cc_[w_cc][j];  // grab the jth weight from the w_cc weights width
            double shift_mag = (1 / curvatures[idx]) * (1 / radii_ma[idx] - 1);
            shift += curvature_unit_vector * weight * shift_mag;
        }

        out.col(i) =
            points_ma.col(i + w_cc + 1) +
            unit_tangent.cross(
                shift);  // gets the normal to the vector between the adjacent points and multiplies it by the shift and shifts the original point.
    }

    return out;
}

/*MatrixXd fill_boundary_mode(const MatrixXd& points, bool cc_active) const {
	int dim = points.rows();
	MatrixXd out = MatrixXd::Zero(points.rows(), dim);							// only needed if we want to use edge filling which keeps the same amount of points, without this we lose ~ w_ccma points from either end, but we have a ton of points so this shouldn't really matter. Can be implemented if needed.
}*/

Eigen::MatrixXd CCMA::filter(const Eigen::MatrixXd& points, const string& mode, bool cc_mode) {
    // PADDING AND WRAPPING NOT IMPLEMENTED BECAUSE WE SHOULDN'T NEED THEM, PADDING MIGHT BE NICE TO HAVE BUT WE NEED TO MAKE SURE THIS WORKS

    if (points.cols() < (cc_mode ? w_ccma_ * 2 + 1 : w_ma_ * 2 + 1)) {
        // since we need more than our width as points we just panic if we dont get them and return 0.
        return Eigen::MatrixXd::Zero(points.rows(), points.cols());
    }
    return _filter(points, w_ma_, w_cc_, cc_mode);
}
