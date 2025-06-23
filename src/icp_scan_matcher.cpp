// icp_scan_matcher.cpp
//
// Point-to-point 2D ICP for RPLIDAR A1 scans (360 deg, ~0.9 deg angular step,
// 8 m nominal range). Recovers the rigid transform between the previous and
// current scan in the odometry frame, then fuses it with wheel encoders
// upstream. Kabsch/SVD rotation recovery, trimmed correspondences so the
// pallet legs and the odd reflective vest do not drag the solution.
//
// Build: see Makefile. Depends on Eigen3 only.

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace slam {

struct Point2 {
  double x{0.0};
  double y{0.0};
  bool valid{false};
};

// Rigid transform, scan_prev -> scan_curr.
struct Transform2 {
  double x{0.0};
  double y{0.0};
  double theta{0.0};

  Eigen::Matrix3d matrix() const {
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    m(0, 0) = c;
    m(0, 1) = -s;
    m(1, 0) = s;
    m(1, 1) = c;
    m(0, 2) = x;
    m(1, 2) = y;
    return m;
  }
};

struct IcpParams {
  int max_iterations{40};
  double max_correspondence_dist{0.35};  // m, hard gate
  double trim_fraction{0.80};            // keep best 80% of pairs
  double convergence_eps{1e-5};          // on incremental translation
  int min_pairs{20};
};

struct IcpResult {
  Transform2 transform;
  int iterations{0};
  int pairs{0};
  double rmse{std::numeric_limits<double>::infinity()};
  bool converged{false};
};

namespace detail {

inline Eigen::Vector2d toVec(const Point2& p) { return {p.x, p.y}; }

inline Eigen::Vector2d apply(const Transform2& t, const Eigen::Vector2d& p) {
  const double c = std::cos(t.theta);
  const double s = std::sin(t.theta);
  return {c * p.x() - s * p.y() + t.x, s * p.x() + c * p.y() + t.y};
}

// Brute-force nearest neighbour. Scans are capped at 400 points after
// decimation so this is ~160k distance checks per iteration. If we ever move
// to full 4000-point scans, swap in a kd-tree; do not optimise before that.
inline int nearest(const std::vector<Point2>& cloud, const Eigen::Vector2d& q,
                   double max_dist) {
  int best = -1;
  double best_d2 = max_dist * max_dist;
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    if (!cloud[i].valid) continue;
    const double dx = cloud[i].x - q.x();
    const double dy = cloud[i].y - q.y();
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = static_cast<int>(i);
    }
  }
  return best;
}

// Kabsch on the trimmed pair set. Returns the incremental rigid transform
// that maps src onto dst.
inline Transform2 kabsch(const std::vector<Eigen::Vector2d>& src,
                         const std::vector<Eigen::Vector2d>& dst) {
  const std::size_t n = src.size();
  Eigen::Vector2d cs = Eigen::Vector2d::Zero();
  Eigen::Vector2d cd = Eigen::Vector2d::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    cs += src[i];
    cd += dst[i];
  }
  cs /= static_cast<double>(n);
  cd /= static_cast<double>(n);

  Eigen::Matrix2d h = Eigen::Matrix2d::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    h += (src[i] - cs) * (dst[i] - cd).transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix2d> svd(h, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d r = svd.matrixV() * svd.matrixU().transpose();
  // Guard against a reflection; with planar point sets this only triggers on
  // degenerate (near-collinear) geometry, e.g. staring down a long aisle.
  if (r.determinant() < 0.0) {
    Eigen::Matrix2d v = svd.matrixV();
    v.col(1) *= -1.0;
    r = v * svd.matrixU().transpose();
  }

  Transform2 out;
  out.theta = std::atan2(r(1, 0), r(0, 0));
  const Eigen::Vector2d t = cd - r * cs;
  out.x = t.x();
  out.y = t.y();
  return out;
}

}  // namespace detail

// Aligns `current` onto `previous`. `guess` is the odometry-predicted motion
// and is used as the initial estimate, which matters a lot at 5 Hz.
inline IcpResult align(const std::vector<Point2>& previous,
                       const std::vector<Point2>& current,
                       const Transform2& guess,
                       const IcpParams& params = IcpParams{}) {
  IcpResult result;
  result.transform = guess;
  Transform2 total = guess;

  for (int iter = 0; iter < params.max_iterations; ++iter) {
    std::vector<Eigen::Vector2d> src;
    std::vector<Eigen::Vector2d> dst;
    std::vector<double> err;
    src.reserve(current.size());
    dst.reserve(current.size());
    err.reserve(current.size());

    for (const Point2& p : current) {
      if (!p.valid) continue;
      const Eigen::Vector2d q = detail::apply(total, detail::toVec(p));
      const int j = detail::nearest(previous, q, params.max_correspondence_dist);
      if (j < 0) continue;
      src.push_back(detail::toVec(p));
      dst.push_back(detail::toVec(previous[j]));
      err.push_back((q - dst.back()).norm());
    }

    if (static_cast<int>(src.size()) < params.min_pairs) {
      result.pairs = static_cast<int>(src.size());
      result.iterations = iter;
      return result;  // not enough overlap; caller falls back to odometry
    }

    // Trim the worst pairs. Sorting an index vector keeps the pair arrays
    // consistent without a second allocation pass per element.
    std::vector<std::size_t> order(src.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(),
              [&err](std::size_t a, std::size_t b) { return err[a] < err[b]; });

    const std::size_t keep = std::max<std::size_t>(
        static_cast<std::size_t>(params.min_pairs),
        static_cast<std::size_t>(params.trim_fraction * static_cast<double>(order.size())));

    std::vector<Eigen::Vector2d> src_t;
    std::vector<Eigen::Vector2d> dst_t;
    src_t.reserve(keep);
    dst_t.reserve(keep);
    double sq = 0.0;
    for (std::size_t k = 0; k < keep; ++k) {
      const std::size_t i = order[k];
      src_t.push_back(src[i]);
      dst_t.push_back(dst[i]);
      sq += err[i] * err[i];
    }

    const Transform2 step = detail::kabsch(src_t, dst_t);
    total = [&] {
      Transform2 out;
      const Eigen::Matrix3d m = step.matrix() * total.matrix();
      out.theta = std::atan2(m(1, 0), m(0, 0));
      out.x = m(0, 2);
      out.y = m(1, 2);
      return out;
    }();

    result.pairs = static_cast<int>(keep);
    result.rmse = std::sqrt(sq / static_cast<double>(keep));
    result.iterations = iter + 1;

    const double dtrans = std::hypot(step.x, step.y);
    const double drot = std::abs(step.theta);
    if (dtrans < params.convergence_eps && drot < params.convergence_eps) {
      result.converged = true;
      break;
    }
  }

  result.transform = total;
  return result;
}

}  // namespace slam
