#include "Tools/Kalman/extended_kalman_filter.hpp"

#include <numeric>

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nees"] = 0.0;
  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  P = F * P * F.transpose() + Q;
  x = f(x);
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  // 先验（predict 之后、量测更新之前）
  const Eigen::VectorXd x_prior = x;
  const Eigen::MatrixXd P_prior = P;

  // 创新（须用先验预测）：ν = z - h(x⁻), S = H P⁻ Hᵀ + R
  const Eigen::VectorXd innovation = z_subtract(z, h(x_prior));
  const Eigen::MatrixXd S = H * P_prior * H.transpose() + R;
  const Eigen::MatrixXd S_inv = S.inverse();
  const Eigen::MatrixXd K = P_prior * H.transpose() * S_inv;

  // Joseph form 后验协方差
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  P = (I - K * H) * P_prior * (I - K * H).transpose() + K * R * K.transpose();
  x = x_add(x_prior, K * innovation);

  // NIS：归一化创新平方（先验）；无真值时 NEES 用状态修正相对 P⁻ 作一致性参考
  const double nis = innovation.transpose() * S_inv * innovation;
  const Eigen::VectorXd dx = x - x_prior;
  const double nees = dx.transpose() * P_prior.llt().solve(dx);

  // 卡方检验阈值（95%，按量测维数 / 状态维数）
  const int dof = static_cast<int>(innovation.size());
  auto chi2_95 = [](int d) -> double {
    static const double kTable[] = {0.0, 3.841, 5.991, 7.815, 9.488, 11.070, 12.592};
    if (d >= 1 && d < static_cast<int>(sizeof(kTable) / sizeof(kTable[0]))) return kTable[d];
    return 12.592;
  };
  const double nis_threshold = chi2_95(dof);
  const double nees_threshold = chi2_95(static_cast<int>(x.rows()));

  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  if (nis > nis_threshold) {
    nis_count_++;
    data["nis_fail"] = 1.0;
  }
  if (nees > nees_threshold) {
    nees_count_++;
    data["nees_fail"] = 1.0;
  }
  total_count_++;
  last_nis = nis;

  recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);

  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

  if (innovation.size() > 0) data["residual_yaw"] = innovation[0];
  if (innovation.size() > 1) data["residual_pitch"] = innovation[1];
  if (innovation.size() > 2) data["residual_distance"] = innovation[2];
  if (innovation.size() > 3) data["residual_angle"] = innovation[3];
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = recent_rate;

  return x;
}

}  // namespace tools