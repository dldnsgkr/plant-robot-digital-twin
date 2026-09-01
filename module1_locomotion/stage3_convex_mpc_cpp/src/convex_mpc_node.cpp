// stage3: SRBD Convex MPC 보행/균형 제어기 (MIT Cheetah 방식, Di Carlo 2018)
//
// 구조:
//   MPC 루프(25Hz): 단일강체(SRBD) 동역학을 예측구간 N=10(0.4s)으로 선형화·
//     응축(condense)하고, 지면반력(GRF) 시퀀스를 마찰원뿔·수직력 제약 QP로
//     최적화 (EiquadprogFast active-set).
//   제어 루프(200Hz): 지지다리 τ = -Jᵀ Rᵀ f (GRF→관절토크), 스윙다리는
//     IK 목표 관절 PD. 토크는 URDF 한계로 클램프.
//
// stage2의 결론(위치제어 지지에서는 CoP-LIP 모델 불일치 → 토크 제어 필요)이
// 본 구현의 출발점이다. 모드: stand(4지지 균형) / trot(대각 보행).
#include <cmath>
#include <algorithm>
#include <array>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <eiquadprog/eiquadprog-fast.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

// ---- 로봇 파라미터 (URDF 실측) ----
static constexpr double kMass = 16.09;
static const Vector3d kInertiaDiag(0.08, 0.28, 0.30);  // SRBD 근사 관성
static constexpr double kHipX = 0.1934, kHipY = 0.0465, kLHip = 0.0955;
static constexpr double kL1 = 0.213, kL2 = 0.213;
static constexpr double kMu = 0.6;          // 마찰계수
static constexpr double kFzMin = 5.0, kFzMax = 130.0;
static constexpr double kBodyH = 0.30;
static const double kTauLim[3] = {23.7, 23.7, 45.4};

// 다리 순서: 컨트롤러 yaml과 동일 (FL, FR, RL, RR) × (hip, thigh, calf)
static const char* kLegs[4] = {"FL", "FR", "RL", "RR"};
static const double kSide[4] = {+1, -1, +1, -1};
static const double kFwd[4] = {+1, +1, -1, -1};

// ---- MPC 차원 ----
static constexpr int kN = 10;               // 예측 스텝
static constexpr double kDtMpc = 0.04;      // 예측 간격 (0.4s 구간)
static constexpr int kNx = 13, kNu = 12;

// 다리 FK: 관절각 → 몸좌표 발 위치
static Vector3d legFk(int leg, const Vector3d& q) {
  const double loff = kSide[leg] * kLHip;
  const double s1 = std::sin(q[0]), c1 = std::cos(q[0]);
  const double x = -(kL1 * std::sin(q[1]) + kL2 * std::sin(q[1] + q[2]));
  const double r = kL1 * std::cos(q[1]) + kL2 * std::cos(q[1] + q[2]);
  Vector3d p;
  p.x() = kFwd[leg] * kHipX + x;
  p.y() = kSide[leg] * kHipY + loff * c1 + r * s1;
  p.z() = loff * s1 - r * c1;
  return p;
}

// 다리 Jacobian (몸좌표): ∂p/∂q
static Matrix3d legJac(int leg, const Vector3d& q) {
  const double loff = kSide[leg] * kLHip;
  const double s1 = std::sin(q[0]), c1 = std::cos(q[0]);
  const double s2 = std::sin(q[1]), c2 = std::cos(q[1]);
  const double s23 = std::sin(q[1] + q[2]), c23 = std::cos(q[1] + q[2]);
  const double x = -(kL1 * s2 + kL2 * s23);
  const double r = kL1 * c2 + kL2 * c23;
  const double drdq2 = -(kL1 * s2 + kL2 * s23);   // = x
  const double drdq3 = -kL2 * s23;
  const double dxdq2 = -(kL1 * c2 + kL2 * c23);
  const double dxdq3 = -kL2 * c23;
  Matrix3d J;
  J(0, 0) = 0;                J(0, 1) = dxdq2;          J(0, 2) = dxdq3;
  J(1, 0) = -loff * s1 + r * c1;  J(1, 1) = s1 * drdq2;  J(1, 2) = s1 * drdq3;
  J(2, 0) = loff * c1 + r * s1;   J(2, 1) = -c1 * drdq2; J(2, 2) = -c1 * drdq3;
  return J;
}

// 다리 IK (stage1 파이썬 구현의 이식) — 스윙 목표용
static Vector3d legIk(int leg, const Vector3d& pBody) {
  const double loff = kSide[leg] * kLHip;
  const double x = pBody.x() - kFwd[leg] * kHipX;
  const double y = pBody.y() - kSide[leg] * kHipY;
  const double z = pBody.z();
  const double r = std::sqrt(std::max(y * y + z * z - loff * loff, 1e-6));
  const double q1 = std::atan2(z, y) + std::atan2(r, loff);
  double d = std::sqrt(x * x + r * r);
  d = std::clamp(d, 0.11, kL1 + kL2 - 1e-4);
  const double cosKnee =
      (kL1 * kL1 + kL2 * kL2 - d * d) / (2 * kL1 * kL2);
  const double q3 = std::acos(std::clamp(cosKnee, -1.0, 1.0)) - M_PI;
  const double psi = std::acos(std::clamp(
      (kL1 * kL1 + d * d - kL2 * kL2) / (2 * kL1 * d), -1.0, 1.0));
  const double q2 = std::atan2(-x, r) + psi;
  return {q1, q2, q3};
}

static Matrix3d crossMat(const Vector3d& v) {
  Matrix3d m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

class ConvexMpcNode : public rclcpp::Node {
 public:
  ConvexMpcNode() : Node("convex_mpc") {
    declare_parameter("mode", "stand");      // stand | trot
    declare_parameter("tau_sign", -1.0);     // GRF→토크 부호 (실측 보정용)

    pubTau_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/joint_group_effort_controller/commands", 10);
    subJs_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](sensor_msgs::msg::JointState::SharedPtr m) { onJs(*m); });
    subOdom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/model/go2/odometry", 10,
        [this](nav_msgs::msg::Odometry::SharedPtr m) { onOdom(*m); });
    subCmd_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](geometry_msgs::msg::Twist::SharedPtr m) { cmd_ = *m; });

    ctrlTimer_ = create_wall_timer(std::chrono::milliseconds(5),
                                   [this] { controlStep(); });
    mpcTimer_ = create_wall_timer(std::chrono::milliseconds(40),
                                  [this] { mpcStep(); });
    dbgTimer_ = create_wall_timer(std::chrono::seconds(2), [this] { debug(); });
    RCLCPP_INFO(get_logger(), "convex_mpc 시작 (mode=%s, N=%d, dt=%.2f)",
                get_parameter("mode").as_string().c_str(), kN, kDtMpc);
  }

 private:
  // ---------- 상태 수신 ----------
  void onJs(const sensor_msgs::msg::JointState& m) {
    if (jsIdx_.empty()) {
      for (size_t i = 0; i < m.name.size(); ++i) jsIdx_[m.name[i]] = i;
      for (int l = 0; l < 4; ++l) {
        const std::string base = kLegs[l];
        idx_[l][0] = jsIdx_.at(base + "_hip_joint");
        idx_[l][1] = jsIdx_.at(base + "_thigh_joint");
        idx_[l][2] = jsIdx_.at(base + "_calf_joint");
      }
    }
    for (int l = 0; l < 4; ++l)
      for (int j = 0; j < 3; ++j) {
        q_[l][j] = m.position[idx_[l][j]];
        dq_[l][j] = m.velocity.empty() ? 0.0 : m.velocity[idx_[l][j]];
      }
    haveJs_ = true;
  }

  void onOdom(const nav_msgs::msg::Odometry& m) {
    const auto& p = m.pose.pose.position;
    const auto& qo = m.pose.pose.orientation;
    pos_ = {p.x, p.y, p.z};
    Eigen::Quaterniond quat(qo.w, qo.x, qo.y, qo.z);
    R_ = quat.toRotationMatrix();
    rpy_ = R_.eulerAngles(2, 1, 0).reverse();  // (roll,pitch,yaw)? 아래서 재계산
    // eulerAngles 다의성 회피: 직접 계산
    rpy_.x() = std::atan2(R_(2, 1), R_(2, 2));
    rpy_.y() = -std::asin(std::clamp(R_(2, 0), -1.0, 1.0));
    rpy_.z() = std::atan2(R_(1, 0), R_(0, 0));
    const auto& tv = m.twist.twist.linear;
    const auto& tw = m.twist.twist.angular;
    velW_ = R_ * Vector3d(tv.x, tv.y, tv.z);    // 몸좌표 → 월드
    omegaW_ = R_ * Vector3d(tw.x, tw.y, tw.z);
    haveOdom_ = true;
  }

  // ---------- 게이트 ----------
  bool trot() const { return get_parameter("mode").as_string() == "trot"; }
  static constexpr double kPeriod = 0.5, kDuty = 0.5;

  double legPhase(int leg, double t) const {
    const double off = (leg == 0 || leg == 3) ? 0.0 : 0.5;  // FL,RR / FR,RL
    return std::fmod(t / kPeriod + off, 1.0);
  }
  bool inContact(int leg, double t) const {
    if (!trot() || !walking_) return true;
    return legPhase(leg, t) < kDuty;
  }

  // 스윙 발 목표 (몸좌표) — stage1 규칙의 축약 이식
  Vector3d swingTarget(int leg, double t) const {
    const double p = legPhase(leg, t);
    const double s = (p - kDuty) / (1.0 - kDuty);
    const double sx = std::clamp((s - 0.15) / 0.65, 0.0, 1.0);
    const double stepX = cmd_.linear.x * (kPeriod * kDuty) / 2.0;
    const double stepY = cmd_.linear.y * (kPeriod * kDuty) / 2.0;
    Vector3d pB;
    pB.x() = kFwd[leg] * kHipX + stepX * (2 * sx - 1);
    pB.y() = kSide[leg] * (kHipY + kLHip) + stepY * (2 * sx - 1);
    pB.z() = -kBodyH + 0.07 * std::sin(M_PI * s);
    return pB;
  }

  // ---------- MPC ----------
  void mpcStep() {
    if (!haveJs_ || !haveOdom_) return;
    const double t = now().seconds();
    if (tEngage_ < 0) {
      // 시퀀스: 상태 수신 1s 후 → PD 기립 3s (effort 활성화 직후의 무너진
      // 자세에서 토크 PD로 일어섬) → MPC 개시
      tEngage_ = t + 1.0;
      return;
    }
    if (t < tEngage_) return;
    if (!engaged_) {
      engaged_ = true;
      tMpcStart_ = t + 5.0;   // TUCK 2.5s + STAND 램프 2.5s
      RCLCPP_INFO(get_logger(), "PD 기립 단계 시작 (TUCK→STAND)");
    }
    if (t < tMpcStart_) return;
    if (!mpcActive_) {
      // 기립 성공 판정: 1초간 '연속' 안정해야 MPC 진입 (순간 통과 방지)
      const bool ok = pos_.z() > 0.26 && std::abs(rpy_.x()) < 0.2 &&
                      std::abs(rpy_.y()) < 0.2;
      if (!ok) {
        stableSince_ = -1;
        tMpcStart_ = t + 5.0;   // TUCK부터 재시도
        RCLCPP_WARN(get_logger(), "기립 미완(z=%.2f rpy=%.2f,%.2f) — 재시도",
                    pos_.z(), rpy_.x(), rpy_.y());
        return;
      }
      if (stableSince_ < 0) stableSince_ = t;
      if (t - stableSince_ < 1.0) return;
      mpcActive_ = true;
      holdPos_ = pos_; holdYaw_ = rpy_.z();
      RCLCPP_INFO(get_logger(), "MPC 개시 (z=%.2f, hold %.2f, %.2f, yaw %.2f)",
                  pos_.z(), holdPos_.x(), holdPos_.y(), holdYaw_);
    }
    // 붕괴 감지 → PD 재기립으로 복귀 (무너진 자세에서는 자세가중>위치가중
    // 구조상 QP가 fz 최소화를 선택해 스스로 회복하지 못한다 — TUNING.md)
    if (mpcActive_ && (pos_.z() < 0.18 || std::abs(rpy_.x()) > 0.5 ||
                       std::abs(rpy_.y()) > 0.5)) {
      mpcActive_ = false;
      stableSince_ = -1;
      tMpcStart_ = t + 5.0;
      RCLCPP_WARN(get_logger(), "붕괴 감지(z=%.2f) → PD 재기립", pos_.z());
      return;
    }
    walking_ = trot() && (std::abs(cmd_.linear.x) + std::abs(cmd_.linear.y) +
                          std::abs(cmd_.angular.z)) > 1e-3;

    // 발 위치 (월드, COM 기준)
    std::array<Vector3d, 4> rW;
    for (int l = 0; l < 4; ++l) rW[l] = R_ * legFk(l, q_[l]);

    // 관성 (월드)
    Matrix3d Ib = kInertiaDiag.asDiagonal();
    Matrix3d IwInv = (R_ * Ib * R_.transpose()).inverse();

    // 연속계 → 이산계
    MatrixXd A = MatrixXd::Zero(kNx, kNx);
    const double cy = std::cos(rpy_.z()), sy = std::sin(rpy_.z());
    Matrix3d RzT;
    RzT << cy, sy, 0, -sy, cy, 0, 0, 0, 1;
    A.block<3, 3>(0, 6) = RzT;
    A.block<3, 3>(3, 9) = Matrix3d::Identity();
    A(11, 12) = -9.81;
    MatrixXd B = MatrixXd::Zero(kNx, kNu);
    for (int l = 0; l < 4; ++l) {
      B.block<3, 3>(6, 3 * l) = IwInv * crossMat(rW[l]);
      B.block<3, 3>(9, 3 * l) = Matrix3d::Identity() / kMass;
    }
    MatrixXd Ad = MatrixXd::Identity(kNx, kNx) + A * kDtMpc;
    MatrixXd Bd = B * kDtMpc;

    // 응축: X = Aqp x0 + Bqp U
    MatrixXd Aqp(kNx * kN, kNx), Bqp = MatrixXd::Zero(kNx * kN, kNu * kN);
    MatrixXd Apow = Ad;
    for (int k = 0; k < kN; ++k) {
      Aqp.block(kNx * k, 0, kNx, kNx) = Apow;
      MatrixXd tmp = Bd;
      for (int j = k; j >= 0; --j) {
        Bqp.block(kNx * k, kNu * j, kNx, kNu) = tmp;
        if (j > 0) tmp = Ad * tmp;
      }
      Apow = Ad * Apow;
    }

    // 상태 가중 / 기준
    VectorXd Ldiag(kNx);
    Ldiag << 60, 60, 30,  40, 40, 120,  1, 1, 2,  6, 6, 10,  0;
    VectorXd x0(kNx);
    x0 << rpy_, pos_, omegaW_, velW_, 1.0;
    VectorXd xref(kNx);
    Vector3d pref = holdPos_;
    pref.z() = kBodyH + 0.02;
    Vector3d vref(cmd_.linear.x * cy, cmd_.linear.x * sy, 0);
    if (walking_) {
      holdPos_.x() += vref.x() * kDtMpc; holdPos_.y() += vref.y() * kDtMpc;
    }
    VectorXd Xref(kNx * kN);
    for (int k = 0; k < kN; ++k) {
      xref << 0, 0, holdYaw_, pref, 0, 0, 0, (walking_ ? vref : Vector3d::Zero()), 1.0;
      Xref.segment(kNx * k, kNx) = xref;
    }

    MatrixXd Lbig = Ldiag.replicate(kN, 1).asDiagonal();
    MatrixXd H = Bqp.transpose() * Lbig * Bqp;
    H.diagonal().array() += 1e-4;             // 정칙화 + R
    VectorXd g0 = Bqp.transpose() * Lbig * (Aqp * x0 - Xref);

    // 제약: 접촉발 마찰원뿔+수직력, 스윙발 f=0 (등식)
    std::vector<int> contactAtStep[kN];
    int nEq = 0, nIn = 0;
    for (int k = 0; k < kN; ++k)
      for (int l = 0; l < 4; ++l) {
        if (inContact(l, t + k * kDtMpc)) { contactAtStep[k].push_back(l); nIn += 6; }
        else nEq += 3;
      }
    MatrixXd Aeq = MatrixXd::Zero(nEq, kNu * kN);
    VectorXd beq = VectorXd::Zero(nEq);
    MatrixXd Ain = MatrixXd::Zero(nIn, kNu * kN);
    VectorXd bin = VectorXd::Zero(nIn);
    int e = 0, c = 0;
    for (int k = 0; k < kN; ++k)
      for (int l = 0; l < 4; ++l) {
        const int col = kNu * k + 3 * l;
        if (inContact(l, t + k * kDtMpc)) {
          // fz ≥ fzmin ; fzmax - fz ≥ 0 ; μfz ± fx ≥ 0 ; μfz ± fy ≥ 0
          Ain(c, col + 2) = 1;  bin(c++) = -kFzMin;
          Ain(c, col + 2) = -1; bin(c++) = kFzMax;
          Ain(c, col) = 1;  Ain(c, col + 2) = kMu; bin(c++) = 0;
          Ain(c, col) = -1; Ain(c, col + 2) = kMu; bin(c++) = 0;
          Ain(c, col + 1) = 1;  Ain(c, col + 2) = kMu; bin(c++) = 0;
          Ain(c, col + 1) = -1; Ain(c, col + 2) = kMu; bin(c++) = 0;
        } else {
          for (int a = 0; a < 3; ++a) { Aeq(e, col + a) = 1; beq(e++) = 0; }
        }
      }

    eiquadprog::solvers::EiquadprogFast qp;
    qp.reset(kNu * kN, nEq, nIn);
    VectorXd U(kNu * kN);
    const auto status = qp.solve_quadprog(H, g0, Aeq, beq, Ain, bin, U);
    if (status != eiquadprog::solvers::EIQUADPROG_FAST_OPTIMAL) {
      solveFail_++;
      return;                                  // 이전 해 유지
    }
    for (int l = 0; l < 4; ++l) fW_[l] = U.segment(3 * l, 3);
    solveOk_++;
  }

  // ---------- 200Hz 토크 ----------
  void controlStep() {
    if (!engaged_ || !haveJs_ || !haveOdom_) return;
    const double tNow = now().seconds();
    // PD 기립 단계: TUCK(웅크림) 1.5s → STAND 램프 2.5s
    if (!mpcActive_) {
      const double ph = tNow - (tMpcStart_ - 5.0);
      const Vector3d qTuck(0.0, 1.3, -2.5), qStand(0.0, 0.8, -1.6);
      Vector3d qDes;
      if (ph < 2.5) {
        qDes = qTuck;
      } else {
        const double a = std::clamp((ph - 2.5) / 2.0, 0.0, 1.0);
        qDes = qTuck + a * (qStand - qTuck);
      }
      std::array<Vector3d, 4> tau;
      for (int l = 0; l < 4; ++l)
        for (int j = 0; j < 3; ++j)
          tau[l][j] = 38.0 * (qDes[j] - q_[l][j]) - 1.2 * dq_[l][j];
      publishTau(tau);
      return;
    }
    // 안전: 전도 시 토크 차단
    if (std::abs(rpy_.x()) > 0.7 || std::abs(rpy_.y()) > 0.7) {
      publishTau(std::array<Vector3d, 4>{Vector3d::Zero(), Vector3d::Zero(),
                                         Vector3d::Zero(), Vector3d::Zero()});
      return;
    }
    const double t = now().seconds();
    const double sgn = get_parameter("tau_sign").as_double();
    std::array<Vector3d, 4> tau;
    for (int l = 0; l < 4; ++l) {
      if (inContact(l, t)) {
        const Vector3d fB = R_.transpose() * fW_[l];
        tau[l] = sgn * legJac(l, q_[l]).transpose() * fB;
        tau[l] -= 0.5 * dq_[l];               // 관절 감쇠
      } else {
        const Vector3d qDes = legIk(l, swingTarget(l, t));
        for (int j = 0; j < 3; ++j)
          tau[l][j] = 28.0 * (qDes[j] - q_[l][j]) - 0.8 * dq_[l][j];
      }
    }
    publishTau(tau);
  }

  void publishTau(const std::array<Vector3d, 4>& tau) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(12);
    for (int l = 0; l < 4; ++l)
      for (int j = 0; j < 3; ++j)
        msg.data[3 * l + j] =
            std::clamp(tau[l][j], -kTauLim[j], kTauLim[j]);
    pubTau_->publish(msg);
  }

  void debug() {
    if (!haveOdom_) return;
    RCLCPP_INFO(get_logger(),
                "dbg z=%.2f rpy=(%.2f,%.2f,%.2f) fz=[%.0f %.0f %.0f %.0f] "
                "qp ok/fail=%d/%d",
                pos_.z(), rpy_.x(), rpy_.y(), rpy_.z(), fW_[0].z(), fW_[1].z(),
                fW_[2].z(), fW_[3].z(), solveOk_, solveFail_);
  }

  // 상태
  std::map<std::string, size_t> jsIdx_;
  size_t idx_[4][3];
  Vector3d q_[4], dq_[4];
  Vector3d pos_{0, 0, 0}, rpy_{0, 0, 0}, velW_{0, 0, 0}, omegaW_{0, 0, 0};
  Matrix3d R_ = Matrix3d::Identity();
  std::array<Vector3d, 4> fW_{Vector3d(0, 0, kMass * 9.81 / 4),
                              Vector3d(0, 0, kMass * 9.81 / 4),
                              Vector3d(0, 0, kMass * 9.81 / 4),
                              Vector3d(0, 0, kMass * 9.81 / 4)};
  geometry_msgs::msg::Twist cmd_;
  Vector3d holdPos_{0, 0, 0};
  double holdYaw_ = 0, tEngage_ = -1, tMpcStart_ = 0, stableSince_ = -1;
  bool haveJs_ = false, haveOdom_ = false, engaged_ = false,
       mpcActive_ = false, walking_ = false;
  int solveOk_ = 0, solveFail_ = 0;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pubTau_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subJs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subCmd_;
  rclcpp::TimerBase::SharedPtr ctrlTimer_, mpcTimer_, dbgTimer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConvexMpcNode>());
  rclcpp::shutdown();
  return 0;
}
