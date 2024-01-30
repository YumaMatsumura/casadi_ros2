// Copyright 2024 Yuma Matsumura All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "casadi_ros2/nonlinear_programs_component.hpp"

namespace casadi_ros2
{

NonlinearPrograms::NonlinearPrograms(const rclcpp::NodeOptions & options)
: Node("nonlinear_programs_node", options)
{
  // 最適化問題
  //  z + (1 - x)^2 - y = 0 を満たすとき、
  //  x^2 + 100z^2 を最小化するx, y, zを求める。

  casadi::MX x = casadi::MX::sym("x");
  casadi::MX y = casadi::MX::sym("y");
  casadi::MX z = casadi::MX::sym("z");

  casadi::MX f = pow(x, 2) + 100 * pow(z, 2);  // 最適化関数の目的関数
  casadi::MX g = z + pow(1 - x, 2) - y;        // 制約条件

  // NLP(非線形最適化問題)の定義
  casadi::MXDict nlp;
  nlp["x"] = vertcat(x, y, z); // x, y, zを連結
  nlp["f"] = f;
  nlp["g"] = g;

  // NLP(非線形最適化問題)の設定
  casadi::Function F = nlpsol("F", "ipopt", nlp); // 非線形最適化問題を解くための関数を作成

  // 最適化問題の解の計算
  casadi::DMDict result = F(casadi::DMDict{
    {"x0", casadi::DM({2.5, 3.0, 0.75})},    // 初期値x0
    {"ubg", 0},                              // 制約条件の上限ubg
    {"lbg", 0}});                            // 制約条件の下限lbg

  casadi::DM optimal_x = result.at("x");
  casadi::DM optimal_f = result.at("f");
  casadi::DM optimal_g = result.at("g");
  RCLCPP_INFO_STREAM(this->get_logger(), "x: " << optimal_x);
  RCLCPP_INFO_STREAM(this->get_logger(), "f: " << optimal_f);
  RCLCPP_INFO_STREAM(this->get_logger(), "g: " << optimal_g);
}

NonlinearPrograms::~NonlinearPrograms()
{
}

} // namespace casadi_ros2

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(casadi_ros2::NonlinearPrograms)
