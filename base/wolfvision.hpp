/**
 * @file wolfvision.hpp
 * @author XX (2796393320@qq.com)
 *         WCJ (1767851382@qq.com)
 *         SMS (2436210442@qq.com)
 *         SHL (2694359979@qq.com)
 * @brief 主函数
 * @date 2021-08-28
 *
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 *
 */
#pragma once

#include <fmt/color.h>
#include <fmt/core.h>

#include <opencv2/core.hpp>

#include "devices/camera/mv_video_capture.hpp"
#include "devices/serial/uart_serial.hpp"
#include "module/armor/basic_armor.hpp"
#include "module/buff/basic_buff.hpp"
#include "module/ml/onnx_inferring.hpp"
#include "module/record/record.hpp"
#include "module/roi/basic_roi.hpp"
#include "utils/reset_mv_camera.hpp"
#include <string>

auto idntifier = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "wolfvision");
