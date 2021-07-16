#pragma once

#include <fmt/core.h>
#include <fmt/color.h>

#include <opencv2/core.hpp>

#include "devices/camera/mv_video_capture.hpp"
#include "devices/serial/uart_serial.hpp"
#include "module/armor/basic_armor.hpp"
#include "module/buff/basic_buff.hpp"
#include "module/roi/basic_roi.hpp"

auto idntifier = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "wolfvision");
