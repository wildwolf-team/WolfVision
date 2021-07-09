#include "wolfvision.hpp"

namespace wolfvision {
Connector::Connector() {}

Connector::~Connector() {}

void Connector::run() {
  while (true) {
    if (mv_capture_.isindustryimgInput()) {
      src_img_ = mv_capture_.image();
    }
    if (!src_img_.empty()) {
      serial_.updateReceiveInformation();
      switch (serial_.returnReceiveMode()) {
        case uart::SUP_SHOOT:
          serial_.rmSerialWrite(
              armor_.run_Armor(src_img_, serial_.returnReceive()));
          break;
        case uart::ENERGY_AGENCY:
          break;

        case uart::SENTRY_MODE:
          serial_.rmSerialWrite(
              armor_.run_Armor(src_img_, serial_.returnReceive()));

          break;
        case uart::BASE_MODE:
          serial_.rmSerialWrite(
              armor_.run_Armor(src_img_, serial_.returnReceive()));

          break;
        default:
          serial_.rmSerialWrite(
              armor_.run_Armor(src_img_, serial_.returnReceive()));

          break;
      }
      mv_capture_.cameraReleasebuff();
      armor_.free_Memory();
      // usleep(1);
      if (cv::waitKey(1) == 'q') {
        return;
      }
    }
  }
}
}  // namespace wolfvision

int main() { return 0; }
