#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args: []
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include "app_framework.hpp"
#include "can.hpp"

class FdcanRx : public LibXR::Application {
public:
  FdcanRx(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app)
      : can1_(hw.template FindOrExit<LibXR::CAN>({"fdcan1"})),
        can2_(hw.template FindOrExit<LibXR::CAN>({"fdcan2"})),
        can3_(hw.template FindOrExit<LibXR::CAN>({"fdcan3"})) {
    auto rx_can1_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, FdcanRx *self, const LibXR::CAN::ClassicPack &pack) {
          can1RxCallback(in_isr, self, pack);
        },
        this);
    auto rx_can2_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, FdcanRx *self, const LibXR::CAN::ClassicPack &pack) {
          can2RxCallback(in_isr, self, pack);
        },
        this);
    auto rx_can3_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, FdcanRx *self, const LibXR::CAN::ClassicPack &pack) {
          can3RxCallback(in_isr, self, pack);
        },
        this);

    can1_->Register(rx_can1_callback, LibXR::CAN::Type::STANDARD,
                    LibXR::CAN::FilterMode::ID_RANGE, 0x01, 0x01);

    can2_->Register(rx_can2_callback, LibXR::CAN::Type::STANDARD,
                    LibXR::CAN::FilterMode::ID_RANGE, 0x02, 0x02);
    can3_->Register(rx_can3_callback, LibXR::CAN::Type::STANDARD,
                    LibXR::CAN::FilterMode::ID_RANGE, 0x03, 0x03);
    thread_.Create(this, ThreadFunc, "fdcan_tx", 2048,
                   LibXR::Thread::Priority::HIGH);
  }

  static void ThreadFunc(FdcanRx *fdcan_rx) {
    while (true) {
      fdcan_rx->Fdcan1Update();
      fdcan_rx->Fdcan2Update();
      fdcan_rx->Fdcan3Update();
    }
  }

  static void can1RxCallback(bool in_isr, FdcanRx *self,
                             const LibXR::CAN::ClassicPack &pack) {
    UNUSED(in_isr);
    while (self->recv1_.Push(pack) != ErrorCode::OK) {
      self->recv1_.Pop(); // 如果队列满了，尝试弹出一个包
    }
  }

  static void can2RxCallback(bool in_isr, FdcanRx *self,
                             const LibXR::CAN::ClassicPack &pack) {
    UNUSED(in_isr);
    while (self->recv2_.Push(pack) != ErrorCode::OK) {
      self->recv2_.Pop(); // 如果队列满了，尝试弹出一个包
    }
  }
  static void can3RxCallback(bool in_isr, FdcanRx *self,
                             const LibXR::CAN::ClassicPack &pack) {
    UNUSED(in_isr);
    while (self->recv3_.Push(pack) != ErrorCode::OK) {
      self->recv3_.Pop(); // 如果队列满了，尝试弹出一个包
    }
  }

  void Fdcan1Update() {
    this->now_ = LibXR::Timebase::GetMilliseconds();
    this->dt_ = now_ - this->last_wakeup_;
    this->last_wakeup_ = this->now_;
    while (recv1_.Pop(can1_rx_pack_) == ErrorCode::OK) {
      if (can1_rx_pack_.id == 0x01) {
        memcpy(Data_1, can1_rx_pack_.data, 8);
      }
    }
  }

  void Fdcan2Update() {
    this->now_ = LibXR::Timebase::GetMilliseconds();
    this->dt_ = now_ - this->last_wakeup_;
    this->last_wakeup_ = this->now_;
    while (recv2_.Pop(can2_rx_pack_) == ErrorCode::OK) {
      if (can2_rx_pack_.id == 0x02) {
        memcpy(Data_2, can2_rx_pack_.data, 8);
      }
    }
  }

  void Fdcan3Update() {
    this->now_ = LibXR::Timebase::GetMilliseconds();
    this->dt_ = now_ - this->last_wakeup_;
    this->last_wakeup_ = this->now_;
    while (recv3_.Pop(can3_rx_pack_) == ErrorCode::OK) {
      if (can3_rx_pack_.id == 0x03) {
        memcpy(Data_3, can3_rx_pack_.data, 8);
      }
    }
  }

  void OnMonitor() override {}

private:
  float now_;
  float dt_;
  float last_wakeup_;

  LibXR::CAN *can1_;
  LibXR::CAN *can2_;
  LibXR::CAN *can3_;

  LibXR::CAN::ClassicPack can1_rx_pack_;
  LibXR::CAN::ClassicPack can2_rx_pack_;
  LibXR::CAN::ClassicPack can3_rx_pack_;

  LibXR::LockFreeQueue<LibXR::CAN::ClassicPack> recv1_ =
      LibXR::LockFreeQueue<LibXR::CAN::ClassicPack>(1);

  LibXR::LockFreeQueue<LibXR::CAN::ClassicPack> recv2_ =
      LibXR::LockFreeQueue<LibXR::CAN::ClassicPack>(1);

  LibXR::LockFreeQueue<LibXR::CAN::ClassicPack> recv3_ =
      LibXR::LockFreeQueue<LibXR::CAN::ClassicPack>(1);

  int Data_1[8] = {0};
  int Data_2[8] = {0};
  int Data_3[8] = {0};

  LibXR::Thread thread_;
};
