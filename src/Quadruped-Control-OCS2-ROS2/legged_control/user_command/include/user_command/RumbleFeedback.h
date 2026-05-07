#pragma once

#include <fcntl.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>
#include <string>

class RumbleFeedback {
 public:
  explicit RumbleFeedback(const std::string& eventDevice) {
    fd_ = ::open(eventDevice.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
      return;
    }

    createEffect(stopEffect_, 0xffff, 0x0000, 350);
    createEffect(backEffect_, 0x0000, 0x8000, 180);
  }

  ~RumbleFeedback() {
    if (fd_ < 0) {
      return;
    }
    if (stopEffect_.id >= 0) {
      ::ioctl(fd_, EVIOCRMFF, stopEffect_.id);
    }
    if (backEffect_.id >= 0) {
      ::ioctl(fd_, EVIOCRMFF, backEffect_.id);
    }
    ::close(fd_);
  }

  bool isReady() const {
    return fd_ >= 0;
  }

  void playStop() const {
    playEffect(stopEffect_);
  }

  void playBack() const {
    playEffect(backEffect_);
  }

 private:
  void createEffect(ff_effect& effect, unsigned short strong, unsigned short weak, unsigned short durationMs) {
    if (fd_ < 0) {
      return;
    }

    std::memset(&effect, 0, sizeof(effect));
    effect.type = FF_RUMBLE;
    effect.id = -1;
    effect.u.rumble.strong_magnitude = strong;
    effect.u.rumble.weak_magnitude = weak;
    effect.replay.length = durationMs;
    effect.replay.delay = 0;

    if (::ioctl(fd_, EVIOCSFF, &effect) < 0) {
      effect.id = -1;
    }
  }

  void playEffect(const ff_effect& effect) const {
    if (fd_ < 0 || effect.id < 0) {
      return;
    }

    input_event playEvent{};
    playEvent.type = EV_FF;
    playEvent.code = effect.id;
    playEvent.value = 1;
    (void)::write(fd_, &playEvent, sizeof(playEvent));
  }

  int fd_ = -1;
  ff_effect stopEffect_{};
  ff_effect backEffect_{};
};
