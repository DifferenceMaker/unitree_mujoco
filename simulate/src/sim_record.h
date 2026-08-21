#pragma once
// In-sim frame recorder (2026-08-21). Why not x11grab: under a Wayland
// compositor the XWayland root is not composited, so x11grab records a BLACK
// screen (cursor only — the 14:38 run); portal screencast APIs are not
// scriptable headlessly on this desktop. So grab the pixels where they are
// guaranteed to exist: mjr_readPixels at the end of Render(), piped to ffmpeg
// as rawvideo. Captures the WHOLE window incl. UI panels and all HUD overlays.
//
// Enabled when ARCHB_RECORD_FILE is set (launcher --record). ~30 fps wall
// clock (the render loop is vsync-off and free-running). Window resize after
// start stops the capture (ffmpeg needs a constant size) with a notice.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

namespace sim_record {

inline FILE*& pipe_() { static FILE* p = nullptr; return p; }
inline bool& dead_() { static bool d = false; return d; }
inline double now_s() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline void stop() {
  if (pipe_()) {
    pclose(pipe_());
    pipe_() = nullptr;
    const char* path = std::getenv("ARCHB_RECORD_FILE");
    std::printf("[sim_record] saved: %s\n", path ? path : "?");
    std::fflush(stdout);
  }
}

inline void grab(const mjrRect& rect, const mjrContext* con) {
  static const char* path = std::getenv("ARCHB_RECORD_FILE");
  if (!path || !*path || dead_()) return;
  static int fw = 0, fh = 0;
  static std::vector<unsigned char> buf;
  static double last = 0.0;

  if (!pipe_()) {
    fw = rect.width - rect.width % 2;
    fh = rect.height - rect.height % 2;
    if (fw < 64 || fh < 64) return;   // window not realized yet
    char cmd[1024];
    std::snprintf(cmd, sizeof cmd,
        "ffmpeg -loglevel error -y -f rawvideo -pixel_format rgb24 "
        "-video_size %dx%d -framerate 30 -i - -vf vflip -c:v libx264 "
        "-preset veryfast -crf 23 -pix_fmt yuv420p "
        "-movflags +frag_keyframe+empty_moov '%s'",
        fw, fh, path);
    pipe_() = popen(cmd, "w");
    if (!pipe_()) { dead_() = true; return; }
    buf.resize(static_cast<size_t>(3) * fw * fh);
    std::atexit(stop);
    std::printf("[sim_record] recording %dx%d -> %s\n", fw, fh, path);
    std::fflush(stdout);
  }
  if (rect.width - rect.width % 2 != fw || rect.height - rect.height % 2 != fh) {
    std::printf("[sim_record] window resized — recording stopped\n");
    std::fflush(stdout);
    stop();
    dead_() = true;
    return;
  }
  const double t = now_s();
  if (t - last < 1.0 / 30.0) return;   // ~30 fps wall clock
  last = t;
  mjrRect r{rect.left, rect.bottom, fw, fh};
  mjr_readPixels(buf.data(), nullptr, r, con);
  if (std::fwrite(buf.data(), 1, buf.size(), pipe_()) != buf.size()) {
    std::printf("[sim_record] ffmpeg pipe broke — recording stopped\n");
    std::fflush(stdout);
    stop();
    dead_() = true;
  }
}

}  // namespace sim_record
