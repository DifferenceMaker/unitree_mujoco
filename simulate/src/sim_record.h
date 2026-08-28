#pragma once
// In-sim frame recorder (2026-08-21). Why not x11grab: under a Wayland
// compositor the XWayland root is not composited, so x11grab records a BLACK
// screen (cursor only — the 14:38 run); portal screencast APIs are not
// scriptable headlessly on this desktop. So grab the pixels where they are
// guaranteed to exist: mjr_readPixels at the end of Render(), piped to ffmpeg
// as rawvideo. Captures the WHOLE window incl. UI panels and all HUD overlays.
//
// Enabled when ARCHB_RECORD_FILE is set (launcher --record). ~30 fps wall
// clock (the render loop is vsync-off and free-running). ffmpeg needs a
// constant frame size, so: capture starts only once the window size has been
// STABLE for ~1.5 s (the operator maximises the window right after launch —
// the 2026-08-28 recording was a 4 s pre-maximise stub), and a later resize
// closes the current segment and opens the next one (<base>_part2.mp4, ...)
// instead of killing the recording.
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

inline std::string& cur_path_() { static std::string p; return p; }
inline int& segment_() { static int n = 0; return n; }

inline void stop() {
  if (pipe_()) {
    pclose(pipe_());
    pipe_() = nullptr;
    std::printf("[sim_record] saved: %s\n", cur_path_().c_str());
    std::fflush(stdout);
  }
}

// <base>.mp4 for the first segment, <base>_part2.mp4 ... after a resize
inline std::string segment_path(const char* base, int seg) {
  std::string b(base);
  if (seg <= 1) return b;
  const size_t dot = b.rfind('.');
  const std::string stem = (dot == std::string::npos) ? b : b.substr(0, dot);
  const std::string ext = (dot == std::string::npos) ? ".mp4" : b.substr(dot);
  return stem + "_part" + std::to_string(seg) + ext;
}

inline void grab(const mjrRect& rect, const mjrContext* con) {
  static const char* path = std::getenv("ARCHB_RECORD_FILE");
  if (!path || !*path || dead_()) return;
  static int fw = 0, fh = 0;
  static std::vector<unsigned char> buf;
  static double last = 0.0;
  static int pend_w = 0, pend_h = 0;      // size-stability debounce
  static double pend_since = 0.0;
  static bool atexit_set = false;

  const int cw = rect.width - rect.width % 2, ch = rect.height - rect.height % 2;
  if (cw < 64 || ch < 64) return;        // window not realized yet

  if (!pipe_()) {
    // wait until the window size has been stable for 1.5 s before opening a segment
    if (cw != pend_w || ch != pend_h) { pend_w = cw; pend_h = ch; pend_since = now_s(); return; }
    if (now_s() - pend_since < 1.5) return;
    fw = cw; fh = ch;
    segment_() += 1;
    cur_path_() = segment_path(path, segment_());
    char cmd[1024];
    std::snprintf(cmd, sizeof cmd,
        "ffmpeg -loglevel error -y -f rawvideo -pixel_format rgb24 "
        "-video_size %dx%d -framerate 30 -i - -vf vflip -c:v libx264 "
        "-preset veryfast -crf 23 -pix_fmt yuv420p "
        "-movflags +frag_keyframe+empty_moov '%s'",
        fw, fh, cur_path_().c_str());
    pipe_() = popen(cmd, "w");
    if (!pipe_()) { dead_() = true; return; }
    buf.resize(static_cast<size_t>(3) * fw * fh);
    if (!atexit_set) { std::atexit(stop); atexit_set = true; }
    std::printf("[sim_record] recording %dx%d -> %s\n", fw, fh, cur_path_().c_str());
    std::fflush(stdout);
  }
  if (cw != fw || ch != fh) {
    std::printf("[sim_record] window resized — closing segment %d, next one starts when the size settles\n", segment_());
    std::fflush(stdout);
    stop();                                // next grab() re-arms the debounce and opens _partN
    pend_w = cw; pend_h = ch; pend_since = now_s();
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
