#!/usr/bin/env python3
"""Re-apply the sim2sim HUD / arm-slider / vsync edits to the vendored MuJoCo.

`simulate/mujoco` is a symlink to the system MuJoCo install (e.g.
~/.mujoco/mujoco-3.3.6) and is gitignored, so the edits we make to its
simulate.cc / simulate.h are NOT tracked by this repo. This script versions
those edits and re-applies them (e.g. after a MuJoCo reinstall/upgrade).

It is idempotent: each hunk is skipped if already present.

  python3 simulate/mujoco_hud_patch/apply.py            # apply
  python3 simulate/mujoco_hud_patch/apply.py --check     # report only

What it adds (all DEBUG/sim2sim-only, no effect on physics):
  simulate.cc : include policy_hud.h + arm_gui.h; policy/payload/metrics HUD
                overlay in Render(); "Arm Cmd" slider section (MakeArmSection,
                registered in MakeUiSections) + its publish hook in UiEvent;
                "lean rad" slider (Desk6/6b lean command) in the same section.
  simulate.h  : default Vertical Sync off (vsync = 0).
The companion headers (policy_hud.h, arm_gui.h) live in simulate/src/ and ARE
tracked by this repo.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MJ = os.path.join(HERE, "..", "mujoco", "simulate")  # via the symlink
CC = os.path.join(MJ, "simulate.cc")
H = os.path.join(MJ, "simulate.h")

# (file, marker-already-applied, find, replace)
HUNKS = [
    # ---- walk teleop HUD (walk_hud.h): cmd-vs-actual velocity bars ----
    (CC, '#include "walk_hud.h"',
     '#include "policy_hud.h"\n#include "arm_gui.h"',
     '#include "policy_hud.h"\n#include "arm_gui.h"\n#include "walk_hud.h"'),

    (CC, 'walk_hud::render',
     '''    std::string hud_fsm = policy_hud::get_fsm_list();''',
     '''    // walk teleop: bottom-right cmd-vs-actual bars (auto-hides w/o teleop)
    walk_hud::render(rect, &this->platform_ui->mjr_context());
    std::string hud_fsm = policy_hud::get_fsm_list();'''),

    # ---- reward LEDGER gauges (2026-08-21): SIDE PANEL adjacent to the scene
    #      (the scene viewport shrinks; gauges render in the freed strip —
    #      Isaac-HUD style, never over the robot). 'L' cycles hidden/compact/full.
    (CC, 'policy_hud::ledger_render',
     '''    walk_hud::render(rect, &this->platform_ui->mjr_context());''',
     '''    walk_hud::render(rect, &this->platform_ui->mjr_context());
    policy_hud::ledger_render(rect, &this->platform_ui->mjr_context());'''),

    # ---- in-sim recorder (sim_record.h): x11grab is BLACK under Wayland
    #      compositors; readPixels at Render() end captures window+HUD ----
    (CC, '#include "sim_record.h"',
     '#include "policy_hud.h"\n#include "arm_gui.h"',
     '#include "policy_hud.h"\n#include "arm_gui.h"\n#include "sim_record.h"'),

    (CC, 'sim_record::grab',
     '''  // finalize
  this->platform_ui->SwapBuffers();
}''',
     '''  // in-sim recording (--record / ARCHB_RECORD_FILE): grab the full window
  sim_record::grab(this->uistate.rect[0], &this->platform_ui->mjr_context());

  // finalize
  this->platform_ui->SwapBuffers();
}'''),

    # ---- font scale floor 150 (2026-08-21): under X11/XWayland the reported
    #      DPI is 96 -> fontscale 100 -> tiny blurry HUD text after the
    #      compositor upscales the surface. Floor it at 150. ----
    (CC, 'fontscale < 150',
     '''  int fontscale = ComputeFontScale(*this->platform_ui);
  this->font = fontscale/50 - 1;''',
     '''  int fontscale = ComputeFontScale(*this->platform_ui);
  if (fontscale < 150) fontscale = 150;  // XWayland reports 96dpi -> blurry 100
  this->font = fontscale/50 - 1;'''),

    # ---- desk click-to-reach (click_target.h) ----
    (CC, '#include "click_target.h"',
     '#include "policy_hud.h"\n#include "arm_gui.h"',
     '#include "policy_hud.h"\n#include "arm_gui.h"\n#include "click_target.h"'),

    (CC, 'click_target::set_pending',
     """  // 3D press
  if (state->type==mjEVENT_PRESS && state->mouserect==3) {
    // set perturbation
    int newperturb = 0;""",
     """  // 3D press
  if (state->type==mjEVENT_PRESS && state->mouserect==3) {
    // ALT+click = desk reach target (click_target.h). platform_ui_adapter
    // SWAPS left<->right while Alt is held - invert to recover the physical
    // button: reported RIGHT = physical LEFT (-> left arm), etc.
    if (state->alt) {
      int phys = state->control ? 3                       // ctrl+alt = move anchor
               : state->button==mjBUTTON_RIGHT ? 0
               : state->button==mjBUTTON_LEFT  ? 1 : 2;
      click_target::set_pending(phys, state->x, state->y, state->rect[3]);
      return;
    }
    // set perturbation
    int newperturb = 0;"""),

    (CC, 'click_target::take_pending',
     """  if (pending_.select) {
    // determine selection mode
    int selmode;""",
     """  // ALT-click desk-target resolution (same rect math as the stock select)
  if (click_target::has_pending()) {
    click_target::Click ck = click_target::take_pending();
    if (m_ && d_ && ck.r.width > 0 && ck.r.height > 0) {
      mjtNum selpnt[3];
      int selgeom = -1, selflex = -1, selskin = -1;
      int selbody = mjv_select(m_, d_, &this->opt,
                               static_cast<mjtNum>(ck.r.width) / ck.r.height,
                               (ck.x - ck.r.left) / ck.r.width,
                               (ck.y - ck.r.bottom) / ck.r.height,
                               &this->scn, selpnt, &selgeom, &selflex, &selskin);
      if (selbody >= 0) {
        click_target::resolve(m_, d_, selpnt, selgeom, ck.button);
      }
    }
  }

  if (pending_.select) {
    // determine selection mode
    int selmode;"""),


    (CC, '#include "arm_gui.h"',
     '#include "platform_ui_adapter.h"\n#include "array_safety.h"',
     '#include "platform_ui_adapter.h"\n#include "array_safety.h"\n'
     '#include "policy_hud.h"\n#include "arm_gui.h"'),

    (CC, 'policy_hud::get_status()',
     '''  // show ui 0
  if (this->ui0_enable) {''',
     '''  // sim2sim HUD (DEBUG overlay only): top-left = policy/gains (controller),
  // bottom-left = payload (sim) + balance metrics (sidecar).
  {
    std::string hud_status = policy_hud::get_status();
    if (!hud_status.empty()) {
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, hud_status.c_str(),
                  nullptr, &this->platform_ui->mjr_context());
    }
    std::string hud_aux = policy_hud::get_aux();
    if (!hud_aux.empty()) {
      mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, hud_aux.c_str(),
                  nullptr, &this->platform_ui->mjr_context());
    }
    std::string hud_fsm = policy_hud::get_fsm_list();
    if (!hud_fsm.empty()) {
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, rect, hud_fsm.c_str(),
                  nullptr, &this->platform_ui->mjr_context());
    }
  }

  // show ui 0
  if (this->ui0_enable) {'''),

    (CC, 'void MakeArmSection',
     '''// make model-dependent UI sections
void MakeUiSections(mj::Simulate* sim, const mjModel* m, const mjData* d) {''',
     '''// Feature D: live arm-pose command sliders (publishes to rt/arm_pose_cmd).
void MakeArmSection(mj::Simulate* sim) {
  mjuiDef defArm[] = {
    {mjITEM_SECTION,   "Arm Cmd", mjPRESERVE, nullptr,            "AM"},
    {mjITEM_SLIDERNUM, "sh pitch", 2, arm_gui::sliders().data()+0, "-3.0 1.5"},
    {mjITEM_SLIDERNUM, "sh roll",  2, arm_gui::sliders().data()+1, "0 1.5"},
    {mjITEM_SLIDERNUM, "sh yaw",   2, arm_gui::sliders().data()+2, "-1.5 1.5"},
    {mjITEM_SLIDERNUM, "elbow",    2, arm_gui::sliders().data()+3, "-0.9 3.0"},
    {mjITEM_SLIDERNUM, "slew s",   2, arm_gui::sliders().data()+4, "0 5"},
    {mjITEM_END}
  };
  mjui_add(&sim->ui1, defArm);
}

// make model-dependent UI sections
void MakeUiSections(mj::Simulate* sim, const mjModel* m, const mjData* d) {'''),

    (CC, 'MakeArmSection(sim);',
     '  MakeEqualitySection(sim);\n}',
     '  MakeEqualitySection(sim);\n  MakeArmSection(sim);\n}'),

    (CC, 'arm_gui::owns(it->pdata)',
     '''    // stop if UI processed event
    if (it!=nullptr || (state->type==mjEVENT_KEY && state->key==0)) {
      return;
    }
  }

  // shortcut not handled by UI''',
     '''    // Arm Cmd sliders (Feature D): publish the commanded arm pose on any edit.
    if (it && arm_gui::owns(it->pdata)) {
      arm_gui::publish_from_sliders();
    }

    // stop if UI processed event
    if (it!=nullptr || (state->type==mjEVENT_KEY && state->key==0)) {
      return;
    }
  }

  // shortcut not handled by UI'''),

    (CC, 'arm_gui::lean_slider()',
     '    {mjITEM_SLIDERNUM, "slew s",   2, arm_gui::sliders().data()+4, "0 5"},\n    {mjITEM_END}\n  };\n  mjui_add(&sim->ui1, defArm);',
     '    {mjITEM_SLIDERNUM, "slew s",   2, arm_gui::sliders().data()+4, "0 5"},\n    {mjITEM_SLIDERNUM, "lean rad", 2, &arm_gui::lean_slider(),      "-0.10 0.35"},\n    {mjITEM_END}\n  };\n  mjui_add(&sim->ui1, defArm);'),
    (CC, 'arm_gui::is_lean_slider(it->pdata)',
     '    if (it && arm_gui::owns(it->pdata)) {\n      arm_gui::publish_from_sliders();\n    }\n',
     '    if (it && arm_gui::owns(it->pdata)) {\n      arm_gui::publish_from_sliders();\n    }\n    // Lean slider (Desk6/6b lean-command policies): publish on any edit.\n    if (it && arm_gui::is_lean_slider(it->pdata)) {\n      arm_gui::publish_lean_from_slider();\n    }\n'),
    (H, 'int vsync = 0;',
     '  int vsync = 1;',
     '  int vsync = 0;  // off by default (uncapped frame rate; toggle in the Rendering UI)'),
]

check_only = "--check" in sys.argv
applied = skipped = 0
for path, marker, find, repl in HUNKS:
    rp = os.path.realpath(path)
    with open(rp, "r") as f:
        text = f.read()
    label = f"{os.path.basename(path)} :: {marker[:40]}"
    if marker in text:
        print(f"  [skip]  {label}")
        skipped += 1
        continue
    if find not in text:
        print(f"  [WARN]  anchor not found, manual fix needed: {label}")
        continue
    if check_only:
        print(f"  [todo]  {label}")
        continue
    text = text.replace(find, repl, 1)
    with open(rp, "w") as f:
        f.write(text)
    print(f"  [apply] {label}")
    applied += 1

print(f"\n{'check' if check_only else 'done'}: {applied} applied, {skipped} already present")
