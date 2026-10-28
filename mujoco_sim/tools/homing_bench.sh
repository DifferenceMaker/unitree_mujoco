#!/usr/bin/env bash
# homing_bench.sh -- THE walk-policy exam over a list of policies (2026-10-28): for each milestone slug, vendor it into the homing
# worktree's MovementModule/policy/<name> (policy.onnx + deploy.yaml + overrides.json from aspired-isaac-lab/milestone_checkpoints),
# launch the LOCK-STEP homing rig with it as the walk policy (deterministic: HOMING_SEED fixed), run the seed-N homing batch
# (tools/homing_trials.sh: one bench row per policy in results/HOMING_BENCH.md + the three sheets), stop, next.
#   bash mujoco_sim/tools/homing_bench.sh [--seed 1] [--n 10] [--balance p14h_stancewide] [--wall] <slug> [<slug> ...]
#   slugs = milestone_checkpoints dir names (lm7f_kit_gforce_2026-10-26) or vendored policy names already present
# Output: results/HOMING_BENCH.md rows tagged bench_<policy>_seed<N>; logs/homing_plots/*; per-policy summary printed at the end.
set -uo pipefail
SIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPOS="${REPOS:-$HOME/Projects/robot_projects/repos}"
AIL="${AIL:-$REPOS/aspired-isaac-lab}"; MMP="${ASPIRED:-$REPOS/Aspired_Robot_Project-homing}/MovementModule/policy"
SEED=1; N=10; BALANCE="${HOMING_BALANCE:-p14h_stancewide}"; MODE=lockstep
POLICIES=()
while [[ $# -gt 0 ]]; do case "$1" in
  --seed) SEED="$2"; shift 2;; --n) N="$2"; shift 2;; --balance) BALANCE="$2"; shift 2;; --wall) MODE=wall; shift;;
  -*) echo "unknown arg: $1"; exit 1;; *) POLICIES+=("$1"); shift;; esac; done
[[ ${#POLICIES[@]} -gt 0 ]] || { echo "usage: homing_bench.sh [--seed N] [--n N] [--balance NAME] [--wall] <slug> ..."; exit 1; }
vendor() {  # vendor <slug> -> prints the vendored policy name
  local slug="$1" name src
  name="$(echo "$slug" | sed -E 's/_20[0-9]{2}-[0-9]{2}-[0-9]{2}$//')"
  if [[ -f "$MMP/$name/policy.onnx" ]]; then echo "$name"; return 0; fi
  src="$AIL/milestone_checkpoints/$slug"
  [[ -f "$src/exported/policy.onnx" && -f "$src/params/deploy.yaml" ]] || { echo "!! $slug: no exported/policy.onnx or params/deploy.yaml" >&2; return 1; }
  mkdir -p "$MMP/$name" && cp "$src/exported/policy.onnx" "$MMP/$name/policy.onnx" && cp "$src/params/deploy.yaml" "$MMP/$name/deploy.yaml" \
    && cp "$src/overrides.json" "$MMP/$name/overrides.json" 2>/dev/null; cp "$src/MILESTONE.md" "$MMP/$name/" 2>/dev/null
  echo ">>> vendored $slug -> $MMP/$name (onnx md5 $(md5sum "$MMP/$name/policy.onnx" | cut -c1-12))" >&2
  echo "$name"
}
SUMMARY=()
for slug in "${POLICIES[@]}"; do
  name="$(vendor "$slug")" || { SUMMARY+=("$slug: NOT VENDORED"); continue; }
  bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1; pkill -x unitree_mujoco 2>/dev/null; sleep 3
  tag="bench_${name}_seed${SEED}"
  echo; echo "##### $(date +%T) $name ($MODE, balance $BALANCE, seed $SEED, n $N)"
  if [[ "$MODE" == lockstep ]]; then (cd "$SIM" && HOMING_WALK="$name" HOMING_BALANCE="$BALANCE" LOCOSW_LOCKSTEP=1 HOMING_SEED="$SEED" bash run_mujoco_homing.sh > "$SIM/logs/bench_rig_${tag}.log" 2>&1 &)
  else (cd "$SIM" && HOMING_WALK="$name" HOMING_BALANCE="$BALANCE" HOMING_SEED="$SEED" bash run_mujoco_homing.sh > "$SIM/logs/bench_rig_${tag}.log" 2>&1 &); fi
  ok=0; for i in $(seq 1 240); do grep -q "BALANCE STABLE" "$SIM/logs/stack_locosw_latest.log" 2>/dev/null && [[ $i -gt 15 ]] && { ok=1; break; }; grep -q "Traceback\|refused\|sim died" "$SIM/logs/stack_locosw_latest.log" 2>/dev/null && [[ $i -gt 30 ]] && break; sleep 1; done
  if [[ $ok -ne 1 ]]; then echo "!! $name: the rig did not reach BALANCE STABLE (fell at engage?) -- see $SIM/logs/bench_rig_${tag}.log"; grep -E 'Traceback|refused|ENGAGED|sim died' "$SIM/logs/stack_locosw_latest.log" | tail -3; SUMMARY+=("$name: NO STABLE STAND at engage"); continue; fi
  sleep 5
  (cd "$SIM" && bash tools/homing_trials.sh --n "$N" --seed "$SEED" --tag "$tag" > "$SIM/logs/bench_batch_${tag}.log" 2>&1)
  line="$(grep -E '^settled over ARRIVED' "$SIM/logs/bench_batch_${tag}.log" | tail -1)"; trk="$(grep -E '^trajectory' "$SIM/logs/bench_batch_${tag}.log" | tail -1)"
  echo "$line"; echo "$trk"
  SUMMARY+=("$name | $(grep -E '^trials' "$SIM/logs/bench_batch_${tag}.log" | tail -1 | cut -c1-30) | ${line#settled over ARRIVED: } | ${trk#trajectory: }")
done
bash "$SIM/run_mujoco_homing.sh" stop >/dev/null 2>&1
echo; echo "##### $(date +%T) BENCH DONE -- results/HOMING_BENCH.md has one row per policy (tag bench_<name>_seed$SEED)"; printf '%s\n' "${SUMMARY[@]}"
