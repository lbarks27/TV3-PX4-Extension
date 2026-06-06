#!/usr/bin/env bash
set -u

# Default repo path (used when no argument is given and we are not inside the checkout)
DEFAULT_REPO="/Users/liambarkley/Developer/TV3/TV3 PX4 Extension"

# Auto-detect: if no explicit path, prefer the current directory when it looks like the TV3 PX4 Extension repo
if [ -z "${1:-}" ]; then
  if [ -f "./scripts/run_sitl_gazebo_fast.sh" ] && [ -d "config/vehicles" ]; then
    REPO_ROOT="$(pwd)"
  else
    REPO_ROOT="${DEFAULT_REPO}"
  fi
else
  REPO_ROOT="$1"
fi

TV3_ROOT="$(cd "${REPO_ROOT}/.." 2>/dev/null && pwd || true)"
WORKTREE="${TV3_ROOT}/.work/px4-tv3"
MODULES_LINK="${TV3_ROOT}/.work/tv3-px4-extension"

printf 'TV3 SITL status\n'
printf 'repo: %s\n' "${REPO_ROOT}"

if [ -d "${REPO_ROOT}/.git" ]; then
  printf 'git: '
  git -C "${REPO_ROOT}" status --short --branch 2>/dev/null | tr '\n' ' '
  printf '\n'
else
  printf 'git: not a git checkout or missing repo\n'
fi

printf 'worktree: %s' "${WORKTREE}"
if [ -d "${WORKTREE}" ]; then
  printf ' exists\n'
else
  printf ' missing\n'
fi

printf 'external_modules: %s' "${MODULES_LINK}"
if [ -L "${MODULES_LINK}" ]; then
  printf ' -> %s\n' "$(readlink "${MODULES_LINK}")"
elif [ -e "${MODULES_LINK}" ]; then
  printf ' exists but is not a symlink\n'
else
  printf ' missing\n'
fi

printf 'vehicle manifests:\n'
if [ -d "${REPO_ROOT}/config/vehicles" ]; then
  find "${REPO_ROOT}/config/vehicles" -maxdepth 1 -name '*.yaml' -print | sort | sed 's/^/  /'
else
  printf '  missing config/vehicles\n'
fi

printf 'motor roots:\n'
for candidate in \
  "${REPO_ROOT}/build/motors" \
  "${REPO_ROOT}/build/barebones/runtime/fs/microsd/tv3/motors"
do
  if [ -d "${candidate}" ]; then
    printf '  exists %s\n' "${candidate}"
  else
    printf '  missing %s\n' "${candidate}"
  fi
done

printf 'processes:\n'
if pgrep -af "gz sim|gazebo|px4_sitl_default|bin/px4|gz_bridge" >/tmp/tv3-sitl-processes.$$ 2>/dev/null; then
  sed 's/^/  /' /tmp/tv3-sitl-processes.$$
else
  printf '  none\n'
fi
rm -f /tmp/tv3-sitl-processes.$$

printf 'ports:\n'
if lsof -nP -iUDP:18570 -iUDP:14580 >/tmp/tv3-sitl-ports.$$ 2>/dev/null; then
  sed 's/^/  /' /tmp/tv3-sitl-ports.$$
else
  printf '  no listeners on UDP 18570 or 14580\n'
fi
rm -f /tmp/tv3-sitl-ports.$$
