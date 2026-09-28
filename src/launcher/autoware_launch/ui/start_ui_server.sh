#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UI_SCRIPT="${SCRIPT_DIR}/generate_autoware_map.py"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
WORKSPACE_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
WORKSPACE_INSTALL_PREFIX="${WORKSPACE_ROOT}/install"
HOST="${1:-127.0.0.1}"
PORT="${2:-8090}"
OPEN_INDEX="${3:-false}"
SERVER_PID=""

healthcheck_url() {
  echo "http://$(browser_host):${PORT}/health"
}

normalize_bool() {
  local value="${1,,}"
  case "${value}" in
    1|true|yes|on)
      echo "true"
      ;;
    0|false|no|off)
      echo "false"
      ;;
    *)
      echo "false"
      ;;
  esac
}

browser_host() {
  case "${HOST}" in
    0.0.0.0|::|[::])
      echo "127.0.0.1"
      ;;
    *)
      echo "${HOST}"
      ;;
  esac
}

open_index_page() {
  if [[ "$(normalize_bool "${OPEN_INDEX}")" != "true" ]]; then
    return 0
  fi

  if [[ -z "${DISPLAY:-}" ]]; then
    echo "DISPLAY is not set. Skipping automatic browser open."
    return 0
  fi

  if ! command -v xdg-open >/dev/null 2>&1; then
    echo "xdg-open is not available. Skipping automatic browser open."
    return 0
  fi

  local url="http://$(browser_host):${PORT}/index.html"
  xdg-open "${url}" >/dev/null 2>&1 &
  echo "Opened UI page: ${url}"
}

wait_for_server() {
  local url
  url="$(healthcheck_url)"
  local _=0
  for _ in $(seq 1 40); do
    if curl -fsS "${url}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.25
  done
  echo "UI server did not become ready in time: ${url}"
  return 1
}

server_is_healthy() {
  curl -fsS --max-time 1 "$(healthcheck_url)" >/dev/null 2>&1
}

listening_pids_on_port() {
  ss -ltnp "( sport = :${PORT} )" 2>/dev/null \
    | grep -o 'pid=[0-9]\+' \
    | cut -d= -f2 \
    | sort -u
}

pid_state() {
  local pid="${1}"
  ps -o stat= -p "${pid}" 2>/dev/null | tr -d '[:space:]'
}

is_ui_server_pid() {
  local pid="${1}"
  [[ -r "/proc/${pid}/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/${pid}/cmdline" | grep -F -- "${UI_SCRIPT}" >/dev/null 2>&1
}

stop_stale_ui_server_on_port() {
  local pid=""
  local state=""
  local stopped_pids=()
  local any_alive=false
  local _=0

  while read -r pid; do
    [[ -n "${pid}" ]] || continue
    if ! is_ui_server_pid "${pid}"; then
      continue
    fi
    state="$(pid_state "${pid}")"
    if [[ "${state}" == *T* || "${state}" == Z* ]]; then
      stopped_pids+=("${pid}")
    fi
  done < <(listening_pids_on_port)

  if [[ ${#stopped_pids[@]} -eq 0 ]]; then
    return 0
  fi

  echo "Stopping stale UI server process on port ${PORT}: ${stopped_pids[*]}"
  for pid in "${stopped_pids[@]}"; do
    kill -CONT "${pid}" >/dev/null 2>&1 || true
    kill "${pid}" >/dev/null 2>&1 || true
  done

  for _ in $(seq 1 20); do
    any_alive=false
    for pid in "${stopped_pids[@]}"; do
      if kill -0 "${pid}" >/dev/null 2>&1; then
        any_alive=true
        break
      fi
    done
    if [[ "${any_alive}" != "true" ]]; then
      return 0
    fi
    sleep 0.25
  done

  echo "Force killing stale UI server process on port ${PORT}: ${stopped_pids[*]}"
  for pid in "${stopped_pids[@]}"; do
    kill -KILL "${pid}" >/dev/null 2>&1 || true
  done
}

cleanup() {
  local exit_code=$?
  if [[ -n "${SERVER_PID}" ]] && kill -0 "${SERVER_PID}" >/dev/null 2>&1; then
    kill "${SERVER_PID}" >/dev/null 2>&1 || true
    wait "${SERVER_PID}" >/dev/null 2>&1 || true
  fi
  exit "${exit_code}"
}

if server_is_healthy; then
  echo "UI server already running from ${UI_SCRIPT}. Reusing the existing process."
  open_index_page
  exit 0
fi

stop_stale_ui_server_on_port

if server_is_healthy; then
  echo "UI server already running from ${UI_SCRIPT}. Reusing the existing process."
  open_index_page
  exit 0
fi

if ss -ltn "( sport = :${PORT} )" | grep -q LISTEN; then
  echo "Port ${PORT} is already in use, and $(healthcheck_url) is not responding."
  echo "Please stop the process that owns the port, then rerun this script."
  exit 1
fi

echo "Starting UI server: http://${HOST}:${PORT}/index.html"
trap cleanup EXIT INT TERM

workspace_already_sourced=false
case ":${AMENT_PREFIX_PATH:-}:" in
  *":${WORKSPACE_INSTALL_PREFIX}:"*)
    workspace_already_sourced=true
    ;;
esac

if [[ "${workspace_already_sourced}" != "true" && -f "${WORKSPACE_SETUP}" ]]; then
  # Colcon/Ament setup scripts may read optional trace vars that are unset.
  # Temporarily disable nounset so sourcing works even under `set -u`.
  set +u
  # shellcheck disable=SC1090
  source "${WORKSPACE_SETUP}"
  set -u
fi

python3 "${UI_SCRIPT}" --host "${HOST}" --port "${PORT}" &
SERVER_PID=$!

if wait_for_server; then
  open_index_page
fi

wait "${SERVER_PID}"
