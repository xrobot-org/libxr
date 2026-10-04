#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

usage() {
  cat <<'EOF'
Usage:
  tools/check_doxygen_warnings.sh

Description:
  Run Doxygen with the repository Doxyfile (without class graphs) and compare the number
  of warnings with the baseline in doc/doxygen_warning_baseline.txt. The check fails when
  the count exceeds the baseline. CI uses Doxygen 1.12.0, the version of the docs image.
EOF
}

case "${1:-}" in
  "")
    ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    echo "Unknown argument: ${1}" >&2
    usage >&2
    exit 2
    ;;
esac

BASELINE_FILE="doc/doxygen_warning_baseline.txt"
DOXYGEN_BIN="${DOXYGEN_BIN:-doxygen}"

if ! command -v "${DOXYGEN_BIN}" >/dev/null 2>&1; then
  echo "doxygen not found. Install Doxygen 1.12.0 or set DOXYGEN_BIN." >&2
  exit 1
fi

OUT_DIR="$(mktemp -d)"
trap 'rm -rf "${OUT_DIR}"' EXIT

{
  cat Doxyfile
  echo "OUTPUT_DIRECTORY = \"${OUT_DIR}\""
  echo "WARN_LOGFILE = \"${OUT_DIR}/warnings.log\""
  echo "HAVE_DOT = NO"
} | "${DOXYGEN_BIN}" - >"${OUT_DIR}/doxygen.log" 2>&1

count="$(grep -c 'warning:' "${OUT_DIR}/warnings.log" || true)"
baseline="$(tr -d '[:space:]' <"${BASELINE_FILE}")"

echo "Doxygen $("${DOXYGEN_BIN}" --version): ${count} warnings, baseline ${baseline}."

if ((count > baseline)); then
  echo "Doxygen reports more warnings than the baseline. Warnings:" >&2
  sed "s#${REPO_ROOT}/##" "${OUT_DIR}/warnings.log" >&2
  exit 1
fi

if ((count < baseline)); then
  echo "The count is below the baseline; lower ${BASELINE_FILE} to ${count}."
fi
