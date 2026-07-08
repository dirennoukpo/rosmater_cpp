#!/usr/bin/env bash
#
# build_docs.sh — generate (and optionally view) the Doxygen HTML API reference.
#
# Usage:
#   ./scripts/build_docs.sh              # just generate docs/html
#   ./scripts/build_docs.sh --open       # generate, then open the static file in a browser
#   ./scripts/build_docs.sh --serve      # generate, serve on http://localhost:8000, open browser
#   ./scripts/build_docs.sh --serve 9000 # ... on a custom port
#
# Install the tools first (see also the README "Browsing the API reference" section):
#   Debian/Ubuntu/WSL : sudo apt-get install doxygen graphviz
#   Fedora/RHEL       : sudo dnf install doxygen graphviz
#   Arch              : sudo pacman -S doxygen graphviz
#   macOS (Homebrew)  : brew install doxygen graphviz
# graphviz is optional (only enables the call/collaboration graphs).
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
HTML_DIR="$ROOT/docs/html"
INDEX="$HTML_DIR/index.html"

# ── Open a URL or file in the host browser (WSL-, Linux- and macOS-aware) ──────
open_target() {
    local target="$1"
    if   command -v wslview      >/dev/null 2>&1; then wslview "$target" >/dev/null 2>&1 || true
    elif command -v powershell.exe >/dev/null 2>&1; then powershell.exe -NoProfile -Command "Start-Process '$target'" >/dev/null 2>&1 || true
    elif command -v xdg-open     >/dev/null 2>&1; then xdg-open "$target" >/dev/null 2>&1 || true
    elif command -v open         >/dev/null 2>&1; then open "$target"     >/dev/null 2>&1 || true
    else echo "   (no browser opener found — open the address manually)"; fi
}

# ── 1. Check tools ────────────────────────────────────────────────────────────
if ! command -v doxygen >/dev/null 2>&1; then
    echo "error: doxygen is not installed." >&2
    echo "       Debian/Ubuntu/WSL: sudo apt-get install doxygen graphviz" >&2
    echo "       Fedora: sudo dnf install doxygen graphviz | Arch: sudo pacman -S doxygen graphviz | macOS: brew install doxygen graphviz" >&2
    exit 1
fi
command -v dot >/dev/null 2>&1 || \
    echo "warning: graphviz 'dot' not found — call/collaboration graphs will be skipped (install: graphviz)."

# ── 2. Generate ───────────────────────────────────────────────────────────────
echo ">> Running doxygen…"
doxygen Doxyfile
echo ">> Done. Generated: $INDEX"

# ── 3. View ───────────────────────────────────────────────────────────────────
mode="${1:-}"
case "$mode" in
  --open)
    # Open the static file directly. On WSL, translate to a Windows path.
    if command -v wslpath >/dev/null 2>&1 && grep -qi microsoft /proc/version 2>/dev/null; then
        open_target "$(wslpath -w "$INDEX")"
    else
        open_target "file://$INDEX"
    fi
    ;;
  --serve)
    port="${2:-8000}"
    if ! command -v python3 >/dev/null 2>&1; then
        echo "error: python3 is required for --serve." >&2; exit 1
    fi
    fuser -k "${port}/tcp" >/dev/null 2>&1 || true   # free the port if reused
    url="http://localhost:${port}/"
    echo ">> Serving at ${url}"
    echo "   (WSL/localhost tip: if it doesn't load, try http://$(hostname -I | awk '{print $1}'):${port}/ )"
    echo "   Press Ctrl+C to stop."
    ( sleep 1; open_target "$url" ) &                 # open the browser once the server is up
    exec python3 -m http.server "$port" --bind 0.0.0.0 --directory "$HTML_DIR"
    ;;
  "")
    echo "   View it with:  ./scripts/build_docs.sh --serve   (recommended)"
    echo "              or:  ./scripts/build_docs.sh --open"
    ;;
  *)
    echo "unknown option: $mode  (use --open or --serve [PORT])" >&2; exit 1
    ;;
esac
