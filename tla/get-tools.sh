#!/usr/bin/env bash
#
# Fetch tla2tools.jar (TLC model checker + SANY parser) into ./.tools/.
#
# The canonical download is a GitHub release asset:
#   https://github.com/tlaplus/tlaplus/releases/latest/download/tla2tools.jar
# The same jar is also vendored inside the `tlaplus-mcp` npm package, which is
# a useful fallback in sandboxes that can reach the npm registry but not
# github.com. Set TLA_TOOLS_URL to override the source entirely.
set -eu

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$HERE/.tools/tla2tools.jar"
mkdir -p "$HERE/.tools"

if [ -f "$DEST" ]; then
    echo "tla2tools.jar already present at $DEST"
    exit 0
fi

try_github() {
    curl -fsSL -o "$DEST" \
        "https://github.com/tlaplus/tlaplus/releases/latest/download/tla2tools.jar"
}

try_npm() {
    local tgz meta
    meta="$(curl -fsSL https://registry.npmjs.org/tlaplus-mcp)" || return 1
    tgz="$(printf '%s' "$meta" | python3 -c \
        'import json,sys;d=json.load(sys.stdin);v=d["dist-tags"]["latest"];print(d["versions"][v]["dist"]["tarball"])')" \
        || return 1
    local tmp; tmp="$(mktemp -d)"
    curl -fsSL -o "$tmp/pkg.tgz" "$tgz" || { rm -rf "$tmp"; return 1; }
    tar xzf "$tmp/pkg.tgz" -C "$tmp" package/lib/tla2tools.jar || { rm -rf "$tmp"; return 1; }
    cp "$tmp/package/lib/tla2tools.jar" "$DEST"
    rm -rf "$tmp"
}

if [ -n "${TLA_TOOLS_URL:-}" ]; then
    echo "fetching tla2tools.jar from $TLA_TOOLS_URL"
    curl -fsSL -o "$DEST" "$TLA_TOOLS_URL"
elif try_github; then
    echo "fetched tla2tools.jar from GitHub releases"
elif try_npm; then
    echo "fetched tla2tools.jar from the tlaplus-mcp npm package"
else
    echo "could not fetch tla2tools.jar; set TLA_TOOLS_URL to a reachable copy" >&2
    exit 1
fi

echo "tla2tools.jar installed at $DEST"
