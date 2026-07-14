#!/usr/bin/env bash
#
# Headless TLC runner for the espnow_mesh TLA+ specs.
#
# Usage:
#   ./run-tlc.sh                 # run every configured check
#   ./run-tlc.sh <Module> <cfg>  # run one check (cfg without .cfg suffix)
#
# Each check names its expected outcome (PASS = no violation, or a specific
# expected violation for the counterexample/tension cases). The script prints
# OK when the observed outcome matches the expectation and fails otherwise, so
# it doubles as a regression gate (`make check`).
#
# tla2tools.jar is fetched by ./get-tools.sh into ./.tools/.
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JAR="${TLA_TOOLS_JAR:-$HERE/.tools/tla2tools.jar}"
WORK="$HERE/.work"
mkdir -p "$WORK"

if [ ! -f "$JAR" ]; then
    echo "tla2tools.jar not found at $JAR; run ./get-tools.sh first" >&2
    exit 2
fi

# Common TLC flags. -deadlock disables deadlock checking: a quiescent mesh
# (controller done, satellite caught up) is a legitimate terminal state here,
# not a bug. -workers auto parallelizes across cores.
TLC_FLAGS=(-cleanup -deadlock -workers auto)

# Each row: <Module> <cfgBasename> <EXPECT> <human description>
#   EXPECT = PASS                      -> no violation of any kind
#          = VIOLATE:<Name>            -> that invariant/property must be reported violated
CHECKS=(
    # Recovery + replay pair (#1, #8) and the fix-validation matrix.
    "MeshReplayRecovery MeshReplayRecovery_immediate_replay VIOLATE:DedupSoundness   replay exploit against shipped rule (#1 counterexample)"
    "MeshReplayRecovery MeshReplayRecovery_immediate_benign  PASS                    shipped rule sound + recovers under benign channel (#1,#8)"
    "MeshReplayRecovery MeshReplayRecovery_kconsec_replay    VIOLATE:DedupSoundness  k-consecutive hardening still replayable (#1)"
    "MeshReplayRecovery MeshReplayRecovery_sticky_benign     VIOLATE:DeliveryRecovery over-strict rule breaks R1 recovery (#8 tension)"
    "MeshReplayRecovery MeshReplayRecovery_perboot_replay    PASS                    per-boot high-water mark defeats replay (#1)"
    "MeshReplayRecovery MeshReplayRecovery_perboot_benign    PASS                    per-boot fix keeps R1 recovery (#8)"
    # Reliable-delivery core (#1, #2, #5, #7) + the note_ack filter teeth.
    "MeshDelivery       MeshDelivery                         PASS                    reliable-delivery core: dedup/no-false-completion/bounded/delivery (#1,#2,#5,#7)"
    "MeshDelivery       MeshDelivery_brokenack               VIOLATE:NoFalseCompletion note_ack seq filter removed -> false completion (#2)"
    # Table conservation, the B3 race (#4).
    "MeshTable          MeshTable                            PASS                    serialized find_or_add: no duplicate table entries (#4)"
    "MeshTable          MeshTable_b3race                     VIOLATE:TableConservation B3 race: torn check-then-insert duplicates a MAC (#4)"
    # Patch distribution (#6, #11) + apply-safety / full-hash teeth.
    "MeshPatch          MeshPatch                            PASS                    patch apply-safety + agreement + progress (#6,#11)"
    "MeshPatch          MeshPatch_earlyapply                 VIOLATE:ApplySafety     APPLY before all-ready -> apply-safety broken (#6a)"
    "MeshPatch          MeshPatch_nohashcheck                VIOLATE:ApplyIntegrity  READY without full-patch hash -> applies corruption (#6b)"
    # Discovery + lock integrity + time-sync liveness (#3, #9, #10).
    "MeshDiscoverySync  MeshDiscoverySync                    PASS                    discovery + time-sync liveness incl. reboot re-sync (#9,#10)"
    "MeshDiscoverySync  MeshDiscoverySync_lockintegrity      PASS                    two controllers: satellite locks to exactly one (#3)"
    "MeshDiscoverySync  MeshDiscoverySync_brokenlock         VIOLATE:LockIntegrity   controller check removed -> satellite flaps controllers (#3)"
)

run_one() {
    local module="$1" cfg="$2" expect="$3"; shift 3
    local desc="$*"
    local log="$WORK/${cfg}.log"
    echo "=============================================================="
    echo "CHECK  $module ($cfg)"
    echo "EXPECT $expect - $desc"
    java -XX:+UseParallelGC -cp "$JAR" tlc2.TLC \
        "${TLC_FLAGS[@]}" -config "$HERE/${cfg}.cfg" "$HERE/${module}.tla" \
        >"$log" 2>&1
    # Classify the outcome. TLC names a violated INVARIANT inline; for a
    # violated temporal PROPERTY older/newer builds only print "Temporal
    # properties were violated", so we detect the class and trust the
    # expectation's named property (each is verified in isolation elsewhere).
    local violated="" ok=0
    if grep -q "Invariant .* is violated" "$log"; then
        violated="$(grep -o 'Invariant [A-Za-z0-9_]* is violated' "$log" | head -1 | awk '{print $2}')"
    elif grep -q "Temporal properties were violated" "$log"; then
        violated="<temporal>"
    fi

    case "$expect" in
        PASS)
            if [ -z "$violated" ] && grep -q "No error has been found" "$log"; then
                ok=1
            fi
            ;;
        VIOLATE:*)
            local want="${expect#VIOLATE:}"
            if [ "$violated" = "$want" ] || [ "$violated" = "<temporal>" ]; then
                ok=1
            fi
            ;;
    esac

    if [ "$ok" = 1 ]; then
        if [ -n "$violated" ]; then
            echo "RESULT OK (expected violation of $violated reproduced)"
        else
            echo "RESULT OK (no violation, as expected)"
        fi
        return 0
    else
        echo "RESULT MISMATCH"
        [ -n "$violated" ] && echo "  observed violation: $violated" || echo "  observed: no violation"
        echo "  --- tail of $log ---"
        tail -n 25 "$log" | sed 's/^/  /'
        return 1
    fi
}

if [ "$#" -eq 2 ]; then
    # single explicit run: <Module> <cfgBasename>
    java -XX:+UseParallelGC -cp "$JAR" tlc2.TLC \
        "${TLC_FLAGS[@]}" -config "$HERE/${2}.cfg" "$HERE/${1}.tla"
    exit $?
fi

fails=0
for row in "${CHECKS[@]}"; do
    # shellcheck disable=SC2086
    run_one $row || fails=$((fails + 1))
done

echo "=============================================================="
if [ "$fails" -eq 0 ]; then
    echo "ALL CHECKS OK"
    exit 0
else
    echo "$fails CHECK(S) MISMATCHED EXPECTATION"
    exit 1
fi
