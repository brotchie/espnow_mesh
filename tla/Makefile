# Headless TLA+ model checking for the espnow_mesh protocol.
#
#   make tools   # fetch tla2tools.jar into ./.tools/
#   make check   # run every configured TLC check and assert its expected outcome
#   make clean    # remove TLC scratch output
#
# `make check` is the CI gate: it returns non-zero if any check deviates from
# its documented expectation (a PASS that regressed, or an expected
# counterexample that stopped reproducing).

.PHONY: check tools clean

tools:
	./get-tools.sh

check: tools
	./run-tlc.sh

clean:
	rm -rf .work states *_TTrace_*.tla *_TTrace_*.bin
