# Phase 3 Directed Tests

These programs target the R10K-style OoO structures added in Phase 3:
rename-map behavior, wakeup chains, branch checkpoint recovery, nested branch
masks, and conservative load/store ordering.

The initial `OOO=1` build keeps the verified Phase 2 execution path as the
architectural compatibility path while the Phase 3 structures compile and are
validated independently. These tests should continue to pass as the standalone
structures are wired into the active OoO datapath.
