# CLK-02 / CLK-03 / CLK-04 PR3 implementation attempt — blocked

2026-11-15. No PR, merge, deployment, desk access, or subagents. KEEP20us retained.

Latest GitHub inspected before editing: main e0c04602cad1590c02d87ccbbd34fafcdc8a7ca0; dependency #36 OPEN/UNMERGED at a9f0bd53cc124f74698573f783e5c317ee7ff3ef. New VM branch feat/clock-d4-production in /root/oi-acq starts at that exact dependency. The prior branch and /root/oi-clockdesign dirty scratch were preserved. Read clock-design-note.md, sim-run7-D4-glide.md, oi-clockdesign-scratch-v2.diff on VM and the host report in OpenProSeries/.scratch/openpro-v1/clock/. No legacy scratch clock.c was transplanted.

## Changes retained for continuation

Production servo gains .25/.02, phase ±20ppm, integral ±130ppm, total ±150ppm; held output during observed two-second / six-sample LS, subtract integrated commanded output, LS through glide; entire output capped at 5/1ppm/s using min(elapsed,250ms); anti-windup; muted >20us fine step after glide; eight consecutive <1us samples with minimum nominal span. PTP adds a synchronous mute callback into the existing #36 gate before stepping/reacquiring, >200us / 500ms elapsed persistence, current-frequency retention, stale readiness prevention, and rejection of pre-step Delay_Resp stamps. Hardware/software ready thresholds and --ready-us remain unchanged.

## Evidence and blockers

All compilation/tests ran on VM under heavy, maximum make -j4. Unrelated jobs left alone.

- pr3-red.log: original production servo fails the first-estimate 5ppm/s regression.
- pr3-sims.log: nine production servo scenarios at nominal and deterministic jitter/missing cadence pass rolling command excursions <=5ppm and wholly locked <=1ppm, including initial zero and transitions. 100ppm first simulated readiness 25.5s nominal / 31.825s jitter. This is servo-only synthetic measurement testing, NOT full production PTP or the original Gaussian/outlier fpga_model run7 harness.
- pr3-extended.log: MEDIA_INC register writes sampled over 10,000 1ms ticks. Target 4123117.700848073, actual mean 4123117.700800000; ceil writes 7008/10000. Registers intentionally alternate adjacent integers. This proves static mean dither accuracy, NOT dynamic quantized average-frequency glide or physical write timing.
- pr3-persistence.log: real model Ethernet media packet mute/resume after 500us leader phase fault; mute 0.750s after event (first observation + 0.5s persistence), recovery 8.000s after event; no RX/TX media during muted recovery. Existing traffic and forced-reacquire tests pass. The approved persistence explicitly leaves an RX-out interval before mute: do not claim zero outage or hide that half second. Model counters are actual packet playback, unlike run7's sampled offset proxy. This does not establish physical receiver recovery.
- BLOCKER: test_clock.c:227 GO LEADER/source-return regression records steps 2 -> 6, exceeding one acquisition's coarse+fine allowance (<=2 new steps). Reproduces on host and AMU. pr3-clock-trace.log locates successive step measurements -260054ns, +315356ns, +275328ns. Clearing outstanding pre-step Delay_Resp validity alone did not resolve it. Next action: trace acquisition generations, Sync/Follow_Up timestamps and Delay_Req/Resp across source return and each step; reject cross-generation observations rather than permit more steps.
- BLOCKER: unchanged namespace e2e pr3-fourth-e2e.log fails lock after 150 Syncs: SLAVE locked=0, offset -4463ns. Real mute gating consequently blocks media. Next action: capture the actual measurement cadence/noise and servo state sequence in this exact real-time model path; establish the strict eight-sample <1us rule and unchanged e2e together. Do not loosen lock or ready thresholds, bypass gating, or silently increase timeout.

Final reruns: pr3-current-check.log, pr3-current-amu.log, pr3-final-e2e.log. Earlier pr3-final-check/amu passed BEFORE the persistence correction; those are not final passing evidence. No claim of a completed/validated PR.

## Still required before opening the requested PR

Resolve both blockers with failure-first regressions. Rerun the original nine noise/outlier scenarios through the production PTP+model path (the new servo-only scenarios are insufficient). Add exact boundary tests for >100us coarse, >20us fine, eight-sample/jitter readiness and missing-Sync qualification; validate dynamic quantized MEDIA_INC average against command glide. Establish packet outage/recovery under delayed/missing/jittered Sync, not only a regular phase fault. Review anti-windup and transition sample budgets. Run full checks, unchanged e2e and AMU on the final same head. Review the diff, then commit and open a concise stacked PR against #36 and link both layers. Do not merge/deploy.

No hardware timing, real Dante receiver recovery, or independent desk-slip proof. Shared-NCO zero slips are a topology invariant, not hardware evidence. The static register dither test does not prove instantaneous edge smoothness.
