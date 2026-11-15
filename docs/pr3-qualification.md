# CLK-02/03/04 production software qualification

The production gate, not model truth, controls media in test_qualification.
KEEP20us: coarse >100us, fine >20us; raw |offset| <1us for eight samples
spanning >=1.75s. No mute/e2e thresholds or timeouts changed.

## Frequency contract and regression

Each update spends at most rate * min(actual elapsed, 250ms), rate 5ppm/s
before lock and 1ppm/s while locked. Every rolling 1s command excursion is
<=5ppm, and <=1ppm for wholly locked windows. The complete 1ms event history
is checked at all command/lock changes and left-edge expiries; extrema cannot
change between these boundaries. Initial zero command and transition windows
are included. The former 125ms inspection missed a real 6.245ppm excursion
at case 3 t=10.602s (also reproduced in other acquisition cases).
The servo now retains previous staircase levels until change+1s, clipping
against their rolling envelopes as well as the per-update budget. Eight slots
cover production cadence; a full history conservatively holds output. History
survives reacquisition. PTP also applies the acquisition landing frequency on
the fine-step sample instead of discarding it and combining two budgets later.

Adjacent 250ms moving-average deltas <=1.25ppm are a different, stronger policy.
A retained counterexample after the rolling fix is case 7 t=9.999s:
command-average delta -1252.039996236ppb, quantized delta -1251.464709640ppb.
All approved per-update and complete rolling checks pass. The diagnostic logs
four actual command timestamps/levels, exposing staircase/window alignment.
Thus the counterexample is independent of the earlier rolling defect.

MEDIA_INC is integrated against the actual commanded frequency each tick.
For nominal increment N=335544320*48000*256/1e9, sigma-delta residual over any
window of n writes is <1 count, so its mean frequency error is <1e9/(n*N)ppb.
The 250ms and every rolling 1s mean explicitly check this bound; long-double
rolling sums avoid accumulated floating-point cancellation. The one-second
quantized mean differs from command mean by <0.243ppb; comparison of two such
means adds <0.486ppb to the applicable command excursion bound. Instantaneous
integer writes are not claimed to satisfy an analog pitch-slew limit.

## Evidence

pr3-q6.log is failure-first evidence for the rolling defect. pr3-q9.log passes
12,650,437 assertions. Original nine seeded Gaussian 200ns/1% late-outlier
scenarios run production portd/PTP/dclock/model, retain durations and events,
and play actual pattern packets. The rolling correction changes worst initial
readiness from 30.853s to 33.103s; the original <60s assertion is unchanged.
The model noise sequence is not the servo-only harness's measurement sequence.

Additional cases delay Sync delivery by 600ms for two seconds, drop Sync for
two seconds, or combine irregular missing Sync with 0/200/400ns delivery jitter.
Each qualifies through the real gate, plays packets, mutes and resumes; wholly
muted ticks have zero RX/TX leakage. Distinct UUID source replacement restarts
the leader with UUID ending 33 instead of 11; the follower changes parent,
mutes and resumes playback. This is source replacement through production
parent selection, not a three-node simultaneous BMC contest. The original
case 5 remains a modeled leader phase/frequency glide, not a UUID election.
Saturation tests reach both total rails, exercise integral freezing versus
reversal unwinding, and preserve the output through reacquisition. Exact
threshold/eight-sample/span tests are servo API tests, not full PTP boundary
injection. Full-path missing/delayed qualification covers readiness expiry.

## Limits

A persistent >200us fault permits 500ms RX-out from first observation before
mute; at 4Hz this may be 750ms after the event. Actual modeled packet recovery
is software evidence, not Dante hardware qualification or a 30-minute desk soak.
Real old-kernel AF_PACKET device runtime and congestion EAGAIN remain unvalidated.
Final same-head host, unchanged namespace e2e and AMU results are recorded in
the PR; AMU QEMU/static checks do not establish physical old-kernel runtime.
