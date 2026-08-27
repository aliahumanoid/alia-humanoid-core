# Security Policy

Alia is a Phase 0 research prototype: bench hardware under active
development, no production deployments. Security reports are welcome —
especially anything that touches the safety-relevant paths:

- firmware safety behaviour (e-stop handling, motion guards, torque cuts,
  encoder-fault responses)
- the host↔controller CAN protocol and its failure modes
- the CAN firmware-update path
- the host web interface

## How to report

**Preferred:** use GitHub's private vulnerability reporting —
["Report a vulnerability"](../../security/advisories/new) on this
repository. It keeps the report private while we look at it.

**Alternative:** email `info@aliahumanoid.com` with enough detail to
reproduce.

Please do **not** open a public issue for anything exploitable before
we've had a chance to look at it.

## What to expect

Best-effort acknowledgment and follow-up — this is a research project,
not a staffed security team, and there is no bounty program. Reports
that lead to a fix are credited in the release notes unless you prefer
otherwise.

## Scope notes

The safety chain on the bench (hardware e-stop, current-limited supply,
firmware guards) is documented in the repository; findings about gaps
between the documentation and the code are in scope and appreciated.
