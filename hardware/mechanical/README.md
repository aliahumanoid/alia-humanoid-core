# Mechanical Release Guide

This is the direct entrypoint for the currently released mechanical assets.

## Start Here

The current public Phase 0 release is centered on the **right lower leg + right ankle** plus reusable common hardware.

Released printable parts live in:

- [ankle/right/rev_a/stl](ankle/right/rev_a/stl)
- [lower_leg/right/rev_a/stl](lower_leg/right/rev_a/stl)
- [common](common)
- [BOM.csv](BOM.csv)

## What Is Released Now

- Right ankle STL set
- Right lower-leg STL set
- Common motor mounts, pulleys, sensor mounts, and cable-management parts
- Phase 0 BOM

## What Is Not a Public Build Surface Yet

These directories exist for structure and future release flow, but they are not the main public printing path today:

- `ankle/left/`
- `lower_leg/left/`
- `foot/`
- `rev_a/cad/`
- `rev_a/docs/`
- `rev_a/manufacturing/`

Treat them as repository structure, work-in-progress, or future release surface unless a component directory explicitly says otherwise.

## Practical Path

If you want to print the current public lower-body set:

1. Read [hardware/README.md](../README.md) for scope and license.
2. Use the STL folders listed above.
3. Cross-check quantities and hardware callouts in [BOM.csv](BOM.csv).
4. Use [PUBLIC_UPDATES.md](../../PUBLIC_UPDATES.md) for release context.
