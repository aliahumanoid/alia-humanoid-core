# Contributing to Alia

Thanks for your interest in contributing! 

## Current Contribution Scope (March 2026)

Alia is publicly documented, but the contribution surface is intentionally narrow during the current Phase 0 validation cycle.

**Accepted now:**
- Software and firmware bug fixes
- Documentation improvements and clarifications
- Reproducible bug reports
- Hardware replication feedback, measurements, and validation notes
- Small tooling or test improvements that support the current stack

**Feedback first, not PRs yet:**
- Hardware geometry changes
- STL modifications
- New CAD assets intended as canonical project hardware

Why: the public hardware release is still STL-only under CC BY-NC-ND, so hardware design feedback is valuable, but hardware geometry changes are not part of the active PR surface yet. See `hardware/LICENSE.md` for the current release boundary.

## Who This Repo Is Useful For Right Now

Alia is exploring humanoid mechanics with **radical transparency**: we document what works and what does not, inside real human constraints.

This repository is most useful today if you want to:
- Follow a real lower-body humanoid validation effort as it evolves
- Report bugs, unclear documentation, or missing technical context
- Share replication findings, measurements, or hardware validation notes
- Track the project early before the collaboration surface expands

Phase 0 is still a high-barrier prototype stage. That means the most valuable external input right now is usually precise feedback, reproducible issues, and evidence from partial replication rather than large integrated PRs.

---

## Ground Rules

- Use English for code comments, documentation, and issues
- Hardware design files are currently STL-only under CC BY-NC-ND; feedback is welcome, but hardware geometry changes are not accepted as PRs yet
- Keep commits small and logically scoped
- Be respectful and constructive in discussions

---

## DCO Sign-off (Required)

All commits must include a `Signed-off-by` line:

```bash
git commit -s -m "feat: add ankle calibration script"
```

This asserts you have the right to contribute under the project license ([Developer Certificate of Origin](https://developercertificate.org/)).

**Why DCO?** It ensures clean licensing and protects both you and the project. It's a simple line added automatically with `-s` flag.

## Commit Attribution Policy

Commits in this repository must be attributed only to human maintainers or approved project identities.

- Do not add `Co-authored-by` trailers for AI tools, coding assistants, editors, or agent products
- This includes systems such as Cursor, Claude, ChatGPT, Codex, or similar tools
- Tool usage can be acknowledged in pull request descriptions, issues, or documentation when useful, but not in commit authorship metadata
- Before pushing, check commit messages for unintended `Co-authored-by` trailers and remove them if present

---

## Branches

- **`main`** (protected) — Stable code, fast-forward or squash via PR only
- **`feature/<short-topic>`** — For proposed changes

Example branch names:
- `feature/pid-tuning-script`
- `feature/update-ankle-docs`
- `fix/serial-timeout-bug`

---

## Opening a Pull Request

1. **Check whether the change is in scope**
   - Direct PRs are welcome for small documentation fixes and narrow software/firmware fixes
   - Open an issue first for non-trivial changes, new features, refactors, or anything affecting public hardware direction
   - For hardware geometry ideas, open an issue with evidence and rationale instead of a PR

2. **Create an issue first** (if non-trivial change)
   - Describe the problem or improvement
   - Include reproduction steps, data, logs, measurements, or photos when relevant
   - Wait for maintainer feedback before major work

3. **Fork and branch**
   - Fork the repo
   - Create feature branch: `git checkout -b feature/your-topic`

4. **Make changes**
   - Follow style guidelines (see below)
   - Include tests if applicable
   - Update docs if behavior changes

5. **Sign your commits**
   - Always use `git commit -s`
   - Check: `git log -1` should show `Signed-off-by: Your Name <your@email.com>`

6. **Open PR**
   - Reference issue: "Closes #123" or "Related to #456"
   - Describe what changed and why
   - Include any manual test evidence if hardware was involved
   - Request review

---

## Style Guidelines

### Directory Naming
- Use `lowercase-kebab` for directories
- Examples: `ankle-controller`, `pid-tuning`, `hardware-tests`

### Commit Messages
Use **conventional commit** prefixes:

- `feat:` — New feature
- `fix:` — Bug fix
- `docs:` — Documentation only
- `refactor:` — Code restructuring (no behavior change)
- `test:` — Adding or updating tests
- `chore:` — Maintenance (dependencies, build, etc.)

Examples:
```bash
git commit -s -m "feat: add automatic tendon calibration routine"
git commit -s -m "fix: resolve serial timeout in measurement mode"
git commit -s -m "docs: clarify PID tuning procedure in README"
```

### Code Style
- **Python:** Follow PEP 8, use type hints where reasonable
- **C++:** Follow existing firmware style (see `software/firmware/` examples)
- **Markdown:** Keep public docs brief and scannable (see Documentation section below)

---

## Documentation Standards

**Public-facing docs** (README, guides, API references) must be:

- ✅ **Brief and clear** — Prioritize understanding over completeness
- ✅ **Essential information only** — Remove unnecessary examples or verbosity
- ✅ **Scannable** — Use bullet points, tables, and headings
- ✅ **Honest** — Include limitations and known issues

**This is critical.** Over-documentation is as bad as under-documentation. Ask: "Does this help someone get started or solve a problem?"

---

## Testing

Before opening a PR:

- ✅ Code compiles (firmware) or runs without errors (Python)
- ✅ No linter errors (`ruff` for Python, `cppcheck` for C++)
- ✅ Manual testing on hardware (if applicable)
- ✅ Documentation updated (if behavior changed)

We don't have automated tests yet (Phase 0), but manual validation is expected.

---

## What to Contribute

### 🐛 Bug Reports
- Clear description of problem
- Steps to reproduce
- Expected vs actual behavior
- Hardware/software versions

### 🧪 Validation & Testing
- Hardware replication attempts
- Performance measurements
- Edge case testing
- Calibration procedure validation
- Photos, logs, and measured deltas versus expected behavior

### 📖 Documentation
- Clarity improvements
- Missing explanations
- Example code/scripts
- Translation (after English version stable)

### 💡 Feature Proposals
- Open issue first
- Describe use case and rationale
- Consider scope (Phase 0 vs future)
- Wait for maintainer feedback

### 🔧 Code Contributions
- Bug fixes (always welcome)
- Performance improvements
- New features (discuss first in issue)
- Refactoring (keep PRs focused)

### 🛠️ Maintainer-Friendly Improvements
- Build fixes that preserve current behavior
- Test coverage for existing behavior
- Developer tooling that reduces setup or debugging friction
- Small examples that help reproduce real issues

---

## What NOT to Contribute (Yet)

- ❌ **Canonical hardware geometry changes** — use issues for now while hardware remains STL-only
- ❌ **CAD files / STL modifications intended for merge** — wait for STEP/CAD release stages
- ❌ **Large binary assets** — Use external hosting and link instead
- ❌ **Breaking API changes** — Discuss in issue first
- ❌ **Unrelated features** — Stay focused on Phase 0 scope

---

## License Acknowledgement

By contributing you agree:

- **Code contributions:** Licensed under MIT (same as project software)
- **Hardware contributions:** Subject to phased licensing roadmap (see `hardware/LICENSE.md`)
  - Your contributions remain attributed
  - You agree to licensing transitions as hardware matures
  - No retroactive license changes on code (MIT stays MIT)

---

## Code of Conduct

Be respectful, constructive, and patient. We're building in public, which means:

- ✅ Critique ideas, not people
- ✅ Assume good intent
- ✅ Focus on engineering substance
- ❌ No marketing hype or overpromises
- ❌ No unconstructive negativity

We document failures and limitations transparently — that's a feature, not weakness.

---

## Questions?

- **GitHub Issues:** Technical questions, bug reports, feature proposals, hardware feedback
- **Email:** info@aliahumanoid.com (for non-technical inquiries)
- **Social:** [@AliaHumanoid](https://x.com/AliaHumanoid) on X/Twitter (project updates)

GitHub Discussions are not enabled on this repository at the moment, so Issues are the primary async channel.

---

**Thank you for considering contributing to Alia!** Every improvement — from typo fixes to major features — helps build a more transparent and capable humanoid robotics project.
