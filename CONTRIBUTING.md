# Contributing to Alia

Thanks for your interest in contributing! 

## Why Contribute to Alia?

Alia is exploring humanoid mechanics with **radical transparency**: we document what works ✅ and what doesn't ⚠️, inside real human constraints.

**What makes this project different:**
- **Honesty over hype** — We share failures and iterations, not just successes
- **Real constraints** — Building inside human proportions forces innovative solutions
- **Progressive open source** — Phased licensing roadmap toward full collaboration
- **Engineering depth** — Tendon-driven actuation, biomechanics validation, hardware iteration

Phase 0 is early-stage, but we're building a foundation for genuine collaboration. If you value:
- Transparent engineering process
- Bio-inspired robotics design
- Open hardware with clear governance
- Community-driven development

...then Alia might be a project worth your time.

---

## Ground Rules

- Use English for code comments, documentation, and issues
- No CAD / STL uploads yet (Phase 0 — releasing soon with clear licensing)
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

1. **Create an issue first** (if non-trivial change)
   - Describe the problem or improvement
   - Reference related decisions (if known)
   - Wait for feedback before major work

2. **Fork and branch**
   - Fork the repo
   - Create feature branch: `git checkout -b feature/your-topic`

3. **Make changes**
   - Follow style guidelines (see below)
   - Include tests if applicable
   - Update docs if behavior changes

4. **Sign your commits**
   - Always use `git commit -s`
   - Check: `git log -1` should show `Signed-off-by: Your Name <your@email.com>`

5. **Open PR**
   - Reference issue: "Closes #123" or "Related to #456"
   - Describe what changed and why
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

---

## What NOT to Contribute (Yet)

- ❌ **CAD files / STL modifications** — Phase 0 hardware not yet released
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

- **GitHub Issues:** Technical questions, bug reports, feature proposals
- **Email:** info@aliahumanoid.com (for non-technical inquiries)
- **Social:** [@AliaHumanoid](https://x.com/AliaHumanoid) on X/Twitter (project updates)

---

**Thank you for considering contributing to Alia!** Every improvement — from typo fixes to major features — helps build a more transparent and capable humanoid robotics project.
