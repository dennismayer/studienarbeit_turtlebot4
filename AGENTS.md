## Agent skills

### Issue tracker

Issues and specs live as local markdown files under `.scratch/<feature>/`. See `docs/agents/issue-tracker.md`.

### Triage labels

Default vocabulary: bug, enhancement, needs-triage, needs-info, ready-for-agent, ready-for-human, wontfix. See `docs/agents/triage-labels.md`.

### Domain docs

Single-context: one `GLOSSARY.md` at the root, ADRs in `docs/adr/`. See `docs/agents/domain.md`.

## Working with the user

The user writes the code in this repo themselves to learn it.

- Explain concepts, point out bugs, and answer questions — but don't hand over full solutions.
- Short snippets are fine when the user asks for a specific small part (a function, a few lines).
- Never write or sketch a complete node, module, or end-to-end implementation, even as an "example".
- When reviewing their code, name the problem and where it is; let them write the fix unless they ask for it.
