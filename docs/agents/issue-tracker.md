# Issue tracker: GitHub

GitHub Issues are the source of truth for bugs and enhancements in `dennismayer/studienarbeit_turtlebot4`. Use the GitHub CLI (`gh`) with that repository explicitly selected.

## Conventions

- Create issues with `gh issue create --repo dennismayer/studienarbeit_turtlebot4`.
- Use the labels documented in `triage-labels.md` when applicable. The `Category:` and `Status:` metadata belong in GitHub labels rather than local Markdown fields.
- Record dependencies using GitHub issue references in the issue body.
- Keep feature specifications, QA reports, and implementation notes under `.scratch/<feature-slug>/`; these provide context but are not the issue tracker.
- Existing local implementation issue files should be migrated to GitHub before being considered active. Link to the resulting GitHub issue from the local feature notes when useful.

## When a skill says "publish to the issue tracker"

Create a GitHub issue in `dennismayer/studienarbeit_turtlebot4`. Include enough context, reproduction steps, and expected behavior for the ticket to stand on its own.

## When a skill says "fetch the issue"

Use `gh issue view <number> --repo dennismayer/studienarbeit_turtlebot4`. The user will normally pass the issue number or URL.

## Wayfinding operations

Used by `/wayfinder`. The **map** is a file with one **child** file per decision ticket. Children live in `decisions/`; implementation issues are tracked separately in GitHub.

- **Map**: `.scratch/<effort>/map.md` (the Destination / Notes / Decisions so far / Added so far / Not yet specified / Out of scope body).
- **Child ticket**: `.scratch/<effort>/decisions/NN-<slug>.md`, numbered from `01`, with the question in the body. A `Type:` line records the ticket type (`research`/`prototype`/`grilling`/`task`); a `Status:` line records `claimed`/`resolved`.
- **Blocking**: a `Blocked by: NN, NN` line near the top. A ticket is unblocked when every file it lists is `resolved` or closed (the Close operation above).
- **Frontier**: scan `.scratch/<effort>/decisions/` for files that are open, unblocked, and unclaimed; first by number wins.
- **Claim**: set `Status: claimed` and save before any work.
- **Resolve**: append the answer under an `## Answer` heading, set `Status: resolved`, then append a context pointer (gist + link) to the map's Decisions-so-far in `map.md`.
