# CLAUDE.md

Guidance for Claude Code when working in this repository.

## Commit messages

- Do **not** add a `Co-Authored-By: Claude ...` trailer — or any AI co-author
  attribution — to commit messages.
- This repo sets `includeCoAuthoredBy: false` in `.claude/settings.json`. That
  option only suppresses the trailer Claude Code would append *automatically*;
  it does not stop a trailer that is hand-written into a `-m`/`-F` message. So
  never write the trailer into the message text yourself either.
