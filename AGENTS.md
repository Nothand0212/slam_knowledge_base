<!-- code-review-graph MCP tools -->
## MCP Tools: code-review-graph

**IMPORTANT: This project has a knowledge graph. ALWAYS use the
code-review-graph MCP tools BEFORE using Grep/Glob/Read to explore
the codebase.** The graph is faster, cheaper (fewer tokens), and gives
you structural context (callers, dependents, test coverage) that file
scanning cannot.

### When to use graph tools FIRST

- **Exploring code**: `semantic_search_nodes` or `query_graph` instead of Grep
- **Understanding impact**: `get_impact_radius` instead of manually tracing imports
- **Code review**: `detect_changes` + `get_review_context` instead of reading entire files
- **Finding relationships**: `query_graph` with callers_of/callees_of/imports_of/tests_for
- **Architecture questions**: `get_architecture_overview` + `list_communities`

Fall back to Grep/Glob/Read **only** when the graph doesn't cover what you need.

### Key Tools

| Tool | Use when |
|------|----------|
| `detect_changes` | Reviewing code changes — gives risk-scored analysis |
| `get_review_context` | Need source snippets for review — token-efficient |
| `get_impact_radius` | Understanding blast radius of a change |
| `get_affected_flows` | Finding which execution paths are impacted |
| `query_graph` | Tracing callers, callees, imports, tests, dependencies |
| `semantic_search_nodes` | Finding functions/classes by name or keyword |
| `get_architecture_overview` | Understanding high-level codebase structure |
| `refactor_tool` | Planning renames, finding dead code |

### Workflow

1. The graph auto-updates on file changes (via hooks).
2. Use `detect_changes` for code review.
3. Use `get_affected_flows` to understand impact.
4. Use `query_graph` pattern="tests_for" to check coverage.

---

## llm-wiki Knowledge Base Maintenance

This repository is a local llm-wiki knowledge base. When editing wiki
content, preserve the knowledge-base conventions in `.wiki-schema.md`.

### Frontmatter Dates

- Every edited wiki page must have YAML frontmatter with an `updated` field.
- When modifying an existing page, set `updated` to the current date in
  `YYYY-MM-DD` format.
- If an edited page has `created` but no `updated`, add `updated` immediately
  after `created`.
- When creating a new wiki page, set both `created` and `updated`.
- Do not leave stale `updated` dates after adding sections, fixing links,
  changing source citations, or updating code snippets.

### Agent Implementation Hints

For implementable pages, especially `wiki/entities/方法-*.md` and
`wiki/entities/概念-*.md`, add or maintain an `Agent 实现提示` section before
`相关页面` when the page describes an algorithm, engineering method, residual,
state update, data structure, or reusable coding pattern.

The section should include:

- `### 适用场景`
- `### 输入输出契约`
- `### 实现骨架（伪代码）`
- `### 关键源码片段`
- `### 实现注意事项`
- `### 源码检索锚点`

Rules for this section:

- Prefer real source anchors from `raw/codes/{project}` whenever local source
  exists.
- Mark real snippets with path and line range, for example
  `raw/codes/open_vins/ov_core/src/track/TrackKLT.cpp:L56-L71`.
- Keep each real code snippet short, normally 10-40 lines.
- Use pseudocode for transferable structure; use real code only for verified
  API calls, residual/Jacobian structure, state updates, data layout, or edge
  conditions.
- Do not invent "real" code. If code is not copied from `raw/codes` or another
  verified source, label it as pseudocode.
- For resize/crop/remap/deskew/time sync/exposure/color-space processing,
  explicitly state input/output coordinate frames, units, and how the step
  affects downstream residuals or state estimation.

### Source And Link Hygiene

- `raw/codes/` is a read-only source snapshot for wiki citations and short
  code snippets. Do not edit files under `raw/codes/`.
- Subprojects under `raw/codes/` must not keep nested `.git` metadata. Treat
  them as copied source snapshots, not submodules or development worktrees.
- Before replacing or adding a `raw/codes/` project snapshot, record its
  upstream URL, branch/tag, commit hash, copy date, and any local dirty files in
  `raw/codes/MANIFEST.md`.
- Verify every `raw/codes/...:Lx-Ly` anchor points to an existing file and a
  reasonable line range.
- Keep Markdown code fences balanced.
- Avoid malformed wikilinks such as `[[A, B]]`.
- In Markdown tables, if a wikilink needs an alias, write
  `[[页面名\|别名]]` so the pipe is not parsed as a table separator.

### Minimum Verification After Wiki Edits

After changing wiki pages, run checks appropriate to the edit:

- Confirm edited pages have `updated: YYYY-MM-DD`.
- Confirm `Agent 实现提示` pages contain the six required subsections.
- Confirm `raw/codes/...:Lx-Ly` anchors exist and line ranges are short.
- Check for malformed wikilinks and unbalanced code fences.
- Run `git diff --check` on edited wiki files.
