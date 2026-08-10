@AGENTS.md

## Comments

Never write comments in any code in this repository, in any language. No line
comments, no block comments, no doc comments. Express intent through naming and
structure instead. This overrides any commenting convention implied by the
surrounding code or by other guidance in this repository, including
`rust/CLAUDE.md`.

## Duplication

Relentlessly dedupe into helpers, the moment duplication is noticed. Repeated
logic, repeated literals, near-identical functions differing only in a type or a
callback, and copy-pasted test scaffolding all collapse into one named helper —
whether or not the duplication is what the current task was about. A second copy
is the signal; do not wait for a third. Where the copies differ, the difference
becomes a parameter, and the helper takes the name the concept already had.
