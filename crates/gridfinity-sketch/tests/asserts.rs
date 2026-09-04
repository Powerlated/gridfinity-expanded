//! Assertion coverage, checked against the crate's own syntax tree.
//!
//! `AGENTS.md` asks that every relied-on invariant be asserted at the point it
//! is relied on, and that the assertion state the invariant rather than a proxy
//! for it. Prose cannot enforce that. This reads this crate's own `src/**/*.rs`
//! with `syn` and holds four rules over it, three absolute and one a ratchet.
//!
//! Each crate of the model stack carries its own copy over its own `BUDGET`,
//! and the three tables together are the one table `gridfinity-cad` held before
//! the kernel was split out of it.
//!
//! It is an AST pass and not a grep on purpose: `assert` inside a string
//! literal, a comment or a doc example is not an assertion, `#[cfg(test)]`
//! bodies are not production code, and a macro's *arity* -- which is how
//! "carries a message" is decided below -- is not something a regex can see.
//!
//! What it deliberately does not do is count assertions and call a number
//! sufficient. A vacuous `assert!(true)` would satisfy any density floor while
//! asserting nothing, so the only counting rule here is a ratchet on the
//! functions that have *no* assertion at all: it can never rise, and lowering
//! it is a change to this file that someone has to justify.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use syn::punctuated::Punctuated;
use syn::visit::Visit;

/// A function this long is doing enough that at least one thing it relies on is
/// worth stating. Below it, the ratchet says nothing -- a four-line accessor
/// with no assertion is not a defect.
const BIG_FN_STATEMENTS: usize = 20;

/// Functions of `BIG_FN_STATEMENTS` statements or more that assert nothing, per
/// file. Every entry is a place where the invariants are still only in the
/// author's head; the numbers exist to be driven down.
///
/// The test fails if a file goes **over** its budget and equally if it comes in
/// **under** one without the budget being lowered, so the table cannot quietly
/// drift away from the code it describes.
const BUDGET: &[(&str, usize)] = &[
    ("rectregion.rs", 2),
    ("region2d.rs", 3),
];

// ---------------------------------------------------------------------------
// Walking the tree
// ---------------------------------------------------------------------------

fn sources() -> Vec<(String, PathBuf)> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut out = Vec::new();
    let mut stack = vec![root.clone()];
    while let Some(d) = stack.pop() {
        let dir = std::fs::read_dir(&d).expect("the crate's src directory is readable");
        for e in dir {
            let p = e.expect("a readable directory entry").path();
            if p.is_dir() {
                stack.push(p);
            } else if p.extension().is_some_and(|x| x == "rs") {
                let rel = p
                    .strip_prefix(&root)
                    .expect("every source walked started at src")
                    .to_string_lossy()
                    .replace('\\', "/");
                out.push((rel, p));
            }
        }
    }
    out.sort();
    assert!(
        !out.is_empty(),
        "found no sources under src -- the walk is looking in the wrong place"
    );
    out
}

/// How many arguments an assertion macro needs before any message.
fn required_arity(name: &str) -> Option<usize> {
    match name {
        "assert" => Some(1),
        "assert_eq" | "assert_ne" => Some(2),
        _ => None,
    }
}

struct Finding {
    line: usize,
    what: String,
}

#[derive(Default)]
struct FnFacts {
    name: String,
    line: usize,
    stmts: usize,
    asserts: usize,
}

#[derive(Default)]
struct Body {
    facts: FnFacts,
}

impl<'ast> Visit<'ast> for Body {
    fn visit_stmt(&mut self, s: &'ast syn::Stmt) {
        self.facts.stmts += 1;
        syn::visit::visit_stmt(self, s);
    }
    fn visit_macro(&mut self, m: &'ast syn::Macro) {
        if required_arity(&macro_name(m)).is_some() {
            self.facts.asserts += 1;
        }
        syn::visit::visit_macro(self, m);
    }
}

fn macro_name(m: &syn::Macro) -> String {
    m.path
        .segments
        .last()
        .map(|s| s.ident.to_string())
        .unwrap_or_default()
}

fn is_cfg_test(attrs: &[syn::Attribute]) -> bool {
    attrs.iter().any(|a| {
        a.path().is_ident("cfg")
            && a.parse_args::<syn::Meta>()
                .is_ok_and(|m| m.path().is_ident("test"))
    })
}

#[derive(Default)]
struct FileScan {
    in_test: usize,
    fns: Vec<FnFacts>,
    debug_asserts: Vec<Finding>,
    bare_unwraps: Vec<Finding>,
    silent_asserts: Vec<Finding>,
}

impl FileScan {
    fn record(&mut self, name: String, line: usize, block: &syn::Block) {
        let mut b = Body::default();
        b.visit_block(block);
        self.fns.push(FnFacts {
            name,
            line,
            ..b.facts
        });
    }
}

impl<'ast> Visit<'ast> for FileScan {
    fn visit_item_mod(&mut self, m: &'ast syn::ItemMod) {
        let t = is_cfg_test(&m.attrs);
        self.in_test += usize::from(t);
        syn::visit::visit_item_mod(self, m);
        self.in_test -= usize::from(t);
    }

    fn visit_item_fn(&mut self, f: &'ast syn::ItemFn) {
        let t = is_cfg_test(&f.attrs);
        self.in_test += usize::from(t);
        if self.in_test == 0 {
            self.record(f.sig.ident.to_string(), line_of(&f.sig.ident), &f.block);
        }
        syn::visit::visit_item_fn(self, f);
        self.in_test -= usize::from(t);
    }

    fn visit_impl_item_fn(&mut self, f: &'ast syn::ImplItemFn) {
        let t = is_cfg_test(&f.attrs);
        self.in_test += usize::from(t);
        if self.in_test == 0 {
            self.record(f.sig.ident.to_string(), line_of(&f.sig.ident), &f.block);
        }
        syn::visit::visit_impl_item_fn(self, f);
        self.in_test -= usize::from(t);
    }

    fn visit_macro(&mut self, m: &'ast syn::Macro) {
        let name = macro_name(m);
        let line = m
            .path
            .segments
            .last()
            .map(|s| line_of(&s.ident))
            .unwrap_or(0);
        if name.starts_with("debug_assert") {
            self.debug_asserts.push(Finding {
                line,
                what: format!("{name}!"),
            });
        }
        if self.in_test == 0 {
            if let Some(arity) = required_arity(&name) {
                // `parse_terminated` is the same grammar the real macro uses, so
                // "has a message" is exactly "has more arguments than the
                // comparison needs" -- not "contains a string somewhere".
                let args =
                    m.parse_body_with(Punctuated::<syn::Expr, syn::Token![,]>::parse_terminated);
                if let Ok(args) = args {
                    if args.len() <= arity {
                        self.silent_asserts.push(Finding {
                            line,
                            what: format!("{name}!"),
                        });
                    }
                }
            }
        }
        syn::visit::visit_macro(self, m);
    }

    fn visit_expr_method_call(&mut self, e: &'ast syn::ExprMethodCall) {
        if self.in_test == 0 && e.method == "unwrap" && e.args.is_empty() {
            self.bare_unwraps.push(Finding {
                line: line_of(&e.method),
                what: ".unwrap()".to_string(),
            });
        }
        syn::visit::visit_expr_method_call(self, e);
    }
}

fn line_of(i: &syn::Ident) -> usize {
    i.span().start().line
}

fn scan() -> Vec<(String, FileScan)> {
    sources()
        .into_iter()
        .map(|(rel, path)| {
            let src = std::fs::read_to_string(&path).expect("a readable source file");
            let ast = syn::parse_file(&src)
                .unwrap_or_else(|e| panic!("the crate's own source failed to parse: {rel}: {e}"));
            let mut s = FileScan::default();
            s.visit_file(&ast);
            assert_eq!(
                s.in_test, 0,
                "{rel}: the #[cfg(test)] depth did not return to zero"
            );
            (rel, s)
        })
        .collect()
}

fn report(hits: &[(&String, &Finding)]) -> String {
    hits.iter()
        .map(|(f, h)| format!("\n  {}:{} {}", f, h.line, h.what))
        .collect()
}

// ---------------------------------------------------------------------------
// The rules
// ---------------------------------------------------------------------------

/// `--release` compiles `debug_assert!` out, and the whole workspace is tested
/// in release, so one here is an invariant nobody checks.
#[test]
fn no_invariant_is_checked_only_in_debug() {
    let files = scan();
    let hits: Vec<_> = files
        .iter()
        .flat_map(|(f, s)| s.debug_asserts.iter().map(move |h| (f, h)))
        .collect();
    assert!(
        hits.is_empty(),
        "{} debug_assert(s): the release build drops them, so use assert!{}",
        hits.len(),
        report(&hits)
    );
}

/// A failure has to name the invariant that failed. `unwrap` names nothing --
/// it reports the enum variant it found, not the property that was supposed to
/// hold -- so production code uses `expect` with the invariant written out.
#[test]
fn every_failure_in_production_code_names_its_invariant() {
    let files = scan();
    let hits: Vec<_> = files
        .iter()
        .flat_map(|(f, s)| s.bare_unwraps.iter().map(move |h| (f, h)))
        .collect();
    assert!(
        hits.is_empty(),
        "{} bare unwrap(s) outside tests: say what must hold with .expect(\"..\"){}",
        hits.len(),
        report(&hits)
    );
}

/// An assertion with no message states a condition and not the property the
/// condition stands for. The message is where the invariant is written down, so
/// production assertions carry one.
#[test]
fn every_production_assertion_states_what_it_asserts() {
    let files = scan();
    let hits: Vec<_> = files
        .iter()
        .flat_map(|(f, s)| s.silent_asserts.iter().map(move |h| (f, h)))
        .collect();
    assert!(
        hits.is_empty(),
        "{} assertion(s) outside tests with no message{}",
        hits.len(),
        report(&hits)
    );
}

/// The ratchet. Every function long enough to be doing real work either asserts
/// something or is counted here, and the count per file may only fall.
#[test]
fn the_number_of_functions_that_assert_nothing_only_falls() {
    let budget: BTreeMap<&str, usize> = BUDGET.iter().copied().collect();
    assert_eq!(
        budget.len(),
        BUDGET.len(),
        "BUDGET names a file twice, so one of its entries is unreachable"
    );

    let mut over = Vec::new();
    let mut under = Vec::new();
    let mut seen: BTreeMap<&str, ()> = BTreeMap::new();
    for (rel, s) in &scan() {
        let bare: Vec<&FnFacts> = s
            .fns
            .iter()
            .filter(|f| f.asserts == 0 && f.stmts >= BIG_FN_STATEMENTS)
            .collect();
        let allowed = budget.get(rel.as_str()).copied().unwrap_or(0);
        if let Some((k, _)) = budget.get_key_value(rel.as_str()) {
            seen.insert(k, ());
        }
        if bare.len() > allowed {
            over.push(format!(
                "\n  {rel}: {} function(s) assert nothing, budget {allowed}{}",
                bare.len(),
                bare.iter()
                    .map(|f| format!(
                        "\n      {}:{} {} ({} statements)",
                        rel, f.line, f.name, f.stmts
                    ))
                    .collect::<String>()
            ));
        } else if bare.len() < allowed {
            under.push(format!(
                "\n  {rel}: {} function(s) assert nothing, budget {allowed} -- lower it to {}",
                bare.len(),
                bare.len()
            ));
        }
    }
    let stale: Vec<&str> = BUDGET
        .iter()
        .map(|&(f, _)| f)
        .filter(|f| !seen.contains_key(f))
        .collect();

    assert!(
        over.is_empty(),
        "a function of {BIG_FN_STATEMENTS}+ statements that asserts nothing was added:{}",
        over.concat()
    );
    assert!(
        under.is_empty(),
        "assertions were added -- take the credit by lowering the budget:{}",
        under.concat()
    );
    assert!(
        stale.is_empty(),
        "BUDGET names {} file(s) that no longer exist: {stale:?}",
        stale.len()
    );
}
