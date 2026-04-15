#!/usr/bin/env python3
"""Clean up pandoc's raw LaTeX output and wrap each as a kaobook chapter.

Book-style cleanup (2026-04-15):
  * Chapter titles include the week for clarity: kaobook renders
    "Chapter X  Title (Week N)" automatically.
  * The redundant top-level `\\section{Week N: ...}` (or for ROS2 chapters,
    `\\section{Company --- Technical Challenge}`) is dropped, since the
    chapter title already conveys that information.
  * All `\\subsection` are promoted to `\\section`, `\\subsubsection` to
    `\\subsection`, `\\paragraph` to `\\subsubsection`. Each chapter now has
    proper top-level sections rather than one outer section with many
    subsections under it.
  * "Section N:", "Section N --- Part M: ", and "N.M:" numeric prefixes are
    stripped from heading text so kaobook's chapter-section numbering isn't
    duplicated.
  * Emojis are stripped from heading text.
  * Image paths rewritten to images/<week>/basename.
"""
import re
import shutil
import subprocess
from pathlib import Path

BOOK = Path("/home/it-services/control-for-robots/book")
# Read the lecture HTMLs from the PUBLIC repo (the parent of book/) so that
# swaps and renumbering applied to the public content automatically flow
# into the book when chapters are regenerated.
SRC_WS = BOOK.parent

TITLES = {
    "week_01": "Manipulator Dynamics and Computed-Torque Control (Week 1)",
    "week_02": "Model Predictive Control --- Introduction (Week 2)",
    "week_03": "Model Predictive Control --- Design and Simulation (Week 3)",
    "week_04": "Adaptive Control (Week 4)",
    "week_05": "Robust Control (Week 5)",
    # Sliding Mode Control and Backstepping swapped (2026-04-15): SMC is
    # now course Week 6 (was Week 7), Backstepping is course Week 7 (was
    # Week 6). Both the (Week N) suffix and the pandoc source reflect the
    # new numbering.
    "week_06": "Sliding Mode Control (Week 6)",
    "week_07": "Backstepping Control (Week 7)",
    "week_08": "Nonlinear Control Optimization for Robotic Systems (Week 8)",
    "week_09": "ROS\\,2 Control Implementation (Week 9)",
    "week_10": "ROS\\,2 Quadrotor Control (Week 10)",
    "week_11": "ROS\\,2 Navigation Control (Week 11)",
    "week_12": "Controller Comparison and Industry Applications (Week 12)",
}

EMOJI_RE = re.compile(
    "["
    "\U0001F300-\U0001FAFF"
    "\U00002600-\U000027BF"
    "\U0001F000-\U0001F9FF"
    "\U0001F680-\U0001F6FF"
    "\U0001F900-\U0001F9FF"
    "\uFE00-\uFE0F"          # variation selectors 1–16
    "\u200D"                 # zero-width joiner
    "\u2B00-\u2BFF"          # misc symbols & arrows (often emoji-modifier)
    "]+",
    flags=re.UNICODE,
)

# Heading-text prefix patterns to strip (order matters: longer first)
PREFIX_PATTERNS = [
    re.compile(r"^\s*Section\s+\d+\s*(?::|---)\s*Part\s+\d+\s*(?::|---)\s*", re.IGNORECASE),
    re.compile(r"^\s*Section\s+\d+\s*(?::|---)\s*", re.IGNORECASE),
    re.compile(r"^\s*Part\s+\d+[a-z]?\s*(?::|---)\s*", re.IGNORECASE),
    # numeric prefixes with or without colon: "3.1.1: Foo", "3.1: Foo", "3.1 Foo"
    re.compile(r"^\s*\d{1,2}\.\d{1,2}\.\d{1,2}\s*[:\s]\s*(?=[A-Z])", re.IGNORECASE),
    re.compile(r"^\s*\d{1,2}\.\d{1,2}\s*[:\s]\s*(?=[A-Z])", re.IGNORECASE),
]

REDUNDANT_FIRST_SECTION = re.compile(
    r"Week\s+\d+\s*:|"
    r"Technical\s+Challenge|"
    r"\bChallenge\b",
    re.IGNORECASE,
)

# Section titles that are boilerplate / metadata and should be consolidated
# under a synthetic "Overview" or "Wrap-Up" section so each chapter has the
# 7–8 top-level sections a 3-hour lecture actually warrants.
BOILERPLATE_RE = re.compile(
    r"(?:"
    r"Learning\s+Outcomes"
    r"|Session\s+Roadmap"
    r"|Week\s*\d+\s*Recall"
    r"|Exam\s+Practice"
    r"|Company\s+Brief"
    r"|Skills\s+You\s+Will\s+Demonstrate"
    r"|Deliverables"
    r"|Evaluation\s+Criteria"
    r"|Provided\s+Bag\s+Data"
    r"|Hands[- ]?On\s+Activit"
    r"|Activity\s+\d+"
    r"|(?:MATLAB\s+)?Lab\s+(?:Activit|Exercis)"
    r"|End[- ]of[- ]Lecture\s+Quiz"
    r"|Test\s+Your\s+Understanding"
    r"|Quick\s+Check"
    r"|\bQuiz\b"
    r"|Next\s+Week"
    r"|Preparation\s+for\s+Next"
    r"|Running\s+the\s+ROS"
    r"|Appendix"
    r"|Think[- ]Pair[- ]Share"
    r"|Course\s+Wrap[- ]?Up"
    r"|Wrap[- ]?Up"
    r"|Course\s+Recap"
    r"|Closing\s+Remarks"
    r"|(?:Week\s*\d+\s+)?Summary"
    r"|^Break$"
    r")",
    re.IGNORECASE,
)

# Sections to DROP entirely (not just demote / consolidate). The
# "Recommended Approach" table of 3-hour timings is lecture-prep
# material, not book content, and the user asked for it gone.
DROP_SECTION_RE = re.compile(
    r"(?:"
    r"Recommended\s+Approach"
    r"|Session\s+Roadmap"
    r")",
    re.IGNORECASE,
)

def _match_balanced_brace(text, start):
    """Given text[start] == '{', return the index AFTER the matching '}'.
    Counts nested braces properly. Returns -1 if unbalanced."""
    assert text[start] == '{'
    depth = 1
    i = start + 1
    while i < len(text):
        c = text[i]
        if c == '\\' and i + 1 < len(text):
            i += 2        # skip escaped char
            continue
        if c == '{':
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0:
                return i + 1
        i += 1
    return -1


def _replace_underbraces(text):
    """Replace every \\underbrace{X}_{Y} with \\underset{\\text{\\footnotesize Y}}{\\underline{X}}
    using real brace-counting rather than regex."""
    out = []
    i = 0
    prefix = r"\underbrace"
    while i < len(text):
        j = text.find(prefix, i)
        if j == -1:
            out.append(text[i:])
            break
        out.append(text[i:j])
        # expect { right after \underbrace
        k = j + len(prefix)
        if k >= len(text) or text[k] != '{':
            out.append(text[j])
            i = j + 1
            continue
        expr_end = _match_balanced_brace(text, k)
        if expr_end == -1:
            out.append(text[j])
            i = j + 1
            continue
        expr = text[k+1:expr_end-1]
        # expect _{  or _ followed by single char
        if expr_end >= len(text) or text[expr_end] != '_':
            out.append(text[j:expr_end])
            i = expr_end
            continue
        m = expr_end + 1
        if m >= len(text) or text[m] != '{':
            out.append(text[j:expr_end])
            i = expr_end
            continue
        lbl_end = _match_balanced_brace(text, m)
        if lbl_end == -1:
            out.append(text[j:expr_end])
            i = expr_end
            continue
        label = text[m+1:lbl_end-1]
        # If label is wrapped in \text{...}, unwrap it so we don't nest
        tm = re.fullmatch(r"\\text\{(.*)\}", label, flags=re.DOTALL)
        if tm:
            label = tm.group(1)
        # Emit replacement
        out.append(
            r"\underset{\text{\footnotesize " + label + r"}}{\underline{" + expr + r"}}"
        )
        i = lbl_end
    return "".join(out)


def _detect_code_lang(code: str) -> str:
    """Heuristically detect whether a code snippet is Python, MATLAB,
    bash or plain text. Used to tag the <pre><code class="language-X">
    so pandoc emits a properly highlighted lstlisting.
    """
    s = code.strip()
    first_line = s.splitlines()[0] if s else ""
    s_lower = s.lower()
    # Obvious shells
    if first_line.startswith("$ ") or re.search(r"\b(sudo|apt-get|docker|ros2 (run|launch|topic)|colcon|git |cd |source |export )\b", s):
        return "bash"
    # Python markers
    if re.search(r"^\s*(def |class |import |from |if __name__)\b", s, re.MULTILINE) \
            or "rclpy" in s or "self\\." in s or "numpy" in s_lower or "matplotlib" in s_lower:
        return "python"
    # MATLAB markers: comments begin with %, function keyword, semicolons at EOL,
    # end keyword, MATLAB-style transpose (')
    if re.search(r"^\s*(function\s+\[|%%|% [A-Z])", s, re.MULTILINE) \
            or re.search(r";\s*$", s, re.MULTILINE) and "%" in s:
        return "matlab"
    return ""


def _preprocess_html_codeblocks(html: str) -> str:
    """Convert our custom <div class="code-block">...</div> and
    <div class="pseudocode-block">...</div> to <pre><code class="...">
    so pandoc emits lstlisting with the correct language setting.
    """
    def _strip_markup(inner: str) -> str:
        inner = re.sub(r"<[^>]+>", "", inner)
        inner = (inner.replace("&lt;", "<").replace("&gt;", ">")
                      .replace("&amp;", "&").replace("&nbsp;", " ")
                      .replace("&quot;", '"'))
        return inner

    def _repl(m):
        inner = m.group(1)
        inner = re.sub(r'<div class="pseudocode-header">.*?</div>', "",
                       inner, flags=re.DOTALL)
        code = _strip_markup(inner).strip("\n")
        lang = _detect_code_lang(code)
        cls = f' class="language-{lang}"' if lang else ""
        return f"<pre><code{cls}>{code}</code></pre>"

    html = re.sub(
        r'<div class="code-block"[^>]*>(.*?)</div>',
        _repl, html, flags=re.DOTALL,
    )
    html = re.sub(
        r'<div class="pseudocode-block"[^>]*>(.*?)</div>',
        _repl, html, flags=re.DOTALL,
    )
    return html


def _label_for_run(run_titles):
    """Pick an informative wrapper label based on what's in the boilerplate run.

    Priority ordering matters: intro-type content beats activities
    (e.g. 'Week N Recall --- Exam Practice' contains 'practice' but is clearly
    an introductory warm-up, not an activity).
    """
    joined = " ".join(run_titles).lower()
    # Introductory / front-matter content
    intro_keywords = (
        "learning outcomes", "session roadmap", "recall", "company brief",
        "recommended approach", "deliverables", "evaluation criteria",
        "skills you will demonstrate", "provided bag data", "exam practice",
    )
    if any(k in joined for k in intro_keywords):
        return "Overview"
    # Summary / outro content
    summary_keywords = (
        "next week", "preparation for next", "summary", "course wrap",
        "course recap", "closing remarks",
    )
    if any(k in joined for k in summary_keywords):
        return "Summary and Next Steps"
    # Mid-chapter activities / hands-on / quizzes
    return "Activities and Wrap-Up"


def consolidate_boilerplate(body: str, week: str) -> str:
    """Group runs of boilerplate \\section into a single synthetic section
    whose content becomes sub-sections of that wrapper.

    If the same wrapper label would be used for two separate runs in a
    chapter (e.g., an \"Activities and Wrap-Up\" run mid-chapter and
    another one at the end), the second run is merged into the first
    in place of creating duplicate wrapper sections.
    """
    pattern = re.compile(r"^(\\section\{[^{}]*\})\s*$", re.MULTILINE)
    parts = pattern.split(body)
    if len(parts) < 3:
        return body

    preamble = parts[0]
    headers = parts[1::2]
    bodies = parts[2::2]

    # First pass: classify every section, dropping unwanted ones entirely
    entries = []   # list of dicts: {kind, label_or_title, body, subs}
    i = 0
    while i < len(headers):
        title = re.match(r"\\section\{([^{}]*)\}", headers[i]).group(1)
        # Drop sections that are course-management content, not book content
        if DROP_SECTION_RE.search(title):
            i += 1
            continue
        if BOILERPLATE_RE.search(title):
            # collect contiguous boilerplate run
            run = []
            while i < len(headers):
                t = re.match(r"\\section\{([^{}]*)\}", headers[i]).group(1)
                if BOILERPLATE_RE.search(t):
                    run.append((t, bodies[i]))
                    i += 1
                else:
                    break
            entries.append({
                "kind": "wrapper",
                "label": _label_for_run([t for t, _ in run]),
                "subs": run,
            })
        else:
            entries.append({
                "kind": "substantive",
                "title": title,
                "body": bodies[i],
            })
            i += 1

    # Second pass: merge wrapper entries that share the same label so each
    # chapter has at most one "Overview", one "Activities and Wrap-Up", and
    # one "Summary and Next Steps"; subsequent duplicates fold into the first.
    canonical = {}       # label -> index in entries of the first occurrence
    merged = []
    for e in entries:
        if e["kind"] == "wrapper":
            if e["label"] in canonical:
                # merge subs into the earlier wrapper
                merged[canonical[e["label"]]]["subs"].extend(e["subs"])
                continue
            canonical[e["label"]] = len(merged)
        merged.append(e)

    # Emit
    out = [preamble]
    for e in merged:
        if e["kind"] == "substantive":
            out.append(f"\\section{{{e['title']}}}\n")
            out.append(e["body"])
        else:
            out.append(f"\\section{{{e['label']}}}\n\n")
            for t, b in e["subs"]:
                out.append(f"\\subsection{{{t}}}\n")
                out.append(b)
    return "".join(out)


def _clean_heading_text(text: str) -> str:
    t = EMOJI_RE.sub("", text).strip()
    for rx in PREFIX_PATTERNS:
        t = rx.sub("", t, count=1).strip()
    # tidy double spaces left by stripping
    t = re.sub(r"\s{2,}", " ", t)
    return t


def copy_figures_for_week(week):
    """Pull all simulation-output PNGs for this week into book/images/<week>/.
    Sources checked (if present):
      * <public_repo>/<week>/sims/figures/*.png    (weeks 8, 9)
      * <public_repo>/results/<week>/*.png          (weeks 1, 3--7 — MATLAB)
    """
    dst = BOOK / "images" / week
    dst.mkdir(parents=True, exist_ok=True)

    sources = [
        SRC_WS / week / "sims" / "figures",
        SRC_WS / "results" / week,
    ]
    for src in sources:
        if not src.exists():
            continue
        for p in src.glob("*.png"):
            if p.stat().st_size == 0:
                continue        # skip empty / corrupt PNGs
            shutil.copy(p, dst / p.name)


# Captions for the canonical MATLAB output figures so the book has a
# descriptive label on each rather than the raw filename.
FIGURE_CAPTIONS = {
    "manipulator_tracking.png": "Manipulator joint-space tracking.",
    "mobile_robot_tracking.png": "Differential-drive mobile robot trajectory tracking.",
    "quadrotor_tracking.png": "Quadrotor position / attitude tracking.",
    "performance_summary.png": "Performance summary across the three robotic platforms.",
    "smc_results.png": "Sliding Mode Control: baseline vs.\\ optimised gains on manipulator, mobile, quadrotor.",
    "adaptive_results.png": "Adaptive control: baseline vs.\\ optimised gains on all three robots.",
    "robust_results.png": "Robust control: baseline vs.\\ min-max optimised gains on all three robots.",
    "backstepping_results.png": "Backstepping: baseline vs.\\ optimised virtual-control gains on all three robots.",
    "nmpc_results.png": "Nonlinear MPC (iLQR): short vs.\\ long prediction horizon on all three robots.",
    "nmpc_unicycle_trajectory.png": "NMPC closed-loop XY trajectory of the unicycle converging onto the reference circle.",
    "nmpc_unicycle_commands.png": "NMPC control commands $v(t), \\omega(t)$ with saturation limits.",
    "nmpc_unicycle_errors.png": "NMPC closed-loop position and heading errors versus time.",
}


def build_simulation_results_block(week: str) -> str:
    """Emit a LaTeX section that includes every simulation figure we have
    for this week, so the book contains the MATLAB / Python output plots
    right alongside the lecture text they belong to.
    """
    img_dir = BOOK / "images" / week
    if not img_dir.exists():
        return ""
    pngs = sorted(img_dir.glob("*.png"))
    if not pngs:
        return ""

    lines = ["\n% --- auto-generated simulation figures ---\n",
             "\\section{Simulation Results}\n\n",
             "The figures below are the output of the MATLAB (or Python) "
             "simulation scripts that accompany this lecture. Each figure "
             "is reproducible by running the corresponding script in the "
             "companion repository; see Section~\\ref{sec:repository} of "
             "the preface for instructions.\n\n"]
    def _auto_caption(stem: str) -> str:
        words = re.split(r"[_\-]+", stem)
        acronyms = {"lqr", "lqg", "mpc", "smc", "pd", "pid", "3d", "2d",
                    "2dof", "3dof", "dof", "ros", "ros2", "nmpc"}
        out = []
        for i, w in enumerate(words):
            if w.lower() in acronyms:
                out.append(w.upper())
            elif i == 0:
                out.append(w.capitalize())
            else:
                out.append(w.lower())
        return " ".join(out) + "."

    for p in pngs:
        caption = FIGURE_CAPTIONS.get(p.name, _auto_caption(p.stem))
        lines.append(
            "\\begin{figure}[ht]\n"
            "    \\centering\n"
            f"    \\includegraphics[width=0.92\\textwidth,keepaspectratio]{{images/{week}/{p.name}}}\n"
            f"    \\caption{{{caption}}}\n"
            "\\end{figure}\n\n"
        )
    return "".join(lines)


def transform_body(week: str, body: str) -> str:
    # ---- image paths ----
    def _rewrite_img(m):
        path = m.group(1).strip()
        basename = path.split("/")[-1]
        return f"\\includegraphics[width=0.85\\textwidth]{{images/{week}/{basename}}}"
    body = re.sub(
        r"\\includegraphics(?:\[[^\]]*\])?\{([^}]+\.png)\}",
        _rewrite_img, body,
    )

    # ---- drop pandoc's hypertarget wrappers and their trailing `}` ----
    body = re.sub(r"\\hypertarget\{[^}]*\}\{%?\s*\n?", "", body)

    # ---- drop inline \label that sit after heading brace: "\subsection{...}\label{...}}"
    body = re.sub(
        r"(\\(?:chapter|section|subsection|subsubsection|paragraph)\{[^{}]*\})\\label\{[^}]*\}\}?",
        r"\1",
        body,
    )
    # any remaining stand-alone labels
    body = re.sub(r"\\label\{[^}]*\}", "", body)
    # orphan closing braces on their own lines
    body = re.sub(r"^\}\s*$", "", body, flags=re.MULTILINE)
    body = re.sub(r"\\begin\{verbatim\}\s*\\end\{verbatim\}", "", body)

    # ---- drop the first redundant \section header (Week N / Challenge) ----
    m = re.search(r"\\section\{([^{}]*)\}", body)
    if m and REDUNDANT_FIRST_SECTION.search(m.group(1)):
        body = body[:m.start()] + body[m.end():]

    # ---- STRUCTURED promotion: use the ORIGINAL heading text's numeric
    # prefix (e.g. "3.1: ..." or "Section 3: ...") to decide the final level,
    # falling back to a one-level pandoc→book promotion if no prefix is found.
    _DEFAULT_PROMO = {
        "section":       "section",       # pandoc's outer \section was already dropped
        "subsection":    "section",
        "subsubsection": "subsection",
        "paragraph":     "subsubsection",
    }

    def _assign_level(cmd, title):
        t = title.strip()
        if re.match(r"\s*Section\s+\d+\s*[:\-]", t, re.IGNORECASE):
            return "section"
        if re.match(r"\s*\d+\.\d+\.\d+\s*[:\s]", t):
            return "subsubsection"
        if re.match(r"\s*\d+\.\d+\s*[:\s]", t):
            return "subsection"
        return _DEFAULT_PROMO.get(cmd, cmd)

    def _promote(m):
        cmd = m.group(1)
        title = m.group(2)
        new_cmd = _assign_level(cmd, title)
        return f"\\{new_cmd}{{{title}}}"

    body = re.sub(
        r"\\(section|subsection|subsubsection|paragraph)\{([^{}]*)\}",
        _promote,
        body,
    )

    # ---- clean heading text: strip emojis + "Section N:" style prefixes ----
    def _clean_section_head(m):
        cmd = m.group(1)
        inner = _clean_heading_text(m.group(2))
        return f"\\{cmd}{{{inner}}}"
    body = re.sub(
        r"\\(chapter|section|subsection|subsubsection)\{([^{}]*)\}",
        _clean_section_head,
        body,
    )

    # ---- consolidate boilerplate \section runs into synthetic wrapper
    #      sections so each chapter has ~7–8 substantive top-level sections ---
    body = consolidate_boilerplate(body, week)

    return body


def clean_and_wrap(week):
    raw_path = BOOK / "chapters" / f"{week}.tex.raw"
    out_path = BOOK / "chapters" / f"{week}.tex"
    if not raw_path.exists():
        return False
    body = raw_path.read_text(encoding="utf-8", errors="replace")
    body = transform_body(week, body)

    # Drop subsection blocks matching DROP patterns: their content is
    # timing-table lecture planning that doesn't belong in the book.
    # We walk the body and remove each matching \subsection{...} up to
    # the next \section or \subsection heading.
    def _drop_subsection_block(body):
        out = []
        lines = body.split("\n")
        i = 0
        while i < len(lines):
            line = lines[i]
            m = re.match(r"\\subsection\{([^{}]*)\}", line)
            if m and DROP_SECTION_RE.search(m.group(1)):
                # Skip until next \section or \subsection
                i += 1
                while i < len(lines):
                    nl = lines[i]
                    if re.match(r"\\(?:section|subsection)\{", nl):
                        break
                    i += 1
                continue
            out.append(line)
            i += 1
        return "\n".join(out)
    body = _drop_subsection_block(body)

    # Convert pandoc's {verbatim} + class="language-X" to lstlisting so
    # the colourful style applies. Pandoc emits
    #    \begin{Shaded}...\end{Shaded} for highlighted code when --highlight-style
    # is given, but with --no-highlight we get plain {verbatim}. Detect
    # the trailing "class" hint pandoc preserves via the language-X tag
    # in our HTML and wrap accordingly.
    # Strategy: pandoc for our pre>code emits \begin{verbatim}...; we
    # have no class signal there. So we re-detect from content.
    def _verbatim_to_lstlisting(m):
        code = m.group(1)
        lang = _detect_code_lang(code)
        if not lang:
            # leave as is; the tcolorbox wrapper styles it nicely
            return m.group(0)
        lang_key = {"python": "Python", "matlab": "Matlab", "bash": "bash"}.get(lang, "")
        if not lang_key:
            return m.group(0)
        return (f"\\begin{{lstlisting}}[language={lang_key}]\n"
                f"{code}\n"
                f"\\end{{lstlisting}}")
    body = re.sub(
        r"\\begin\{verbatim\}\n?(.*?)\n?\\end\{verbatim\}",
        _verbatim_to_lstlisting,
        body,
        flags=re.DOTALL,
    )

    # Post-process: shrink longtables so wide ones fit within text width.
    # Wrap each longtable as \begin{table}+\adjustbox+tabular so we can
    # scale it horizontally to fit the text block. Pandoc emits a column
    # spec like  {@{}llll@{}}  where the inner spec sits between two
    # balanced-brace groups — we match that explicitly to avoid the
    # greedy [^}] bug where only "@{" was captured before.
    def _tabletize_longtable(m):
        colspec_inner = m.group(1)      # e.g., "llll" (just the column letters)
        body_inner = m.group(2)
        return (
            "\\begin{table}[ht]\\footnotesize\\centering\n"
            "\\adjustbox{max width=\\textwidth,max totalheight=0.85\\textheight}{%\n"
            "\\begin{tabular}{" + colspec_inner + "}\n"
            + body_inner +
            "\\end{tabular}}\n"
            "\\end{table}\n"
        )
    body = re.sub(
        r"\\begin\{longtable\}\[[^\]]*\]\{@\{\}([^@{}]+)@\{\}\}(.*?)\\end\{longtable\}",
        _tabletize_longtable,
        body,
        flags=re.DOTALL,
    )

    # Strip longtable-specific markup that tabular doesn't accept
    body = re.sub(r"\\endfirsthead\s*", "", body)
    body = re.sub(r"\\endhead\s*", "", body)
    body = re.sub(r"\\endfoot\s*", "", body)
    body = re.sub(r"\\endlastfoot\s*", "", body)
    body = re.sub(r"\\noalign\{\}\s*", "", body)

    # Rewrite \underbrace{X}_{Y} to the cleaner \underset{Y}{\underline{X}}
    # using proper brace-balanced parsing (handles deep nesting like
    # \ddot{\mathbf{p}} and \substack{a \\ b} inside the expression or label).
    body = _replace_underbraces(body)

    # Append a "Simulation Results" section with every figure we have
    # for this week. Inserted BEFORE the trailing Summary/Wrap-Up blocks
    # if they exist, so the chapter flows: content -> simulation -> summary.
    sims_block = build_simulation_results_block(week)
    if sims_block:
        # Find the last Summary/Wrap-Up \section and insert before it
        summary_patterns = [
            r"\\section\{Summary and Next Steps\}",
            r"\\section\{Activities and Wrap-Up\}",
        ]
        inserted = False
        for pat in summary_patterns:
            m = list(re.finditer(pat, body))
            if m:
                idx = m[-1].start()
                body = body[:idx] + sims_block + body[idx:]
                inserted = True
                break
        if not inserted:
            body = body + "\n" + sims_block

    title = TITLES.get(week, week)
    # Cleaner chapter opener: no reserved image-space, no margin mini-TOC.
    # kaobook still numbers and formats the chapter heading, just without
    # the Tufte-style sidebar decorations that were leaving a gaping
    # blank at the top of every first page.
    header = (
        f"% ----- auto-generated from {week}/Week*.html via pandoc -----\n"
        f"\\chapter{{{title}}}\n"
        f"\\labch{{{week}}}\n\n"
    )
    out_path.write_text(header + body, encoding="utf-8")
    return True


def main():
    weeks = [f"week_{i:02d}" for i in range(1, 13)]
    for w in weeks:
        raw = BOOK / "chapters" / f"{w}.tex.raw"
        if not raw.exists():
            found = list((SRC_WS / w).glob("Week*.html"))
            if not found:
                print(f"  SKP  {w}: no HTML found")
                continue
            # Preprocess the HTML so pandoc recognises our custom
            # <div class="code-block">...</div> as code. Pandoc otherwise
            # emits the block as plain paragraph text, and MATLAB / shell
            # snippets become unreadable prose.
            html_raw = found[0].read_text(encoding="utf-8", errors="replace")
            html_pp = _preprocess_html_codeblocks(html_raw)
            tmp_html = raw.with_suffix(".html.tmp")
            tmp_html.write_text(html_pp, encoding="utf-8")
            subprocess.run([
                "pandoc",
                # Enable HTML reader extensions for MathJax-style math
                # delimiters.  Without these, pandoc escapes \(...\) and
                # \[...\] as literal text (\textbackslash(...)), producing
                # nonsense in the LaTeX output.
                "-f", "html+tex_math_single_backslash+tex_math_double_backslash",
                "-t", "latex",
                "--wrap=preserve",
                "--no-highlight",
                str(tmp_html), "-o", str(raw),
            ], check=True)
            tmp_html.unlink(missing_ok=True)
        # Copy sim figures BEFORE generating the chapter — clean_and_wrap
        # scans images/<week>/ to build the Simulation Results section.
        copy_figures_for_week(w)
        ok = clean_and_wrap(w)
        sz = (BOOK / "chapters" / f"{w}.tex").stat().st_size if ok else 0
        print(f"  {'ok ' if ok else 'SKP'}  {w}: {sz} bytes")


if __name__ == "__main__":
    main()
