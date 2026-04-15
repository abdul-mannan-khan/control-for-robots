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
SRC_WS = Path("/home/it-services/auto_control_ws")

TITLES = {
    "week_01": "Manipulator Dynamics and Computed-Torque Control (Week 1)",
    "week_02": "Model Predictive Control --- Introduction (Week 2)",
    "week_03": "Model Predictive Control --- Design and Simulation (Week 3)",
    "week_04": "Adaptive Control (Week 4)",
    "week_05": "Robust Control (Week 5)",
    "week_06": "Backstepping Control (Week 6)",
    "week_07": "Sliding Mode Control (Week 7)",
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
    r"|Recommended\s+Approach"
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

    # First pass: classify every section
    entries = []   # list of dicts: {kind, label_or_title, body, subs}
    i = 0
    while i < len(headers):
        title = re.match(r"\\section\{([^{}]*)\}", headers[i]).group(1)
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
    dst = BOOK / "images" / week
    dst.mkdir(parents=True, exist_ok=True)
    sim_fig_dir = SRC_WS / week / "sims" / "figures"
    if sim_fig_dir.exists():
        for p in sim_fig_dir.glob("*.png"):
            shutil.copy(p, dst / p.name)


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

    title = TITLES.get(week, week)
    header = (
        f"% ----- auto-generated from {week}/Week*.html via pandoc -----\n"
        f"\\setchapterimage[6cm]{{}}\n"
        f"\\setchapterpreamble[u]{{\\margintoc}}\n"
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
            subprocess.run([
                "pandoc", "-f", "html", "-t", "latex",
                "--wrap=preserve", "--mathjax", "--no-highlight",
                str(found[0]), "-o", str(raw),
            ], check=True)
        ok = clean_and_wrap(w)
        copy_figures_for_week(w)
        sz = (BOOK / "chapters" / f"{w}.tex").stat().st_size if ok else 0
        print(f"  {'ok ' if ok else 'SKP'}  {w}: {sz} bytes")


if __name__ == "__main__":
    main()
