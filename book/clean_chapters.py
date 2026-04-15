#!/usr/bin/env python3
"""Clean up pandoc's raw LaTeX output and wrap each as a kaobook chapter.

- Strips boilerplate (title, style blocks) that pandoc passes through.
- Rewrites <img src=...> converted by pandoc to use our images/ layout.
- Drops problematic pandoc macros (\tightlist, \@ifdefined) that aren't
  needed under kaobook.
- Adds a \chapter{} header and strips old section numbering so kaobook's
  numbering takes over.
"""
import re
import shutil
from pathlib import Path

BOOK = Path("/home/it-services/control-for-robots/book")
SRC_WS = Path("/home/it-services/auto_control_ws")

TITLES = {
    "week_01": "Manipulator Dynamics and Computed-Torque Control",
    "week_02": "Model Predictive Control --- Introduction",
    "week_03": "Model Predictive Control --- Design and Simulation",
    "week_04": "Adaptive Control",
    "week_05": "Robust Control",
    "week_06": "Backstepping Control",
    "week_07": "Sliding Mode Control",
    "week_08": "Nonlinear Control Optimization for Robotic Systems",
    "week_09": "ROS\\,2 Control Implementation",
    "week_10": "ROS\\,2 Quadrotor Control",
    "week_11": "ROS\\,2 Navigation Control",
    "week_12": "Controller Comparison and Industry Applications",
}

# Strip pandoc boilerplate lines that appear at top
STRIP_LINES = (
    r"^\\tightlist",
    r"^\\providecommand",
    r"^\\IfFileExists",
    r"^\\addbibresource",
    r"^\\makeatletter",
    r"^\\makeatother",
)

def copy_figures_for_week(week):
    """Copy output figures used in the week into book/images/<week>/."""
    dst = BOOK / "images" / week
    dst.mkdir(parents=True, exist_ok=True)
    # week_08/sims/figures/*.png
    for p in (SRC_WS / week / "sims" / "figures").glob("*.png"):
        shutil.copy(p, dst / p.name)
    # week_09/sims/figures/*.png
    for p in (SRC_WS / week / "sims" / "figures").glob("*.png"):
        shutil.copy(p, dst / p.name)

def clean_and_wrap(week):
    raw_path = BOOK / "chapters" / f"{week}.tex.raw"
    out_path = BOOK / "chapters" / f"{week}.tex"
    if not raw_path.exists():
        return False
    body = raw_path.read_text(encoding="utf-8", errors="replace")

    # -- Fix image paths: pandoc produces \includegraphics{sims/figures/foo.png}
    #    We want them pointed to book/images/<week>/foo.png
    def _rewrite_img(m):
        path = m.group(1).strip()
        basename = path.split("/")[-1]
        return f"\\includegraphics[width=0.85\\textwidth]{{images/{week}/{basename}}}"
    body = re.sub(r"\\includegraphics(?:\[[^\]]*\])?\{([^}]+\.png)\}", _rewrite_img, body)

    # -- Strip broken pandoc hyperref targets
    body = re.sub(r"\\hypertarget\{[^}]*\}\{%?\s*\n?", "", body)
    body = re.sub(r"\\label\{[^}]*\}(?=\n|$)", "", body, count=200)

    # -- Convert \section to \section*, \subsection to \subsection* to avoid
    #    polluting kaobook's TOC (we want each week as a single chapter with
    #    unnumbered inner structure); or just let kaobook number them.
    #    Keep sections numbered — kaobook chapter numbers them as X.Y.

    # -- Remove {}s left by pandoc after anchors
    body = re.sub(r"^\}\s*$", "", body, flags=re.MULTILINE)

    # -- Drop stray \begin{verbatim}...\end{verbatim} with only whitespace
    body = re.sub(r"\\begin\{verbatim\}\s*\\end\{verbatim\}", "", body)

    # -- Cap table widths
    body = body.replace("\\begin{longtable}", "\\begin{longtable}")

    # Drop nothing else — preserve everything else.

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
        ok = clean_and_wrap(w)
        copy_figures_for_week(w)
        print(f"  {'ok ' if ok else 'SKP'}  {w}: {(BOOK/'chapters'/f'{w}.tex').stat().st_size if ok else 0} bytes")

    # Clean up .raw files
    for p in (BOOK / "chapters").glob("*.raw"):
        p.unlink()

if __name__ == "__main__":
    main()
