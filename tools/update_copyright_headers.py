#!/usr/bin/env python3
"""
Check and update djnn Smala source headers.

Examples:
  tools/update_copyright_headers.py --check
  tools/update_copyright_headers.py --write
  tools/update_copyright_headers.py --write --contributor "Name <email>"
  tools/update_copyright_headers.py --write --insert-missing --contributor "Name <email>"

The script scans src, cookbook and src_lib for source files, excluding any path
component named "ext" or "gen". By default it only rewrites recognized djnn
Smala headers. Missing headers are listed; use --insert-missing with --write to
add them.

For recognized and inserted headers, missing Git authors with an @enac.fr
address are appended to the contributors list. --contributor can add one more
explicit @enac.fr entry. If no contributor remains available, a default ENAC
contributors list is used.
"""

from __future__ import annotations

import argparse
import datetime as _datetime
import re
import subprocess
import sys
from pathlib import Path


SCAN_DIRS = ("src", "cookbook", "src_lib")
SOURCE_SUFFIXES = {".cpp", ".h", ".hpp", ".sma", ".l", ".y"}
HEADER_RE = re.compile(r"\A(?P<prefix>\ufeff?\s*)(?P<header>/\*.*?\*/\s*)", re.DOTALL)
YEAR_RE = re.compile(r"Ecole Nationale de l'Aviation Civile, France \((?P<years>\d{4}(?:-\d{4})?)\)")
CONTRIBUTOR_RE = re.compile(r"^\s*\*\s+(.+?)\s*$")
EMAIL_RE = re.compile(r"<([^<>@\s]+@[^<>\s]+)>")
DEFAULT_CONTRIBUTORS = [
    "Mathieu Poirier <mathieu.poirier@enac.fr>",
    "Stephane Conversy <stephane.conversy@enac.fr>",
]


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def is_excluded_path(path: Path) -> bool:
    return bool({"ext", "gen"} & set(path.parts))


def source_files(root: Path) -> list[Path]:
    files: list[Path] = []
    for dirname in SCAN_DIRS:
        base = root / dirname
        if not base.exists():
            continue
        for path in base.rglob("*"):
            if path.suffix not in SOURCE_SUFFIXES:
                continue
            rel = path.relative_to(root)
            if is_excluded_path(rel):
                continue
            files.append(path)
    return sorted(files)


def git_years(root: Path, path: Path) -> tuple[int, int]:
    rel = path.relative_to(root)
    try:
        out = subprocess.check_output(
            ["git", "log", "--follow", "--format=%ad", "--date=format:%Y", "--", str(rel)],
            cwd=root,
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except subprocess.CalledProcessError:
        out = ""
    years = [int(line) for line in out.splitlines() if line.strip().isdigit()]
    if not years:
        year = _datetime.date.today().year
        return year, year
    return min(years), max(years)


def git_contributors(root: Path, path: Path) -> list[str]:
    rel = path.relative_to(root)
    try:
        out = subprocess.check_output(
            ["git", "log", "--follow", "--reverse", "--format=%an <%ae>", "--", str(rel)],
            cwd=root,
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except subprocess.CalledProcessError:
        return []
    contributors: list[str] = []
    for line in out.splitlines():
        contributor = line.strip()
        if contributor.endswith(" <>"):
            continue
        if contributor and contributor not in contributors:
            contributors.append(contributor)
    return contributors


def contributor_email(contributor: str) -> str | None:
    match = EMAIL_RE.search(contributor)
    if not match:
        return None
    return match.group(1).lower()


def is_enac_contributor(contributor: str) -> bool:
    email = contributor_email(contributor)
    return email is not None and email.endswith("@enac.fr")


def has_contributor(contributors: list[str], contributor: str) -> bool:
    email = contributor_email(contributor)
    if email is not None:
        return any(contributor_email(existing) == email for existing in contributors)
    normalized = " ".join(contributor.lower().split())
    return any(" ".join(existing.lower().split()) == normalized for existing in contributors)


def parse_existing_header(text: str) -> tuple[str, str, int] | None:
    match = HEADER_RE.match(text)
    if not match:
        return None
    return match.group("prefix"), match.group("header"), match.end()


def is_smala_header(header: str) -> bool:
    return "djnn Smala compiler" in header and "The copyright holders for the contents of this file are:" in header


def parse_start_year(header: str) -> int | None:
    match = YEAR_RE.search(header)
    if not match:
        return None
    return int(match.group("years").split("-", 1)[0])


def parse_contributors(header: str) -> list[str]:
    contributors: list[str] = []
    in_contributors = False
    for line in header.splitlines():
        if re.match(r"^\s*\*\s+Contributors:\s*$", line):
            in_contributors = True
            continue
        if not in_contributors:
            continue
        if line.strip() in {"*", "*/"}:
            if contributors:
                break
            continue
        match = CONTRIBUTOR_RE.match(line)
        if match:
            contributors.append(match.group(1))
    return contributors


def format_years(start_year: int, end_year: int) -> str:
    if start_year == end_year:
        return str(start_year)
    return f"{start_year}-{end_year}"


def build_header(start_year: int, end_year: int, contributors: list[str]) -> str:
    lines = [
        "/*",
        "*  djnn Smala compiler",
        "*",
        "*  The copyright holders for the contents of this file are:",
        f"*    Ecole Nationale de l'Aviation Civile, France ({format_years(start_year, end_year)})",
        "*  See file \"license.terms\" for the rights and conditions",
        "*  defined by copyright holders.",
        "*",
        "*",
        "*  Contributors:",
    ]
    for contributor in contributors:
        lines.append(f"*     {contributor}")
    lines.extend(["*", "*/", ""])
    return "\n".join(lines)


def add_contributor(contributors: list[str], contributor: str | None, require_enac: bool = True) -> None:
    if not contributor:
        return
    if require_enac and not is_enac_contributor(contributor):
        return
    if not has_contributor(contributors, contributor):
        contributors.append(contributor)


def add_contributors(contributors: list[str], new_contributors: list[str], require_enac: bool = True) -> None:
    for contributor in new_contributors:
        add_contributor(contributors, contributor, require_enac=require_enac)


def ensure_default_contributors(contributors: list[str]) -> None:
    if contributors:
        return
    add_contributors(contributors, DEFAULT_CONTRIBUTORS, require_enac=True)


def update_text(root: Path, path: Path, contributor: str | None, insert_missing: bool) -> tuple[str, str]:
    text = path.read_text()
    parsed = parse_existing_header(text)
    git_start, git_end = git_years(root, path)
    missing_git_contributors = git_contributors(root, path)

    if parsed is None:
        if not insert_missing:
            return "missing", text
        contributors: list[str] = []
        add_contributors(contributors, missing_git_contributors)
        add_contributor(contributors, contributor)
        ensure_default_contributors(contributors)
        return "updated", build_header(git_start, git_end, contributors) + text

    prefix, header, end = parsed
    if not is_smala_header(header):
        return "atypical", text

    start_year = parse_start_year(header) or git_start
    contributors = parse_contributors(header)
    add_contributors(contributors, missing_git_contributors)
    add_contributor(contributors, contributor)
    ensure_default_contributors(contributors)

    new_header = build_header(start_year, git_end, contributors)
    new_text = prefix + new_header + text[end:]
    return ("updated" if new_text != text else "ok"), new_text


def progress(message: str, enabled: bool) -> None:
    if not enabled:
        return
    width = 100
    clipped = message[: width - 4]
    sys.stderr.write("\r" + clipped.ljust(width))
    sys.stderr.flush()


def finish_progress(enabled: bool) -> None:
    if not enabled:
        return
    sys.stderr.write("\r" + " " * 100 + "\r")
    sys.stderr.flush()


def print_group(title: str, paths: list[Path], root: Path) -> None:
    if not paths:
        return
    print(f"\n{title}:")
    for path in paths:
        print(f"  {path.relative_to(root)}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Check or update djnn Smala source headers.")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true", help="report files that would change")
    mode.add_argument("--write", action="store_true", help="rewrite recognized headers")
    parser.add_argument("--insert-missing", action="store_true", help="with --write, insert missing djnn headers")
    parser.add_argument("--contributor", help='contributor to add, e.g. "Name <email>"')
    parser.add_argument("--no-progress", action="store_true", help="disable one-line progress output")
    args = parser.parse_args()

    if args.insert_missing and not args.write:
        parser.error("--insert-missing requires --write")

    root = repo_root()
    would_update: list[Path] = []
    updated: list[Path] = []
    missing: list[Path] = []
    atypical: list[Path] = []

    files = source_files(root)
    show_progress = not args.no_progress
    for index, path in enumerate(files, start=1):
        rel = path.relative_to(root)
        progress(f"[{index}/{len(files)}] checking {rel}", show_progress)
        status, new_text = update_text(root, path, args.contributor, args.insert_missing)
        if status == "missing":
            missing.append(path)
        elif status == "atypical":
            atypical.append(path)
        elif status == "updated":
            if args.write:
                path.write_text(new_text)
                updated.append(path)
            else:
                would_update.append(path)
    finish_progress(show_progress)

    print_group("Modified" if args.write else "Would update", updated if args.write else would_update, root)
    print_group("Missing djnn Smala header", missing, root)
    print_group("Ignored atypical or third-party header", atypical, root)

    if not any([would_update, updated, missing, atypical]):
        print("All checked headers are up to date.")

    if args.check and (would_update or missing or atypical):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
