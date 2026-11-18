#!/usr/bin/env python3
"""Build a static commit history page grouped by feature area.

Reads the git log of one or more branches, sorts every changed file into a
feature area (areas.json), parses the commit message body (description,
"Validation:" paragraph, trailers) and writes a self-contained index.html
plus history.json into the output directory.

Only the Python standard library is used, so the script runs in CI without
installing anything:

    python3 tools/commit_history/build_history.py --out _site
"""

from __future__ import annotations

import argparse
import fnmatch
import json
import os
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
RECORD_SEP = "\x1e"
FIELD_SEP = "\x1f"
TRAILER_RE = re.compile(r"^[A-Za-z][A-Za-z-]*: \S")
TRAILER_KEYS = {
    "signed-off-by",
    "co-authored-by",
    "claude-session",
    "reviewed-by",
    "acked-by",
    "tested-by",
    "change-id",
}
LIST_ITEM_RE = re.compile(r"^\s*(?:[-*•]|\d+[.)])\s+")

# Commit kind from the subject. Checked in this order; the first match wins.
KIND_RULES = [
    ("merge", re.compile(r"^merge\b", re.I)),
    (
        "chore",
        re.compile(r"\b(lint|ci|clang-format|ruff|dockerfile|docker-compose|format)\b", re.I),
    ),
    ("remove", re.compile(r"^(remove|removed|drop|dropped|delete|deleted)\b", re.I)),
    (
        "fix",
        re.compile(
            r"^(fix|fixed|fixes|resolve|resolved|correct|corrected|repair)\b|\bfailing\b|\bbug\b",
            re.I,
        ),
    ),
    (
        "feature",
        re.compile(
            r"^(add|added|adds|introduce|introduced|support|implement|implemented|create|"
            r"initial)\b",
            re.I,
        ),
    ),
    ("refactor", re.compile(r"^(refactor|refactored|rename|renamed|restructure|clean)\b", re.I)),
    ("docs", re.compile(r"^(document|documented|docs?)\b|\breadme\b", re.I)),
]


def git(*args: str, cwd: Path) -> str:
    result = subprocess.run(
        ["git", *args],
        cwd=cwd,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    return result.stdout


def load_areas(path: Path) -> tuple[list[dict], dict]:
    data = json.loads(path.read_text(encoding="utf-8"))
    return data["areas"], data["fallback"]


def area_of(path: str, areas: list[dict], fallback: dict) -> str:
    lowered = path.lower()
    for area in areas:
        for pattern in area["patterns"]:
            if fnmatch.fnmatchcase(lowered, pattern.lower()):
                return area["id"]
    return fallback["id"]


def resolve_rename(path: str) -> str:
    """Turn numstat rename notation ("a/{x => y}/z" or "x => y") into the new path."""
    match = re.match(r"^(.*)\{(.*) => (.*)\}(.*)$", path)
    if match:
        prefix, _old, new, suffix = match.groups()
        return re.sub(r"/+", "/", prefix + new + suffix).strip("/")
    if " => " in path:
        return path.split(" => ", 1)[1]
    return path


def parse_body(body: str) -> dict:
    """Split a message body into paragraphs, the Validation paragraph and trailers."""
    trailers = []
    paragraphs = []
    for block in re.split(r"\n\s*\n", body.strip()):
        lines = [line.rstrip() for line in block.splitlines() if line.strip()]
        if not lines:
            continue
        if all(
            TRAILER_RE.match(line) and line.split(":", 1)[0].lower() in TRAILER_KEYS
            for line in lines
        ):
            trailers.extend(lines)
            continue
        # Re-join hard-wrapped lines, keeping list items on their own line.
        merged: list[str] = []
        for line in lines:
            if merged and not LIST_ITEM_RE.match(line):
                merged[-1] = f"{merged[-1]} {line.strip()}"
            else:
                merged.append(line.strip())
        paragraphs.append(merged)

    validation = []
    description = []
    for paragraph in paragraphs:
        head = paragraph[0]
        if re.match(r"^(validation|verified|testing|tests?)\s*:", head, re.I):
            first = head.split(":", 1)[1].strip()
            validation.append([first, *paragraph[1:]] if first else paragraph[1:])
        else:
            description.append(paragraph)
    return {"description": description, "validation": validation, "trailers": trailers}


def classify(subject: str, files: list[dict], is_merge: bool) -> str:
    if is_merge:
        return "merge"
    if files and all(f["area"] == "docs" for f in files):
        return "docs"
    for kind, pattern in KIND_RULES:
        if pattern.search(subject):
            return kind
    return "change"


def read_branch(
    repo: Path, ref: str, areas: list[dict], fallback: dict, limit: int | None
) -> list[dict]:
    fmt = FIELD_SEP.join(["%H", "%P", "%an", "%aI", "%cI", "%s", "%b"])
    args = ["log", ref, "--numstat", "-M", f"--format={RECORD_SEP}{fmt}{RECORD_SEP}"]
    if limit:
        args.append(f"--max-count={limit}")
    raw = git(*args, cwd=repo)

    commits = []
    chunks = raw.split(RECORD_SEP)
    # chunks: ["", header, numstat, header, numstat, ...]
    for i in range(1, len(chunks) - 1, 2):
        header, stat = chunks[i], chunks[i + 1]
        sha, parents, author, adate, cdate, subject, body = header.split(FIELD_SEP, 6)
        files = []
        for line in stat.strip().splitlines():
            parts = line.split("\t")
            if len(parts) < 3:
                continue
            added, deleted, path = parts[0], parts[1], "\t".join(parts[2:])
            path = resolve_rename(path)
            binary = added == "-"
            files.append(
                {
                    "path": path,
                    "add": 0 if binary else int(added),
                    "del": 0 if binary else int(deleted),
                    "binary": binary,
                    "area": area_of(path, areas, fallback),
                }
            )
        files.sort(key=lambda f: (-(f["add"] + f["del"]), f["path"]))

        area_lines: dict[str, int] = {}
        area_files: dict[str, int] = {}
        for f in files:
            # A binary file still counts as one changed line so its area shows up.
            area_lines[f["area"]] = area_lines.get(f["area"], 0) + max(1, f["add"] + f["del"])
            area_files[f["area"]] = area_files.get(f["area"], 0) + 1
        commit_areas = sorted(
            ({"id": k, "lines": v, "files": area_files[k]} for k, v in area_lines.items()),
            key=lambda a: -a["lines"],
        )
        is_merge = len(parents.split()) > 1
        parsed = parse_body(body)
        commits.append(
            {
                "sha": sha,
                "short": sha[:7],
                "parents": parents.split(),
                "author": author,
                "date": adate,
                "committed": cdate,
                "subject": subject,
                "kind": classify(subject, files, is_merge),
                "description": parsed["description"],
                "validation": parsed["validation"],
                "areas": commit_areas,
                "files": files,
                "add": sum(f["add"] for f in files),
                "del": sum(f["del"] for f in files),
            }
        )
    return commits


def list_branches(repo: Path) -> list[str]:
    remote = git(
        "for-each-ref", "--format=%(refname:short)", "refs/remotes/origin", cwd=repo
    ).split()
    names = [r.split("/", 1)[1] for r in remote if r not in ("origin", "origin/HEAD")]
    if names:
        return names
    return git("for-each-ref", "--format=%(refname:short)", "refs/heads", cwd=repo).split()


def resolve_ref(repo: Path, branch: str) -> str | None:
    for candidate in (f"refs/remotes/origin/{branch}", f"refs/heads/{branch}", branch):
        try:
            git("rev-parse", "--verify", "--quiet", f"{candidate}^{{commit}}", cwd=repo)
            return candidate
        except subprocess.CalledProcessError:
            continue
    return None


def repo_url(repo: Path) -> str:
    try:
        url = git("remote", "get-url", "origin", cwd=repo).strip()
    except subprocess.CalledProcessError:
        return ""
    match = re.search(r"github\.com[:/]+([^/]+)/([^/]+?)(?:\.git)?/?$", url)
    return f"https://github.com/{match.group(1)}/{match.group(2)}" if match else ""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--repo", default=".", help="git repository (default: current dir)")
    parser.add_argument(
        "--branch",
        action="append",
        dest="branches",
        help="branch to include; repeat for several (default: every origin branch)",
    )
    parser.add_argument("--default-branch", help="branch shown first on the page")
    parser.add_argument("--out", default="_site", help="output directory (default: _site)")
    parser.add_argument("--repo-url", help="https://github.com/<owner>/<repo> for commit links")
    parser.add_argument("--title", help="page title (default: repository name)")
    parser.add_argument("--areas", default=str(HERE / "areas.json"), help="area rules file")
    parser.add_argument("--max-commits", type=int, help="newest N commits per branch")
    args = parser.parse_args()

    repo = Path(args.repo).resolve()
    areas, fallback = load_areas(Path(args.areas))
    url = (args.repo_url or repo_url(repo)).rstrip("/")
    title = args.title or (url.rsplit("/", 1)[-1] if url else repo.name)

    branch_names = args.branches or list_branches(repo)
    branches = []
    for name in branch_names:
        ref = resolve_ref(repo, name)
        if ref is None:
            print(f"skip {name}: no such branch", file=sys.stderr)
            continue
        commits = read_branch(repo, ref, areas, fallback, args.max_commits)
        if not commits:
            continue
        branches.append({"name": name, "head": commits[0]["sha"], "commits": commits})
        print(f"{name}: {len(commits)} commits", file=sys.stderr)
    if not branches:
        print("no commits found", file=sys.stderr)
        return 1

    first = args.default_branch or os.environ.get("GITHUB_REF_NAME")
    # The pushed (or requested) branch first, the rest by newest activity.
    branches.sort(key=lambda b: b["commits"][0]["date"], reverse=True)
    branches.sort(key=lambda b: b["name"] != first)

    used = {a["id"] for b in branches for c in b["commits"] for a in c["areas"]}
    area_meta = [{k: a[k] for k in ("id", "label", "description")} for a in [*areas, fallback]]
    area_meta = [a for a in area_meta if a["id"] in used]

    data = {
        "title": title,
        "repoUrl": url,
        "generated": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "areas": area_meta,
        "branches": branches,
    }

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(data, ensure_ascii=False, separators=(",", ":"))
    (out / "history.json").write_text(payload, encoding="utf-8")
    template = (HERE / "template.html").read_text(encoding="utf-8")
    # "</" inside a JSON string must not close the <script> element.
    inline = payload.replace("</", "<\\/")
    html = template.replace("/*__HISTORY_DATA__*/null", inline).replace(
        "__PAGE_TITLE__", _html_escape(f"{title} 커밋 히스토리")
    )
    (out / "index.html").write_text(html, encoding="utf-8")
    (out / ".nojekyll").write_text("", encoding="utf-8")
    print(f"wrote {out / 'index.html'}", file=sys.stderr)
    return 0


def _html_escape(text: str) -> str:
    return (
        text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace('"', "&quot;")
    )


if __name__ == "__main__":
    sys.exit(main())
