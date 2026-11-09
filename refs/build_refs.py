# -*- coding: utf-8 -*-
"""由 refs_data.py 生成 refs.bib / refs.ris / zotero_items.json，并逐条核对 Manuscript.md。

用法：python build_refs.py [--md <稿件路径>]
自检不过直接 SystemExit，不产出文件（避免半成品被当成真数据）。
"""
from __future__ import annotations

import argparse
import io
import json
import os
import re
import sys

from refs_data import REFS

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MD = os.path.join(HERE, "..", "simulation", "Manuscript.md")


# ---------------------------------------------------------------- 读取稿件
def load_ref_lines(md_path):
    with io.open(md_path, encoding="utf-8", newline="") as f:
        txt = f.read()
    body = txt[txt.index("## References"):]
    lines = {}
    for raw in body.splitlines():
        m = re.match(r"^(\d{1,2})\.\s+(.*\S)\s*$", raw.strip())
        if m:
            lines[int(m.group(1))] = m.group(2)
    return lines


# ---------------------------------------------------------------- 逐条核对
def norm(s):
    return re.sub(r"\s+", " ", s).strip()


def check(refs, lines):
    problems = []
    for r in refs:
        n = r["n"]
        line = lines.get(n)
        if line is None:
            problems.append("[%d] 稿件里找不到这一条" % n)
            continue
        text = norm(line)
        # 作者：按顺序逐个出现
        pos = 0
        for au in r["a"]:
            i = text.find(au, pos)
            if i < 0:
                problems.append("[%d] 作者 %r 未按顺序出现" % (n, au))
                break
            pos = i + len(au)
        # 标题
        if norm(r["t"]) not in text:
            problems.append("[%d] 标题与稿件不一致：%r" % (n, norm(r["t"])[:50]))
        if r.get("t_en") and norm(r["t_en"]) not in text:
            problems.append("[%d] 英文题名未出现" % n)
        # 刊名 / 会议名（预印本无刊名，改查 arXiv 前缀）
        if r["type"] == "preprint":
            if "arXiv:" not in text:
                problems.append("[%d] 预印本行没有 arXiv:" % n)
        elif norm(r["v"]) not in text:
            problems.append("[%d] 刊名/会议名不一致：%r" % (n, r["v"]))
        # 年份
        if str(r["y"]) not in text:
            problems.append("[%d] 年份 %s 未出现" % (n, r["y"]))
        # 卷 / 期
        if r.get("vol"):
            if r["type"] == "conference":
                if "Volume %s" % r["vol"] not in text:
                    problems.append("[%d] 会议卷次未出现" % n)
            else:
                pat = "*%s*" % r["vol"]
                if r.get("no"):
                    pat += "(%s)" % r["no"]
                if pat not in text:
                    problems.append("[%d] 卷期写法与稿件不符：期望 %r" % (n, pat))
        # 页码：区间两端都要在
        if r.get("p"):
            for num in re.findall(r"\d+", r["p"]):
                if num not in text:
                    problems.append("[%d] 页码片段 %s 未出现" % (n, num))
        # DOI / arXiv（DOI 必须与稿件行逐字相同，前缀相同也算错）
        m_doi = re.search(r"https://doi\.org/(\S+?)[.\s]*$", text)
        got_doi = m_doi.group(1).rstrip(".") if m_doi else None
        if r.get("doi"):
            if got_doi != r["doi"]:
                problems.append("[%d] DOI 不一致：记录 %r / 稿件 %r" % (n, r["doi"], got_doi))
            if text.count("doi.org") != 1:
                problems.append("[%d] 该行 DOI 出现 %d 次" % (n, text.count("doi.org")))
        else:
            if "doi.org" in text:
                problems.append("[%d] 记录没有 DOI，但稿件行有 DOI" % n)
        if r.get("arx"):
            m_arx = re.search(r"arXiv:(\S+?),", text)
            got_arx = m_arx.group(1) if m_arx else None
            if got_arx != r["arx"]:
                problems.append("[%d] arXiv 不一致：记录 %r / 稿件 %r" % (n, r["arx"], got_arx))
            if not r.get("doi") and text.count("arXiv:") != 1:
                problems.append("[%d] 该行 arXiv 出现 %d 次" % (n, text.count("arXiv:")))
    return problems


# ---------------------------------------------------------------- BibTeX
def tex_escape(s):
    out = s.replace("&", r"\&").replace("%", r"\%").replace("#", r"\#")
    out = out.replace("_", r"\_")
    return out


def build_bib(refs):
    chunks = [
        "% P1 orchard visibility — 42 references, generated from refs_data.py",
        "% Encoding: UTF-8. Keys are <family><year><word>.",
        "",
    ]
    for r in refs:
        authors = " and ".join(r["a"])
        title = tex_escape(r["t"])
        if r.get("t_en"):
            title = "%s [%s]" % (title, tex_escape(r["t_en"]))
        if r["type"] == "article":
            kind = "article"
        elif r["type"] == "conference":
            kind = "inproceedings"
        else:
            kind = "misc"
        fields = [("author", tex_escape(authors)), ("title", title), ("year", str(r["y"]))]
        if r["type"] == "article":
            fields.append(("journal", tex_escape(r["v"])))
        elif r["type"] == "conference":
            fields.append(("booktitle", tex_escape(r["v"])))
            if r.get("place"):
                fields.append(("address", r["place"]))
            if r.get("date"):
                fields.append(("note", r["date"]))
        if r.get("vol"):
            fields.append(("volume", r["vol"]))
        if r.get("no"):
            fields.append(("number", r["no"]))
        if r.get("p"):
            fields.append(("pages", r["p"].replace("–", "--")))
        if r.get("doi"):
            fields.append(("doi", r["doi"]))
        if r.get("arx"):
            fields.append(("eprint", r["arx"]))
            fields.append(("archivePrefix", "arXiv"))
            fields.append(("note", "arXiv:%s" % r["arx"]))
        if r.get("lang") == "zh":
            fields.append(("langid", "chinese"))
        width = max(len(k) for k, _ in fields)
        body = ",\n".join("  %-*s = {%s}" % (width, k, v) for k, v in fields)
        chunks.append("@%s{%s,\n%s\n}\n" % (kind, r["key"], body))
    return "\n".join(chunks)


# ---------------------------------------------------------------- RIS
def build_ris(refs):
    ty = {"article": "JOUR", "conference": "CPAPER", "preprint": "UNPB"}
    out = []
    for r in refs:
        out.append("TY  - %s" % ty[r["type"]])
        for au in r["a"]:
            out.append("AU  - %s" % au)
        title = r["t"]
        if r.get("t_en"):
            title = "%s [%s]" % (title, r["t_en"])
        out.append("TI  - %s" % title)
        if r["type"] == "article":
            out.append("JO  - %s" % r["v"])
        elif r["type"] == "conference":
            out.append("T2  - %s" % r["v"])
            if r.get("place"):
                out.append("CY  - %s" % r["place"])
        else:
            out.append("T2  - arXiv")
            out.append("M3  - arXiv:%s" % r["arx"])
        out.append("PY  - %s" % r["y"])
        if r.get("vol"):
            out.append("VL  - %s" % r["vol"])
        if r.get("no"):
            out.append("IS  - %s" % r["no"])
        if r.get("p"):
            nums = re.findall(r"\d+", r["p"])
            if len(nums) == 2:
                out.append("SP  - %s" % nums[0])
                out.append("EP  - %s" % nums[1])
            else:
                out.append("SP  - %s" % r["p"])
        if r.get("doi"):
            out.append("DO  - %s" % r["doi"])
        if r.get("arx"):
            out.append("UR  - https://arxiv.org/abs/%s" % r["arx"])
        out.append("ER  - ")
        out.append("")
    return "\n".join(out)


# ---------------------------------------------------------------- Zotero
def split_name(name):
    if "," in name:
        last, first = name.split(",", 1)
        return {"creatorType": "author", "lastName": last.strip(), "firstName": first.strip()}
    return {"creatorType": "author", "name": name.strip()}


def build_items(refs):
    items = []
    for r in refs:
        it = {"creators": [split_name(a) for a in r["a"]],
              "title": r["t"],
              "date": str(r["y"])}
        if r["type"] == "article":
            it["itemType"] = "journalArticle"
            it["publicationTitle"] = r["v"]
            if r.get("vol"):
                it["volume"] = r["vol"]
            if r.get("no"):
                it["issue"] = r["no"]
            if r.get("p"):
                it["pages"] = r["p"]
            if r.get("doi"):
                it["DOI"] = r["doi"]
        elif r["type"] == "conference":
            it["itemType"] = "conferencePaper"
            it["proceedingsTitle"] = r["v"]
            it["conferenceName"] = r["v"]
            if r.get("place"):
                it["place"] = r["place"]
            if r.get("vol"):
                it["volume"] = r["vol"]
            if r.get("p"):
                it["pages"] = r["p"]
            if r.get("doi"):
                it["DOI"] = r["doi"]
        else:
            it["itemType"] = "preprint"
            it["repository"] = "arXiv"
            it["archiveID"] = "arXiv:%s" % r["arx"]
            it["url"] = "https://arxiv.org/abs/%s" % r["arx"]
        if r.get("t_en"):
            it["language"] = "zh-CN"
            it["extra"] = "English title: %s" % r["t_en"]
        items.append(it)
    return items


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", default=DEFAULT_MD)
    args = ap.parse_args(argv)

    lines = load_ref_lines(args.md)
    if len(lines) != 42:
        raise SystemExit("参考文献条数 %d，期望 42" % len(lines))
    problems = check(REFS, lines)
    if problems:
        print("自检未通过，共 %d 项：" % len(problems))
        for p in problems:
            print("  -", p)
        raise SystemExit(1)
    print("自检通过：42 条与 Manuscript.md 逐字段一致（作者顺序、标题、刊名、年、卷期、页码、DOI/arXiv）")

    bib = build_bib(REFS)
    ris = build_ris(REFS)
    items = build_items(REFS)

    with io.open(os.path.join(HERE, "refs.bib"), "w", encoding="utf-8", newline="\n") as f:
        f.write(bib)
    with io.open(os.path.join(HERE, "refs.ris"), "w", encoding="utf-8", newline="\n") as f:
        f.write(ris)
    with io.open(os.path.join(HERE, "zotero_items.json"), "w", encoding="utf-8") as f:
        json.dump(items, f, ensure_ascii=False, indent=1)

    print("已生成 refs.bib（%d 条）、refs.ris（%d 条）、zotero_items.json（%d 条）"
          % (bib.count("\n@") + bib.startswith("@"), ris.count("TY  - "), len(items)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
