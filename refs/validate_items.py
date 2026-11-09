# -*- coding: utf-8 -*-
"""按 Zotero 自带 schema.json 预校验 zotero_items.json。

目的：本地 API 的一次性密钥只够写一次，发出去前必须确认
      条目类型、字段名、creatorType 都在 Zotero 认可范围内。
"""
from __future__ import annotations

import io
import json
import os
import sys
import zipfile

OMNI = r"C:/Program Files/Zotero/app/omni.ja"
SCHEMA = "resource/schema/global/schema.json"
HERE = os.path.dirname(os.path.abspath(__file__))

BASE_FIELDS = {"itemType", "creators", "collections", "tags", "relations", "key",
               "version", "dateAdded", "dateModified", "parentItem", "deleted", "inPublications"}
BASE_CREATORS = {"creatorType", "firstName", "lastName", "name"}


def load_schema():
    with zipfile.ZipFile(OMNI) as z:
        return json.loads(z.read(SCHEMA).decode("utf-8"))


def main():
    schema = load_schema()
    types = {}
    for t in schema["itemTypes"]:
        types[t["itemType"]] = {
            "fields": {f["field"] for f in t["fields"]},
            "creators": {c["creatorType"] for c in t["creatorTypes"]},
        }
    with io.open(os.path.join(HERE, "zotero_items.json"), encoding="utf-8") as f:
        items = json.load(f)

    bad = []
    stat = {}
    for i, it in enumerate(items, 1):
        ty = it.get("itemType")
        if ty not in types:
            bad.append("[%d] 未知条目类型 %r" % (i, ty))
            continue
        stat[ty] = stat.get(ty, 0) + 1
        allowed = types[ty]["fields"] | BASE_FIELDS
        for k in it:
            if k not in allowed:
                bad.append("[%d] %s 不允许字段 %r" % (i, ty, k))
        for c in it.get("creators", []):
            if c["creatorType"] not in types[ty]["creators"]:
                bad.append("[%d] %s 不允许 creatorType %r" % (i, ty, c["creatorType"]))
            for k in c:
                if k not in BASE_CREATORS:
                    bad.append("[%d] creator 多余键 %r" % (i, k))
            has_full = "firstName" in c and "lastName" in c
            has_single = "name" in c
            if has_full == has_single:
                bad.append("[%d] creator 姓名形式不合法：%r" % (i, c))
        if not it.get("title"):
            bad.append("[%d] 缺标题" % i)
        if not it.get("creators"):
            bad.append("[%d] 缺作者" % i)

    print("条目类型分布：", stat)
    if bad:
        print("预校验失败 %d 项：" % len(bad))
        for b in bad[:30]:
            print("  -", b)
        return 1
    print("预校验通过：%d 条，字段与 creatorType 均在 Zotero schema 内" % len(items))
    return 0


if __name__ == "__main__":
    sys.exit(main())
