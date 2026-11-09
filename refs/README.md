# refs/ — P1 稿件参考文献（Zotero 同步用）

`Manuscript.md` 里那 42 条参考文献的机器可读版本。**唯一数据源是 `refs_data.py`**，
其余文件都由它生成，不要手改生成物。

## 文件

| 文件 | 作用 |
|---|---|
| `refs_data.py` | 42 条结构化记录（作者、题名、刊名、年、卷期页、DOI/arXiv） |
| `build_refs.py` | 校验 + 生成下面三个文件 |
| `refs.bib` | BibTeX，42 条，UTF-8，键名 `<姓><年><词>` |
| `refs.ris` | RIS，Zotero/EndNote 通用导入格式（备用） |
| `zotero_items.json` | 本地 API 写入载荷（Zotero Web API v3 条目格式） |
| `validate_items.py` | 按 Zotero 自带 `schema.json` 预校验载荷字段与 creatorType |
| `insert_zotero.py` | 写进本机 Zotero 并回读核对 |
| `zotero_result.json` | 最近一次写入结果（分类 key 与 42 个条目 key） |

## 重新生成

```bash
cd refs
python build_refs.py            # 校验 42 条 ↔ Manuscript.md，再写 bib/ris/items
python validate_items.py        # 按 Zotero schema 预校验
```

`build_refs.py` 的自检**不通过就退出、不产出文件**。它逐条比对稿件：

- 每位作者按顺序出现；
- 标题逐字一致（中文条目另含英文题名）；
- 刊名/会议名一致；
- 年份、卷期写法（`*vol*(issue)`）、页码各片段；
- DOI **精确相等**（前缀相同也算错）、arXiv 编号精确相等。

改参考文献时只改 `refs_data.py`，再跑一次。若稿件的参考文献行改了而这里没改，脚本会报错。

## 写入 Zotero

已写入本机 Zotero 的分类 **`果园可见性_文献库`**（42 条）。collection key 与本机脚本不入库，见 `.gitignore`。

重跑需要两个前提：

1. Zotero 设置 → 高级 → 勾选“允许本机其他程序与 Zotero 通信”（`extensions.zotero.httpServer.localAPI.enabled`，本机已置为 true）；
2. 拿到本地 API 密钥：

```bash
curl -X POST http://127.0.0.1:23119/api/local/authorize \
     -H "Content-Type: application/json" \
     -H "Zotero-Server-ID: <GET /api 响应头里的值>" \
     -d '{"appName":"WorkBuddy 文献整理"}'
```

弹窗里选 **Always Allow** 才是长期密钥；选 Allow 得到的一次性密钥**只够一次写入**，
且任何一次被受理的写请求（哪怕 400）都会把它消耗掉。

```bash
python insert_zotero.py --key <密钥> --reuse <collection-key>   # 加进已有分类
python insert_zotero.py --key <密钥> --name 新分类名       # 新建分类再写入
```

## 目前状态（2026-11-07）

- 条目类型：25 篇 journalArticle、6 篇 conferencePaper、11 篇 preprint；
- 30 条带 DOI，12 条无 DOI（1 篇期刊未给 DOI + 11 篇 arXiv）；
- 中文条目《空地协同系统关键技术发展现状综述》保留中文题名，英文题名放 `extra`；
- 回读核对：条数、标题集合、DOI 集合三项与本地一致。

## PDF 附件（2026-11-07 已挂）

`map_pdfs.py` 用 **PDF 正文里的 DOI / arXiv 号**定位条目，而不是靠文件名——文件名是缩写式的，
按题名匹配会把 15 条对错。结果：42 个 PDF 里 37 条按 DOI/arXiv 精确命中，3 条按题名兜底，
2 条人工绑定（`45_UGV_UAV_3DMapping_Orchard` → [12]，`24_CarrilloArce2013` → [14]）。

`attach_pdfs.py` 走本地 API 三步上传（建附件条目 → 申请 uploadKey → multipart 传字节 → 注册），
`verify_attachments.py` 回读核验。

当前：**39 / 42 条有 PDF**，附件占 **186.3 MB**（存储副本模式，会计入 Zotero 同步配额）。

缺 PDF 的 3 条：

| # | 文献 | 说明 |
|---|---|---|
| 18 | Julier & Uhlmann 1997, ACC | IEEE 付费墙 |
| 39 | Wang et al. 2021, IROS | 未找到公开版本 |
| 42 | Garrido-Jurado et al. 2014, *Pattern Recognition* | ArUco 原文，付费墙 |

`03_文献全文/` 里另有 3 份不属于这 42 条的 PDF，未挂：
`04_` 是 Go 的博士论文（非所引的 [37] 期刊/会议版）、`21_` 是 [10] 的预印本版（与已挂的 `22_` 同文）、
`48_` 是 Karcı 2025 *Drones*（未被引用）。

## 附件相关的接口注意

- 核验文件路径用 `/api/users/0/items/<key>/file/view/url`，`/file/url` 会返回 `No endpoint found`；
- 授权阶段必须带 `If-None-Match: *`，否则 428；`mtime` 必须是**毫秒**；
- 只支持 `imported_file` / `imported_url`，链接文件（linked_file）不能用接口建。
