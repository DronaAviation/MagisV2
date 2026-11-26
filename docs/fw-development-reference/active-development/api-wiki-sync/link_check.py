"""Check every link in a GitHub wiki folder ( api-wiki-sync, task 15 ).

    python -I link_check.py <wiki-folder>

For each Markdown link or <img src> outside code blocks:
  - a page link ( `Page`, `Page#anchor`, `Page.md` ) must name an existing `Page.md`;
  - an anchor ( `#x` or `Page#x` ) must match a heading of that page, slugged as GitHub does;
  - a relative file link ( `images/x.png` ) must exist;
  - http(s) and mailto links are not fetched.
Also reports page links written with `.md`, which GitHub wikis do not need.
Exit code 1 if any dead link is found.
"""
import os
import re
import sys

LINK = re.compile(r'(?<!!)\[[^\]]*\]\(([^)\s]+)\)|!\[[^\]]*\]\(([^)\s]+)\)|<img[^>]*\bsrc="([^"]+)"')
HEAD = re.compile(r'^(#{1,6})\s+(.*?)\s*#*\s*$')


def slug(text):
    text = re.sub(r'`', '', text).strip().lower()
    text = re.sub(r'[^\w\- ]', '', text)
    return text.replace(' ', '-')


def strip_code(lines):
    out, fence = [], False
    for ln in lines:
        if ln.lstrip().startswith('```'):
            fence = not fence
            out.append('')
            continue
        out.append('' if fence else re.sub(r'`[^`]*`', '', ln))
    return out


def anchors(path):
    seen, result = {}, set()
    for ln in _raw_noncode(path):
        m = HEAD.match(ln)
        if m:
            s = slug(m.group(2))
            n = seen.get(s, 0)
            result.add(s if n == 0 else '%s-%d' % (s, n))
            seen[s] = n + 1
    return result


def _raw_noncode(path):
    with open(path, encoding='utf-8') as fh:
        fence = False
        for ln in fh.read().splitlines():
            if ln.lstrip().startswith('```'):
                fence = not fence
                continue
            if not fence:
                yield ln


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else '.'
    pages = {f[:-3]: os.path.join(root, f) for f in os.listdir(root) if f.endswith('.md')}
    cache, dead, mdstyle, total = {}, [], [], 0
    for name, path in sorted(pages.items()):
        with open(path, encoding='utf-8') as fh:
            lines = strip_code(fh.read().splitlines())
        for no, ln in enumerate(lines, 1):
            for m in LINK.finditer(ln):
                target = m.group(1) or m.group(2) or m.group(3)
                total += 1
                if re.match(r'(https?|mailto):', target):
                    continue
                page, _, anchor = target.partition('#')
                if page.endswith('.md') and '/' not in page:
                    mdstyle.append('%s:%d %s' % (name, no, target))
                    page = page[:-3]
                if page and ('/' in page or '.' in page):
                    if not os.path.exists(os.path.join(root, page)):
                        dead.append('%s:%d %s ( no such file )' % (name, no, target))
                    continue
                tgt = page or name
                if tgt not in pages:
                    dead.append('%s:%d %s ( no such page )' % (name, no, target))
                    continue
                if anchor:
                    if tgt not in cache:
                        cache[tgt] = anchors(pages[tgt])
                    if anchor not in cache[tgt]:
                        dead.append('%s:%d %s ( no such heading )' % (name, no, target))
    for d in dead:
        print('DEAD   ', d)
    for d in mdstyle:
        print('.md    ', d)
    print('%d links checked in %d pages, %d dead, %d written with .md' % (total, len(pages), len(dead), len(mdstyle)))
    return 1 if dead else 0


if __name__ == '__main__':
    sys.exit(main())
