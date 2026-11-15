"""Flag Markdown table rows whose cell count differs from the table's header row.

GitHub splits table cells on every unescaped pipe, even inside backticks, so a stray
pipe in a cell shifts the rest of the row. Usage: python -I table_check.py <files...>
"""
import re
import sys

PIPE = re.compile(r"(?<!\\)\|")
SEPARATOR = re.compile(r"^\|[\s:|-]+\|\s*$")


def pipes(line):
    return len(PIPE.findall(line))


bad = 0
for path in sys.argv[1:]:
    lines = open(path, encoding="utf-8").read().splitlines()
    i, in_code = 0, False
    while i < len(lines):
        line = lines[i]
        if line.lstrip().startswith("```"):
            in_code = not in_code
            i += 1
            continue
        if not in_code and line.startswith("|") and i + 1 < len(lines) and SEPARATOR.match(lines[i + 1]):
            want = pipes(line)
            j = i + 2
            while j < len(lines) and lines[j].startswith("|"):
                if pipes(lines[j]) != want:
                    print(f"{path}:{j + 1}: {pipes(lines[j])} pipes, header has {want}: {lines[j][:100]}")
                    bad += 1
                j += 1
            i = j
            continue
        i += 1
print("mismatched table rows:", bad)
sys.exit(1 if bad else 0)
