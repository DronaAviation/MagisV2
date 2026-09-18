#!/usr/bin/env python3
"""
Name graphify communities after what they contain, then regenerate the
graph.html visualisation and GRAPH_REPORT.md with those names.

`graphify update .` ( AST-only, no LLM ) re-clusters the graph and falls back to
"Community 0, 1, 2 ...". Community ids can change on every re-cluster, so saved
names go stale; this script derives names from the members instead, so it can
be re-run after every update and always matches the current clustering.

    graphify update .                  # refresh the graph from the code
    python tools/graph_labels.py       # name the communities, rewrite html + report

Name format:  <area>/<file>: <most-connected symbol>
    flight/altitudehold: applyAltHold()
    StdPeriph/tim: TIM_TimeBaseInit()
    blackbox + io: blackbox, flashfs             ( no single dominant area / file )

It reads the cluster assignment already stored in graphify-out/graph.json and
does not re-cluster. Needs the graphify package ( pip install graphifyy ).
"""

import collections
import json
import re
import sys
from pathlib import Path

try:
    from graphify.analyze import god_nodes, suggest_questions, surprising_connections
    from graphify.build import build_from_json
    from graphify.cluster import score_all
    from graphify.detect import detect
    from graphify.export import to_html
    from graphify.report import generate
except ImportError:
    sys.exit ( "graphify is not installed: pip install graphifyy" )

OUT = Path ( "graphify-out" )

# Vendored / top-level paths -> short area names. Checked in order, first match wins.
AREAS = [
    ( "lib/main/STM32F30x_StdPeriph_Driver", "StdPeriph" ),
    ( "lib/main/STM32_USB-FS-Device_Driver", "USB-FS" ),
    ( "lib/main/CMSIS",                      "CMSIS" ),
    ( "lib/main/VL53L0X_API",                "VL53L0X" ),
    ( "lib/main/VL53L1X_API",                "VL53L1X" ),
    ( "lib/test",                            "gtest" ),
    ( "src/main/API-Src",                    "API" ),
    ( "src/main/API",                        "API" ),
    ( "PlutoPilot",                          "user" ),
    ( "tools",                               "tools" ),
]
STRIP_PREFIXES = ( "stm32f30x_", "vl53l0x_", "vl53l1_", "usb_", "core_" )


def area_of ( path ):
    for prefix, name in AREAS:
        if path.startswith ( prefix ):
            return name
    parts = path.split ( "/" )
    if len ( parts ) >= 4 and parts [ 0 ] == "src" and parts [ 1 ] == "main":
        return parts [ 2 ]                      # flight, drivers, io, sensors, rx, ...
    if len ( parts ) == 3 and parts [ :2 ] == [ "src", "main" ]:
        return "core"                           # main.cpp, mw.cpp, ...
    return parts [ 0 ] if len ( parts ) > 1 else "root"


def file_of ( path ):
    stem = Path ( path ).stem
    low = stem.lower ()
    for p in STRIP_PREFIXES:
        if low.startswith ( p ) and len ( stem ) > len ( p ):
            return stem [ len ( p ): ]
    return stem


def clean_symbol ( label ):
    label = re.sub ( r"\s+", " ", label ).strip ()
    return label if len ( label ) <= 32 else label [ :31 ] + "…"


def name_communities ( G, communities ):
    labels, used = {}, collections.Counter ()
    for cid, members in communities.items ():
        paths = [ ( G.nodes [ n ].get ( "source_file" ) or "" ).replace ( "\\", "/" ).lstrip ( "./" ) for n in members ]
        paths = [ p for p in paths if p ]
        if not paths:
            labels [ cid ] = f"Community {cid}"
            continue
        areas = collections.Counter ( area_of ( p ) for p in paths )
        files = collections.Counter ( file_of ( p ) for p in paths )
        area, area_n = areas.most_common ( 1 ) [ 0 ]
        mixed = area_n < 0.6 * len ( paths ) and len ( areas ) > 1
        if mixed:
            area = " + ".join ( a for a, _ in areas.most_common ( 2 ) )
        top_file, file_n = files.most_common ( 1 ) [ 0 ]
        # Most-connected member that is not just the file node itself.
        ranked = sorted ( members, key = lambda n: G.degree ( n ), reverse = True )
        symbol = next ( ( G.nodes [ n ].get ( "label", n ) for n in ranked
                          if not re.search ( r"\.(c|cc|cpp|cxx|h|hh|hpp|hxx|s|py|md|ld)$", G.nodes [ n ].get ( "label", "" ), re.I ) ), None )
        if len ( files ) > 1 and ( mixed or file_n < 0.5 * len ( paths ) ):
            second = files.most_common ( 2 ) [ 1 ] [ 0 ]
            name = f"{area}: {top_file}, {second}"
        else:
            name = f"{area}/{top_file}"
            if symbol:
                name += f": {clean_symbol ( symbol )}"
        used [ name ] += 1
        if used [ name ] > 1:                   # keep names unique in the legend
            name = f"{name} ({used [ name ]})"
        labels [ cid ] = name
    return labels


def main ():
    graph_path = OUT / "graph.json"
    if not graph_path.exists ():
        sys.exit ( f"{graph_path} not found - run `graphify update .` first" )
    raw = json.loads ( graph_path.read_text ( encoding = "utf-8" ) )
    G = build_from_json ( raw, directed = bool ( raw.get ( "directed", False ) ) )

    communities = collections.defaultdict ( list )
    for n, d in G.nodes ( data = True ):
        if d.get ( "community" ) is not None:
            communities [ int ( d [ "community" ] ) ].append ( n )
    communities = dict ( communities )
    if not communities:
        sys.exit ( "graph.json has no community assignments - run `graphify update .` first" )

    labels = name_communities ( G, communities )

    # Same inputs graphify's own rebuild passes to the report.
    detected = detect ( Path ( "." ) )
    detection = {
        "files": { "code": detected [ "files" ].get ( "code", [] ), "document": [], "paper": [], "image": [] },
        "total_files": len ( detected [ "files" ].get ( "code", [] ) ),
        "total_words": detected.get ( "total_words", 0 ),
    }
    cohesion  = score_all ( G, communities )
    gods      = god_nodes ( G )
    surprises = surprising_connections ( G, communities )
    questions = suggest_questions ( G, communities, labels )
    report = generate ( G, communities, cohesion, labels, gods, surprises, detection,
                        { "input": 0, "output": 0 }, Path.cwd ().name, suggested_questions = questions,
                        built_at_commit = raw.get ( "built_at_commit" ) )
    ( OUT / "GRAPH_REPORT.md" ).write_text ( report, encoding = "utf-8" )
    ( OUT / ".graphify_labels.json" ).write_text (
        json.dumps ( { str ( k ): v for k, v in labels.items () }, ensure_ascii = False ), encoding = "utf-8" )
    to_html ( G, communities, str ( OUT / "graph.html" ), community_labels = labels )

    sizes = sorted ( communities, key = lambda c: len ( communities [ c ] ), reverse = True )
    print ( f"Named {len ( labels )} communities; graph.html and GRAPH_REPORT.md rewritten. Largest:" )
    for cid in sizes [ :10 ]:
        print ( f"  {len ( communities [ cid ] ):4d}  {labels [ cid ]}" )


if __name__ == "__main__":
    main ()
