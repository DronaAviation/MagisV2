#!/usr/bin/env python3
"""
MagisV2 flight-log analysis for PlutoMonitor captures.

PlutoMonitor writes one field per line:

    [17/09/2026 16:18:02.824] : degC:28.1

A record is one PlutoPilot.cpp log tick. A new record starts whenever a field
name repeats, so no particular field has to come first. Timestamps are the
monitor host's clock ( ms precision ), which is all the log needs: the firmware
`ms` field is optional.

Standard library only ( Python 3.8+ ).

    python tools/flightlog.py summary logs3.txt
    python tools/flightlog.py table   logs3.txt --step 10
    python tools/flightlog.py report  logs3.txt
    python tools/flightlog.py report  logs3.txt --limit 25 --skip 20 40 60

Field names default to the altitude-hold log ( degC, BaroAlt, ToF, PaI, Arm )
and can be overridden with --temp / --alt / --tof / --pressure / --arm.
"""

import argparse
import re
import statistics as st
import sys

LINE_RE   = re.compile(r"\[(\d+)/(\d+)/(\d+) (\d+):(\d+):([\d.]+)\] : (\w+):\s*(-?[\d.]+)\s*$")
INLINE_RE = re.compile(r"(\w+):\s*(-?\d+(?:\.\d+)?)")


# --------------------------------------------------------------------------- parsing

def load ( path ):
    """Return a list of records ( dicts ), each with 't' in seconds from log start."""
    recs, cur, last_t, day_offset = [], {}, None, 0.0
    with open ( path, encoding = "utf-8", errors = "replace" ) as fh:
        for line in fh:
            m = LINE_RE.match ( line )
            if m:
                t = int ( m [ 4 ] ) * 3600 + int ( m [ 5 ] ) * 60 + float ( m [ 6 ] ) + day_offset
                if last_t is not None and t < last_t - 43200:    # crossed midnight
                    day_offset += 86400
                    t += 86400
                last_t = t
                pairs = [ ( m [ 7 ], float ( m [ 8 ] ) ) ]
            else:
                # Older inline format: all fields of a tick on one line, no host time.
                pairs = [ ( k, float ( v ) ) for k, v in INLINE_RE.findall ( line ) ]
                if not pairs:
                    continue
                t = None
            for k, v in pairs:
                if k in cur:
                    recs.append ( cur )
                    cur = {}
                if not cur:
                    cur [ "t" ] = t
                cur [ k ] = v
    if cur:
        recs.append ( cur )
    if not recs:
        sys.exit ( f"{path}: no records found" )
    # Inline logs: fall back to the firmware ms field, else 0.1 s per record.
    if recs [ 0 ] [ "t" ] is None:
        for i, r in enumerate ( recs ):
            r [ "t" ] = r [ "ms" ] / 1000.0 if "ms" in r else i * 0.1
    t0 = recs [ 0 ] [ "t" ]
    for r in recs:
        r [ "t" ] -= t0
    return recs


def complete ( recs, keys ):
    return [ r for r in recs if all ( k in r for k in keys ) ]


def segments ( recs, arm ):
    """Split into runs of constant arm state: [ ( armed, [records] ), ... ]."""
    out, start = [], 0
    for i in range ( 1, len ( recs ) + 1 ):
        if i == len ( recs ) or recs [ i ] [ arm ] != recs [ start ] [ arm ]:
            out.append ( ( int ( recs [ start ] [ arm ] ), recs [ start:i ] ) )
            start = i
    return out


# --------------------------------------------------------------------------- maths

def mean ( rs, k ):
    return st.fmean ( r [ k ] for r in rs )


def fit ( xs, ys ):
    """Least-squares slope, correlation, residual sd."""
    if len ( xs ) < 3:
        return float ( "nan" ), float ( "nan" ), float ( "nan" )
    mx, my = st.fmean ( xs ), st.fmean ( ys )
    sxx = sum ( ( x - mx ) ** 2 for x in xs )
    if sxx == 0:
        return float ( "nan" ), float ( "nan" ), float ( "nan" )
    b = sum ( ( x - mx ) * ( y - my ) for x, y in zip ( xs, ys ) ) / sxx
    try:
        r = st.correlation ( xs, ys )
    except st.StatisticsError:
        r = float ( "nan" )
    res = st.pstdev ( [ y - ( my + b * ( x - mx ) ) for x, y in zip ( xs, ys ) ] )
    return b, r, res


def blocks ( rs, seconds ):
    """Group records into consecutive time blocks; drop sparse blocks."""
    if not rs:
        return []
    out, cur, start = [], [], rs [ 0 ] [ "t" ]
    for r in rs:
        if r [ "t" ] - start >= seconds:
            out.append ( cur )
            cur, start = [], r [ "t" ]
        cur.append ( r )
    out.append ( cur )
    need = max ( 3, int ( seconds * 5 ) )    # at least half the expected 10 Hz ticks
    return [ b for b in out if len ( b ) >= need ]


def cm_per_pa ( pressure_pa ):
    """ISA altitude sensitivity at this pressure, cm per Pa ( ~8.3 at sea level )."""
    return 4433000.0 * 0.190295 * ( pressure_pa / 101325.0 ) ** ( 0.190295 - 1.0 ) / 101325.0


# --------------------------------------------------------------------------- commands

def cmd_summary ( a ):
    recs = load ( a.log )
    counts = {}
    for r in recs:
        for k in r:
            if k != "t":
                counts [ k ] = counts.get ( k, 0 ) + 1
    most = max ( counts.values () )
    print ( f"{a.log}: {len ( recs )} records, {recs [ -1 ] [ 't' ]:.1f} s" )
    for k, n in counts.items ():
        flag = "   <-- missing in {:.0%} of records".format ( 1 - n / most ) if n < 0.95 * most else ""
        print ( f"  {k:10s} {n:6d}{flag}" )
    if any ( n < 0.95 * most for n in counts.values () ):
        print ( "  Fields dropping out usually means the log line exceeded the ~250-byte\n"
                "  Monitor_Print budget ( MSP UART TX buffer overwrite ). Shorten the line." )
    gaps = [ ( recs [ i - 1 ] [ "t" ], recs [ i ] [ "t" ] ) for i in range ( 1, len ( recs ) )
             if recs [ i ] [ "t" ] - recs [ i - 1 ] [ "t" ] > 0.5 ]
    if gaps:
        print ( f"  {len ( gaps )} gaps > 0.5 s, largest {max ( b - x for x, b in gaps ):.1f} s" )
    if a.arm in counts:
        segs = segments ( complete ( recs, [ a.arm ] ), a.arm )
        print ( "  segments: " + ", ".join ( f"{'ARMED' if s else 'ground'} {rs [ 0 ] [ 't' ]:.0f}-{rs [ -1 ] [ 't' ]:.0f}s"
                                             for s, rs in segs ) )


def cmd_table ( a ):
    recs = load ( a.log )
    keys = [ k for k in recs [ 0 ] if k != "t" ]
    print ( "     t " + " ".join ( f"{k:>9s}" for k in keys ) )
    for b in blocks ( recs, a.step ):
        row = []
        for k in keys:
            v = [ r [ k ] for r in b if k in r ]
            row.append ( f"{st.fmean ( v ):9.2f}" if v else f"{'-':>9s}" )
        print ( f"{b [ 0 ] [ 't' ]:6.0f} " + " ".join ( row ) )


def report_tempco ( a, recs, segs ):
    print ( f"\n== Pressure vs temperature ( {a.pressure} vs {a.temp}, {a.block:g} s blocks ) ==" )
    ground = [ b for armed, rs in segs if not armed for b in blocks ( rs, a.block ) ]
    if len ( ground ) >= 4:
        temps = [ mean ( g, a.temp ) for g in ground ]
        b, r, res = fit ( temps, [ mean ( g, a.pressure ) for g in ground ] )
        print ( f"  ground, all disarmed segments ( room assumed constant ): {b:+.2f} Pa/C  r={r:+.2f}  residual {res:.2f} Pa"
                + span_note ( max ( temps ) - min ( temps ) )
                + ( "  [poor fit: pressure is not following temperature alone]" if res > 1.0 else "" ) )
    for i, ( armed, rs ) in enumerate ( segs ):
        if not armed:
            continue
        h = hover ( rs, a.skip [ 0 ] )
        hb = blocks ( h, a.block )
        if len ( hb ) >= 4:
            b, r, res = fit ( [ mean ( g, a.temp ) for g in hb ], [ mean ( g, a.pressure ) for g in hb ] )
            t_first, t_last = mean ( hb [ 0 ], a.temp ), mean ( hb [ -1 ], a.temp )
            print ( f"  flight {i}, hover: {b:+.2f} Pa/C  r={r:+.2f}  residual {res:.2f} Pa  "
                    f"( {a.temp} {t_first:.2f} -> {t_last:.2f} )" + span_note ( abs ( t_last - t_first ) ) )
    for i in range ( 1, len ( segs ) - 1 ):
        if segs [ i ] [ 0 ] == 1 and segs [ i - 1 ] [ 0 ] == 0 and segs [ i + 1 ] [ 0 ] == 0:
            pre, post = segs [ i - 1 ] [ 1 ] [ -100: ], segs [ i + 1 ] [ 1 ] [ :100 ]
            dp, dt = mean ( post, a.pressure ) - mean ( pre, a.pressure ), mean ( post, a.temp ) - mean ( pre, a.temp )
            ratio = f"  -> {dp / dt:+.2f} Pa/C" if abs ( dt ) > 0.5 else ""
            print ( f"  ground closure around flight {i}: {dp:+.2f} Pa over {dt:+.2f} C{ratio}" + span_note ( abs ( dt ) ) )


def span_note ( span ):
    """Pressure noise is ~1 Pa, so a fit over a small temperature span is mostly noise."""
    return f"  [only {span:.1f} C of range - unreliable]" if span < 2.0 else ""


def hover ( rs, skip ):
    """Armed records minus the climb ( first `skip` s ) and the landing ( last 5 s )."""
    t0, t1 = rs [ 0 ] [ "t" ], rs [ -1 ] [ "t" ]
    return [ r for r in rs if t0 + skip < r [ "t" ] < t1 - 5 ]


def report_hover ( a, recs, segs ):
    have_tof = all ( a.tof in r for r in recs [ :50 ] )
    tof0 = None
    if have_tof:
        g = [ r [ a.tof ] for armed, rs in segs if not armed for r in rs if r [ a.tof ] >= 0 ]
        tof0 = st.median ( g ) if g else 0.0
        print ( f"\n== Height hold ( {a.tof} minus on-ground reading {tof0:g} cm is true height ) ==" )
    else:
        print ( f"\n== Height hold ( no {a.tof} field: {a.alt} trend only, no ground truth ) ==" )
    for i, ( armed, rs ) in enumerate ( segs ):
        if not armed or rs [ -1 ] [ "t" ] - rs [ 0 ] [ "t" ] < 30:
            continue
        print ( f"  flight {i}: armed {rs [ 0 ] [ 't' ]:.0f}-{rs [ -1 ] [ 't' ]:.0f} s" )
        for skip in a.skip:
            hb = blocks ( hover ( rs, skip ), a.block )
            if len ( hb ) < 4:
                continue
            tt = [ g [ 0 ] [ "t" ] / 60.0 for g in hb ]
            line = f"    from arm+{skip:>3.0f}s ( {tt [ -1 ] * 60 - tt [ 0 ] * 60:4.0f} s ): "
            b, _, _ = fit ( tt, [ mean ( g, a.alt ) for g in hb ] )
            line += f"{a.alt} {b:+6.1f} cm/min mean {st.fmean ( mean ( g, a.alt ) for g in hb ):5.0f}"
            if have_tof:
                tv = [ g for g in hb if sum ( 1 for r in g if r [ a.tof ] >= 0 ) > len ( g ) / 2 ]
                if len ( tv ) >= 4:
                    h = [ st.fmean ( r [ a.tof ] for r in g if r [ a.tof ] >= 0 ) - tof0 for g in tv ]
                    b, _, _ = fit ( [ g [ 0 ] [ "t" ] / 60.0 for g in tv ], h )
                    line += f" | true height {b:+6.1f} cm/min mean {st.fmean ( h ):5.0f} sd {st.pstdev ( h ):4.1f}"
            print ( line )
        if have_tof:
            steps = blocks ( rs, 30 )
            diffs = []
            for g in steps:
                v = [ r [ a.tof ] for r in g if r [ a.tof ] >= 0 ]
                if v:
                    diffs.append ( f"{st.fmean ( v ) - tof0 - mean ( g, a.alt ):+.0f}" )
            print ( f"    true height minus {a.alt}, 30 s steps from arm: {' '.join ( diffs )} cm" )
    print ( "  A hover under ~2 min gives a trend that swings with the window start;\n"
            "  trust it only if the rows above agree." )


def report_applied ( a, recs, segs ):
    """Back out the correction the firmware applied: comp = -alt / (cm/Pa) - pressure + const."""
    print ( f"\n== Correction applied by the firmware ( from {a.alt} vs {a.pressure} ) ==" )
    k = cm_per_pa ( st.fmean ( r [ a.pressure ] for r in recs ) )
    for i, ( armed, rs ) in enumerate ( segs ):
        if not armed or rs [ -1 ] [ "t" ] - rs [ 0 ] [ "t" ] < 60:
            continue
        comp = [ -r [ a.alt ] / k - r [ a.pressure ] for r in rs ]
        c0 = st.fmean ( comp [ :3 ] )
        cend = st.fmean ( comp [ -60:-15 ] ) if len ( comp ) > 80 else comp [ -1 ]
        t_arm = st.fmean ( r [ a.temp ] for r in rs [ :3 ] )
        t_end = st.fmean ( r [ a.temp ] for r in rs [ -60:-15 ] ) if len ( rs ) > 80 else rs [ -1 ] [ a.temp ]
        hb = blocks ( hover ( rs, a.skip [ 0 ] ), a.block )
        for g in hb:
            for r in g:
                r [ "_comp" ] = -r [ a.alt ] / k - r [ a.pressure ]
        slope = fit ( [ mean ( g, a.temp ) for g in hb ], [ mean ( g, "_comp" ) for g in hb ] ) if len ( hb ) >= 4 else ( float ( "nan" ), ) * 3
        used = cend - c0
        print ( f"  flight {i}: {a.temp} at arm {t_arm:.2f}, end {t_end:.2f} ( +{t_end - t_arm:.2f} C )" )
        print ( f"    applied slope {slope [ 0 ]:+.2f} Pa/C ( temperature coefficient + throttle term ), r={slope [ 1 ]:+.2f}" )
        if a.limit:
            print ( f"    correction used {used:+.1f} Pa = {abs ( used ) / a.limit:.0%} of the {a.limit:g} Pa limit" )
        else:
            print ( f"    correction used {used:+.1f} Pa" )
    print ( f"  ( {k:.2f} cm/Pa at this pressure. The applied slope identifies which coefficient\n"
            "    a build was flashed with; compare against a flight with a known build. )" )


def cmd_report ( a ):
    recs = load ( a.log )
    need = [ a.arm, a.pressure, a.temp, a.alt ]
    full = complete ( recs, need )
    if len ( full ) < 20:
        sys.exit ( f"only {len ( full )} records with all of {need}; check field names with 'summary'" )
    segs = segments ( full, a.arm )
    print ( f"{a.log}: {len ( full )} complete records, {full [ -1 ] [ 't' ]:.0f} s, "
            f"segments: " + ", ".join ( f"{'ARMED' if s else 'ground'} {rs [ 0 ] [ 't' ]:.0f}-{rs [ -1 ] [ 't' ]:.0f}s" for s, rs in segs ) )
    report_tempco ( a, full, segs )
    report_hover ( a, full, segs )
    report_applied ( a, full, segs )


def main ():
    p = argparse.ArgumentParser ( description = __doc__, formatter_class = argparse.RawDescriptionHelpFormatter )
    sub = p.add_subparsers ( dest = "cmd", required = True )
    for name, fn, hlp in ( ( "summary", cmd_summary, "fields, counts, dropouts, gaps, arm segments" ),
                           ( "table",   cmd_table,   "block-averaged table of every field" ),
                           ( "report",  cmd_report,  "temperature fit, height hold, applied correction" ) ):
        s = sub.add_parser ( name, help = hlp )
        s.set_defaults ( fn = fn )
        s.add_argument ( "log" )
        s.add_argument ( "--arm",      default = "Arm" )
        s.add_argument ( "--pressure", default = "PaI" )
        s.add_argument ( "--temp",     default = "degC" )
        s.add_argument ( "--alt",      default = "BaroAlt" )
        s.add_argument ( "--tof",      default = "ToF" )
        s.add_argument ( "--block",    type = float, default = 5.0, help = "averaging block, s ( default 5 )" )
        s.add_argument ( "--step",     type = float, default = 10.0, help = "table row spacing, s ( default 10 )" )
        s.add_argument ( "--skip",     type = float, nargs = "+", default = [ 20, 40, 60 ],
                         help = "hover windows start this many s after arm ( default 20 40 60 )" )
        s.add_argument ( "--limit",    type = float, default = 25.0,
                         help = "BARO_COMP_LIMIT_PA to report headroom against, 0 to skip ( default 25 )" )
    a = p.parse_args ()
    a.fn ( a )


if __name__ == "__main__":
    main ()
