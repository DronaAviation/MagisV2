#!/usr/bin/env python3
"""Compile-and-link check for the C++ examples in the MagisV2 GitHub wiki pages.

Each ```cpp block of a page is checked exactly as a user's PlutoPilot.cpp: it is compiled
at global scope with the exact firmware compile command for PlutoPilot.cpp ( read from
`make -n`, so the flags never drift from the Makefile ), with an empty definition added for
each of the five hooks the block does not define. The object then replaces the firmware's
own PlutoPilot.o in the firmware link, into a throwaway ELF. So a call to a function that is
declared but never defined fails here, and so does a file-level name that collides with a
firmware global, as they would for a user.

Nothing outside Build/wikicheck/ is written: PlutoPilot.cpp, Build/<TARGET>/ and the
flashable .hex are never touched by the check itself. The only other write is the normal
incremental `make` that brings Build/<TARGET>/bin up to date first ( skip with --no-make ).

Usage, from the repo root ( Git Bash or PowerShell ):
  python -I docs/fw-development-reference/active-development/api-wiki-sync/wiki_check.py \\
         MagisV2.wiki/BMS.md [more pages]
  --target PRIMUS_X2_v1   board to compile for ( default: selected_target in plutoide.ini )
  --no-make               do not run the incremental make first
  --keep                  keep Build/wikicheck/ ( generated sources, objects, ELFs )
  -j N                    parallel jobs ( default: CPU count )

Markers: an HTML comment ( invisible on GitHub ) on the last non-blank line before a block.
  <!-- wiki-check: skip -->     header quotes, signatures, text that is not meant to compile
  <!-- wiki-check: warn-ok -->  an example that is meant to show a compiler warning

Exit status: 0 when every checked block passes, 1 when any block fails ( compile error,
link error, or a warning in the example's own lines without warn-ok ), 2 on a setup error.
"""

import argparse
import concurrent.futures
import configparser
import os
import re
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

HOOKS = ( "plutoRxConfig", "plutoInit", "onLoopStart", "plutoLoop", "onLoopFinish" )
HOOK_RE = re.compile ( r"\bvoid\s+(" + "|".join ( HOOKS ) + r")\s*\(\s*(?:void)?\s*\)\s*\{" )
FENCE_OPEN = re.compile ( r"^\s*```\s*(cpp|c\+\+)\s*$", re.IGNORECASE )
FENCE_ANY = re.compile ( r"^\s*```" )
MARKER_RE = re.compile ( r"<!--\s*wiki-check:\s*(skip|warn-ok)\s*-->" )
STUBS_FILE = "wiki_check_stubs"    # #line file name for the generated empty hooks


def find_repo ( ) -> Path:
  for parent in Path ( __file__ ).resolve ( ).parents:
    if ( parent / "Makefile" ).is_file ( ) and ( parent / "PlutoPilot.h" ).is_file ( ):
      return parent
  sys.exit ( "wiki_check: repo root ( Makefile + PlutoPilot.h ) not found above this script" )


def toolchain_env ( ) -> dict:
  """PATH with the PlutoIDE toolchain and make in front, as pluto-build's driver.sh does."""
  env = dict ( os.environ )
  home = Path.home ( )
  candidates = [
    home / ".pluto-ide" / "tools" / "ARM GNU ToolChain" / "bin",
    Path ( "C:/PlutoIDE/tools/ARM GNU ToolChain/bin" ),
  ]
  extra = [ str ( c ) for c in candidates if c.is_dir ( ) ][ :1 ]
  if shutil.which ( "make", path = env.get ( "PATH" ) ) is None:
    cyg = Path ( "C:/PlutoIDE/tools/CYGWIN 64/bin" )
    if cyg.is_dir ( ):
      extra.append ( str ( cyg ) )
  env [ "PATH" ] = os.pathsep.join ( extra + [ env.get ( "PATH", "" ) ] )
  return env


def ini_value ( repo: Path, key: str, default: str ) -> str:
  ini = configparser.ConfigParser ( )
  ini.read ( repo / "plutoide.ini" )
  return ini.get ( "Project", key, fallback = default )


def exe ( env: dict, name: str ) -> str:
  """Full path of a tool on env's PATH. Windows CreateProcess searches the parent's PATH, not env's."""
  found = shutil.which ( name, path = env [ "PATH" ] )
  if found is None:
    print ( f"wiki_check: {name} not found ( PlutoIDE toolchain / make )", file = sys.stderr )
    sys.exit ( 2 )
  return found


def firmware_commands ( repo: Path, env: dict, target: str, project: str ):
  """Exact compile line for PlutoPilot.cpp and the ELF link line, from make -n."""
  out = subprocess.run ( [ exe ( env, "make" ), f"TARGET={target}", f"PROJECT={project}", "-n", "-W", "PlutoPilot.cpp" ],
                         cwd = repo, env = env, capture_output = True, text = True )
  compile_line = link_line = None
  for line in out.stdout.splitlines ( ):
    if not line.startswith ( "arm-none-eabi-g++" ):
      continue
    if " -c " in line and "PlutoPilot.o" in line:
      compile_line = line
    elif ".elf" in line and " -o " in line:
      link_line = line
  if not compile_line or not link_line:
    sys.stderr.write ( out.stdout [ -2000: ] + out.stderr [ -2000: ] )
    sys.exit ( 2 )
  cc, ld = shlex.split ( compile_line ), shlex.split ( link_line )
  cc [ 0 ] = ld [ 0 ] = exe ( env, "arm-none-eabi-g++" )
  return cc, ld


def drop_option ( args, opt ):
  """Remove `opt` and the value after it."""
  i = args.index ( opt )
  return args [ :i ] + args [ i + 2: ]


def compile_args ( base, src: Path, obj: Path ):
  args = drop_option ( base, "-o" )
  args = [ a for a in args if a not in ( "-c", "-MMD", "-MP", "PlutoPilot.cpp" ) ]
  return args + [ "-c", str ( src ), "-o", str ( obj ) ]


def link_args ( base, obj: Path, elf: Path, mapfile: Path ):
  """The firmware link, into `elf`, with the block's object in place of the firmware's PlutoPilot.o."""
  args = list ( base )
  i = args.index ( "-o" )
  args [ i + 1 ] = str ( elf )
  args = [ re.sub ( r"-Map,[^,]+", "-Map," + str ( mapfile ).replace ( "\\", "/" ), a ) for a in args ]
  user = [ k for k, a in enumerate ( args ) if a.replace ( "\\", "/" ).endswith ( "/PlutoPilot.o" ) ]
  if len ( user ) != 1:
    sys.exit ( "wiki_check: the firmware link line does not name exactly one PlutoPilot.o" )
  args [ user [ 0 ] ] = str ( obj )
  return args


def extract_blocks ( page: Path ):
  """( first_line, marker, lines ) for every ```cpp block; first_line is 1-based."""
  lines = page.read_text ( encoding = "utf-8" ).splitlines ( )
  blocks, i = [ ], 0
  while i < len ( lines ):
    if FENCE_OPEN.match ( lines [ i ] ):
      marker, j = None, i - 1
      while j >= 0 and not lines [ j ].strip ( ):
        j -= 1
      if j >= 0:
        m = MARKER_RE.search ( lines [ j ] )
        marker = m.group ( 1 ) if m else None
      start, body = i + 2, [ ]
      i += 1
      while i < len ( lines ) and not FENCE_ANY.match ( lines [ i ] ):
        body.append ( lines [ i ] )
        i += 1
      blocks.append ( ( start, marker, body ) )
    i += 1
  return blocks


def generate_tu ( page_label: str, start: int, body ):
  """The block as a user's PlutoPilot.cpp, plus empty definitions of the hooks it lacks.

  The block stays at global scope, includes and all, under a #line that maps every
  diagnostic back to the wiki page. The firmware calls all five hooks, so the stubs both
  complete the file and keep the block's hooks referenced through the link."""
  hooks = sorted ( set ( HOOK_RE.findall ( "\n".join ( body ) ) ), key = HOOKS.index )
  stubs = "".join ( f"void {h} ( void ) {{\n}}\n" for h in HOOKS if h not in hooks )
  src = ( f"// generated by wiki_check.py from {page_label}:{start}\n"
          '#include "PlutoPilot.h"\n'
          f'#line {start} "{page_label}"\n' + "\n".join ( body ) + "\n"
          f'#line 1 "{STUBS_FILE}"\n' + stubs )
  return src, hooks


def check_block ( job ):
  page_label, start, body, marker, workdir, cc_base, ld_base, env, repo = job
  stem = f"{Path ( page_label ).stem}_L{start}"
  src, obj = workdir / f"{stem}.cpp", workdir / f"{stem}.o"
  elf, mapfile = workdir / f"{stem}.elf", workdir / f"{stem}.map"
  text, hooks = generate_tu ( page_label, start, body )
  src.write_text ( text, encoding = "utf-8" )
  where = f"{page_label}:{start}"

  cc = subprocess.run ( compile_args ( cc_base, src, obj ), cwd = repo, env = env,
                        capture_output = True, text = True )
  own = [ l for l in cc.stderr.splitlines ( ) if l.startswith ( page_label + ":" ) ]
  if cc.returncode != 0:
    errs = [ l for l in own if "error" in l ] or cc.stderr.splitlines ( ) [ -12: ]
    return ( "FAIL", where, "compile", hooks, errs )
  warns = [ l for l in own if "warning:" in l ]
  if warns and marker != "warn-ok":
    return ( "FAIL", where, "warning", hooks, warns )
  if not hooks:
    return ( "FAIL", where, "no hook", hooks,
             [ "block defines none of the five hooks: wrap it in one, or mark it skip" ] )

  ld = subprocess.run ( link_args ( ld_base, obj, elf, mapfile ), cwd = repo, env = env,
                        capture_output = True, text = True )
  if ld.returncode != 0:
    # Keep only the missing symbol: the object path in front of it says nothing to the reader.
    errs = sorted ( { l [ l.index ( "undefined reference" ): ] for l in ld.stderr.splitlines ( )
                      if "undefined reference" in l } )
    return ( "FAIL", where, "link", hooks, errs [ :12 ] or ld.stderr.splitlines ( ) [ -12: ] )
  if warns:
    return ( "WARN-OK", where, "", hooks, warns )
  return ( "PASS", where, "", hooks, [ ] )


def main ( ) -> int:
  repo = find_repo ( )
  ap = argparse.ArgumentParser ( description = __doc__.split ( "\n\n" ) [ 0 ] )
  ap.add_argument ( "pages", nargs = "+" )
  ap.add_argument ( "--target", default = ini_value ( repo, "selected_target", "PRIMUS_X2_v1" ) )
  ap.add_argument ( "--no-make", action = "store_true" )
  ap.add_argument ( "--keep", action = "store_true" )
  ap.add_argument ( "-j", type = int, default = os.cpu_count ( ) or 4 )
  opts = ap.parse_args ( )

  env = toolchain_env ( )
  exe ( env, "arm-none-eabi-g++" )
  project = ini_value ( repo, "project_name", "DEFAULT" )

  if not opts.no_make:
    print ( f"make TARGET={opts.target} PROJECT={project} ( incremental, brings the objects up to date )" )
    built = subprocess.run ( [ exe ( env, "make" ), f"TARGET={opts.target}", f"PROJECT={project}" ], cwd = repo, env = env,
                             capture_output = True, text = True )
    if built.returncode != 0:
      sys.stderr.write ( built.stdout [ -3000: ] + built.stderr [ -3000: ] )
      print ( "wiki_check: firmware build failed; fix that first", file = sys.stderr )
      return 2

  cc_base, ld_base = firmware_commands ( repo, env, opts.target, project )
  workdir = repo / "Build" / "wikicheck" / opts.target
  shutil.rmtree ( workdir, ignore_errors = True )
  workdir.mkdir ( parents = True )

  jobs, skipped = [ ], [ ]
  for page in opts.pages:
    path = Path ( page ).resolve ( )
    label = path.relative_to ( repo ).as_posix ( ) if path.is_relative_to ( repo ) else path.as_posix ( )
    for start, marker, body in extract_blocks ( path ):
      if marker == "skip":
        skipped.append ( f"{label}:{start}" )
      else:
        jobs.append ( ( label, start, body, marker, workdir, cc_base, ld_base, env, repo ) )

  with concurrent.futures.ThreadPoolExecutor ( max_workers = max ( 1, opts.j ) ) as pool:
    results = list ( pool.map ( check_block, jobs ) )

  for where in skipped:
    print ( f"SKIP     {where}" )
  failed = 0
  for status, where, kind, hooks, detail in results:
    print ( f"{status:<8} {where}  {kind + '  ' if kind else ''}[{', '.join ( hooks ) or '-'}]" )
    for line in detail:
      print ( f"           {line}" )
    failed += status == "FAIL"

  print ( f"\n{len ( results )} checked, {len ( results ) - failed} passed, {failed} failed, "
          f"{len ( skipped )} skipped ( target {opts.target} )" )
  if not opts.keep:
    shutil.rmtree ( workdir, ignore_errors = True )
    try:
      workdir.parent.rmdir ( )    # Build/wikicheck, if no other target's run is kept there
    except OSError:
      pass
  return 1 if failed else 0


if __name__ == "__main__":
  sys.exit ( main ( ) )
