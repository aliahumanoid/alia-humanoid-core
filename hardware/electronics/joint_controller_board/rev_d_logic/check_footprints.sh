#!/usr/bin/env bash
set -euo pipefail

# Run from rev_d_logic folder
SCH_FILE="joint_controller_board_rev_d_logic.kicad_sch"
PCB_FILE="joint_controller_board_rev_d_logic.kicad_pcb"
BOM_FILE="BOM_fabrication_combined.csv"
XML_FILE="rev_d_logic_current.xml"

if [[ -n "${KICAD_CLI:-}" ]]; then
  :
elif command -v kicad-cli >/dev/null 2>&1; then
  KICAD_CLI="$(command -v kicad-cli)"
elif [[ -x "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli" ]]; then
  KICAD_CLI="/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"
else
  KICAD_CLI=""
fi

if [[ -z "$KICAD_CLI" || ! -x "$KICAD_CLI" ]]; then
  echo "ERROR: kicad-cli not found. Set KICAD_CLI or add it to PATH."
  exit 2
fi

if [[ ! -f "$SCH_FILE" || ! -f "$PCB_FILE" || ! -f "$BOM_FILE" ]]; then
  echo "ERROR: missing required files in current directory"
  exit 2
fi

# 1) Export current schematic netlist/xml
"$KICAD_CLI" sch export netlist --format kicadxml -o "$XML_FILE" "$SCH_FILE" >/dev/null

# 2) Build ref->footprint maps
awk '
/^[[:space:]]*\(footprint "/{fp=$0; gsub(/.*\(footprint \"|\".*/,"",fp); ref=""}
/^[[:space:]]*\(property "Reference" "/{
  if(ref==""){
    r=$0; gsub(/.*\(property "Reference" "|".*/,"",r);
    ref=r;
    if(ref!="${REFERENCE}") print ref "," fp
  }
}
' "$PCB_FILE" | sort > /tmp/rev_d_logic_pcb_ref_fp.csv

awk '
/^[[:space:]]*<comp ref=/{ref=$0; gsub(/.*ref=\"|\".*/,"",ref)}
/^[[:space:]]*<footprint>/{fp=$0; gsub(/.*<footprint>|<\/footprint>.*/,"",fp); if(ref!="") print ref "," fp}
' "$XML_FILE" | sort > /tmp/rev_d_logic_sch_ref_fp.csv

# 3) Check schematic vs PCB exact
if ! diff -q /tmp/rev_d_logic_sch_ref_fp.csv /tmp/rev_d_logic_pcb_ref_fp.csv >/dev/null; then
  echo "FAIL: schematic and PCB footprints are not aligned"
  diff -u /tmp/rev_d_logic_sch_ref_fp.csv /tmp/rev_d_logic_pcb_ref_fp.csv | sed -n '1,200p'
  exit 1
fi

# 4) Normalize schematic footprints (strip library prefix)
awk -F',' '{ref=$1; fp=$2; sub(/^[^:]*:/,"",fp); print ref "," fp}' /tmp/rev_d_logic_sch_ref_fp.csv | sort > /tmp/rev_d_logic_sch_ref_fp_norm.csv
awk '
NR==1 { next }
{
  line=$0

  # Parse field 1 (Designator), supporting quoted CSV.
  if (substr(line,1,1)=="\"") {
    line=substr(line,2)
    q1=index(line,"\"")
    des=substr(line,1,q1-1)
    line=substr(line,q1+2)   # skip quote and comma
  } else {
    c1=index(line,",")
    des=substr(line,1,c1-1)
    line=substr(line,c1+1)
  }

  # Parse field 2 (Footprint), supporting quoted CSV.
  if (substr(line,1,1)=="\"") {
    line=substr(line,2)
    q2=index(line,"\"")
    fp=substr(line,1,q2-1)
  } else {
    c2=index(line,",")
    fp=substr(line,1,c2-1)
  }

  print des "," fp
}
' "$BOM_FILE" | sort > /tmp/rev_d_logic_bom_ref_fp.csv

# 5) Compare BOM vs schematic footprint names by designator
join -t, -a1 -a2 -e MISSING -o 0,1.2,2.2 \
  <(cut -d, -f1,2 /tmp/rev_d_logic_sch_ref_fp_norm.csv) \
  <(cut -d, -f1,2 /tmp/rev_d_logic_bom_ref_fp.csv) \
  > /tmp/rev_d_logic_join.csv

awk -F',' '($2!=$3){print}' /tmp/rev_d_logic_join.csv > /tmp/rev_d_logic_fp_mismatch.csv

if [[ -s /tmp/rev_d_logic_fp_mismatch.csv ]]; then
  echo "FAIL: BOM footprint mismatches found"
  cat /tmp/rev_d_logic_fp_mismatch.csv
  exit 1
fi

echo "OK: footprint check passed"
echo "- SCH vs PCB: aligned"
echo "- BOM vs SCH: aligned"
