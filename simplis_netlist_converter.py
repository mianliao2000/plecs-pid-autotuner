"""
Create SIMetrix/SIMPLIS visual schematics from SIMPLIS netlists.

The converter is deliberately focused on old SIMetrix/SIMPLIS versions that do
not have the 9.2 netlist-to-schematic converter. It parses the plain netlist and
emits a SIMetrix script that builds a readable `.sxsch` schematic.
"""

from __future__ import annotations

import argparse
import json
import re
import shlex
import subprocess
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable


NODE_COUNTS = {
    "R": 2,
    "C": 2,
    "L": 2,
    "V": 2,
    "I": 2,
    "D": 2,
    "J": 3,
    "Q": 3,
    "M": 4,
    "S": 4,
    "E": 4,
    "G": 4,
    "F": 2,
    "H": 2,
    "T": 4,
    "K": 2,
}


@dataclass(frozen=True)
class NetlistElement:
    ref: str
    kind: str
    nodes: list[str]
    value: str
    params: list[str]
    scope: str
    line_number: int
    raw: str


@dataclass(frozen=True)
class NetlistControl:
    keyword: str
    args: list[str]
    scope: str
    line_number: int
    raw: str


@dataclass(frozen=True)
class Subcircuit:
    name: str
    pins: list[str]
    line_number: int


@dataclass
class ParsedNetlist:
    source: str
    elements: list[NetlistElement]
    controls: list[NetlistControl]
    subcircuits: dict[str, Subcircuit]
    warnings: list[str]

    def scopes(self) -> list[str]:
        scopes = {"TOP"}
        scopes.update(element.scope for element in self.elements)
        scopes.update(control.scope for control in self.controls)
        return ["TOP", *sorted(scope for scope in scopes if scope != "TOP")]

    def elements_in_scope(self, scope: str) -> list[NetlistElement]:
        return [element for element in self.elements if element.scope == scope]


def normalize_net_name(name: str) -> str:
    return "GND" if name in {"0", "GND", "gnd"} else name


def logical_kind(ref: str) -> str:
    stripped = ref.lstrip("!@$")
    for char in stripped:
        if char.isalpha():
            return char.upper()
    return ref[:1].upper() if ref else "?"


def strip_inline_comment(line: str) -> str:
    in_braces = 0
    for idx, char in enumerate(line):
        if char == "{":
            in_braces += 1
        elif char == "}":
            in_braces = max(0, in_braces - 1)
        elif char == ";" and in_braces == 0:
            return line[:idx].rstrip()
    return line.rstrip()


def join_continuations(text: str) -> list[tuple[int, str]]:
    joined: list[tuple[int, str]] = []
    current_line_no = 0
    current = ""
    for line_no, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("*"):
            continue
        if line.startswith("+"):
            current = f"{current} {strip_inline_comment(line[1:].strip())}".strip()
            continue
        if current:
            joined.append((current_line_no, current))
        current_line_no = line_no
        current = strip_inline_comment(line)
    if current:
        joined.append((current_line_no, current))
    return joined


def split_tokens(line: str) -> list[str]:
    lexer = shlex.shlex(line, posix=False)
    lexer.whitespace_split = True
    lexer.commenters = ""
    return list(lexer)


def collect_subcircuit_names(lines: Iterable[tuple[int, str]]) -> dict[str, Subcircuit]:
    subcircuits: dict[str, Subcircuit] = {}
    for line_no, line in lines:
        tokens = split_tokens(line)
        if len(tokens) >= 2 and tokens[0].upper() == ".SUBCKT":
            name = tokens[1]
            pins = [token for token in tokens[2:] if "=" not in token and token.upper() != "PARAMS:"]
            subcircuits[name.upper()] = Subcircuit(name=name, pins=pins, line_number=line_no)
    return subcircuits


def parse_element(
    line_no: int,
    line: str,
    tokens: list[str],
    scope: str,
    subcircuits: dict[str, Subcircuit],
) -> NetlistElement:
    ref = tokens[0]
    kind = logical_kind(ref)
    if kind == "X":
        split_at = len(tokens)
        for idx in range(len(tokens) - 1, 0, -1):
            token = tokens[idx]
            if token.upper() in {"PARAMS:", "VARS:"}:
                continue
            if token.upper() in subcircuits or "=" not in token:
                split_at = idx
                break
        nodes = [normalize_net_name(token) for token in tokens[1:split_at]]
        value = tokens[split_at] if split_at < len(tokens) else ""
        params = tokens[split_at + 1 :] if split_at + 1 < len(tokens) else []
        return NetlistElement(ref, kind, nodes, value, params, scope, line_no, line)

    node_count = NODE_COUNTS.get(kind, 2)
    nodes = [normalize_net_name(token) for token in tokens[1 : 1 + node_count]]
    rest = tokens[1 + node_count :]
    value = rest[0] if rest else ""
    params = rest[1:] if len(rest) > 1 else []
    return NetlistElement(ref, kind, nodes, value, params, scope, line_no, line)


def parse_netlist(path: Path) -> ParsedNetlist:
    text = path.read_text(encoding="utf-8", errors="replace")
    lines = join_continuations(text)
    subcircuits = collect_subcircuit_names(lines)
    elements: list[NetlistElement] = []
    controls: list[NetlistControl] = []
    warnings: list[str] = []
    scope_stack = ["TOP"]

    for line_no, line in lines:
        tokens = split_tokens(line)
        if not tokens:
            continue
        first = tokens[0].upper()
        scope = scope_stack[-1]

        if first.startswith("."):
            controls.append(NetlistControl(first, tokens[1:], scope, line_no, line))
            if first == ".SUBCKT" and len(tokens) >= 2:
                scope_stack.append(tokens[1])
            elif first == ".ENDS" and len(scope_stack) > 1:
                scope_stack.pop()
            continue

        element = parse_element(line_no, line, tokens, scope, subcircuits)
        if len(element.nodes) < 2 and element.kind != "K":
            warnings.append(f"Line {line_no}: {element.ref} has fewer than two parsed nodes")
        elements.append(element)

    if len(scope_stack) > 1:
        warnings.append(f"Unclosed subcircuit scope(s): {', '.join(scope_stack[1:])}")

    return ParsedNetlist(
        source=str(path),
        elements=elements,
        controls=controls,
        subcircuits=subcircuits,
        warnings=warnings,
    )


def parsed_vars(parsed: ParsedNetlist) -> dict[str, str]:
    values: dict[str, str] = {}
    for control in parsed.controls:
        if control.keyword != ".VAR":
            continue
        match = re.match(r"\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.+?)\s*$", " ".join(control.args))
        if match:
            values[match.group(1)] = match.group(2)
    return values


def element_by_ref(parsed: ParsedNetlist) -> dict[str, NetlistElement]:
    return {element.ref.upper(): element for element in parsed.elements}


def value_of(vars_found: dict[str, str], name: str, fallback: str) -> str:
    return vars_found.get(name, fallback)


def element_value(element: NetlistElement | None, fallback: str = "") -> str:
    if element is None:
        return fallback
    return " ".join(part for part in [element.value, *element.params] if part) or fallback


def scope_net_counts(elements: Iterable[NetlistElement]) -> Counter[str]:
    counts: Counter[str] = Counter()
    for element in elements:
        counts.update(element.nodes)
    return counts


def render_json(parsed: ParsedNetlist) -> str:
    data = {
        "source": parsed.source,
        "elements": [asdict(element) for element in parsed.elements],
        "controls": [asdict(control) for control in parsed.controls],
        "subcircuits": {name: asdict(subckt) for name, subckt in parsed.subcircuits.items()},
        "warnings": parsed.warnings,
    }
    return json.dumps(data, indent=2)


def dot_id(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]", "_", text)


def render_dot(parsed: ParsedNetlist) -> str:
    out = [
        "graph simplis_netlist {",
        '  graph [rankdir=LR, overlap=false, splines=true];',
        '  node [fontname="Segoe UI"];',
    ]
    for scope in parsed.scopes():
        elements = parsed.elements_in_scope(scope)
        if not elements:
            continue
        out.append(f'  subgraph cluster_{dot_id(scope)} {{')
        out.append(f'    label="{scope}";')
        out.append('    color="#cbd5e1";')
        for element in elements:
            element_id = f"e_{dot_id(scope)}_{dot_id(element.ref)}"
            label = f"{element.ref}\\n{element.kind} {element.value}".strip()
            out.append(f'    {element_id} [shape=box, style=rounded, label="{label}"];')
            for net in element.nodes:
                net_id = f"n_{dot_id(scope)}_{dot_id(net)}"
                out.append(f'    {net_id} [shape=ellipse, label="{net}"];')
                out.append(f"    {element_id} -- {net_id};")
        out.append("  }")
    out.append("}")
    return "\n".join(out) + "\n"


def script_quote(text: str) -> str:
    return '"' + text.replace("\\", "/").replace('"', '\\"') + '"'


def label_cmd(text: str, x: int, y: int, size: int = 9) -> str:
    escaped = text.replace("\\", "/").replace('"', '\\"').replace("\n", "\\n")
    return f'NewLabel "{escaped}" /size {size} /loc {x} {y}'


def inst_cmd(symbol: str, ref: str, x: int, y: int, value: str = "", note: str = "") -> list[str]:
    lines = [f"Inst /loc {x} {y} 0 {symbol} ref {script_quote(ref)}"]
    label = ref
    if value:
        label += f"\n{value}"
    if note:
        label += f"\n{note}"
    lines.append(label_cmd(label, x + 170, y - 20, 9))
    return lines


def symbol_definitions() -> list[str]:
    return [
        "* Local generic symbols for readable netlist visualization",
        'CreateSym simvis_vsrc "visual voltage source" analog',
        "AddCirc -60 -60 60 60",
        "AddSeg 0 -120 0 -60",
        "AddSeg 0 60 0 120",
        "AddSeg -24 -12 24 -12",
        "AddSeg 0 -36 0 12",
        "AddSeg -36 120 36 120",
        "AddSeg -24 144 24 144",
        "AddSeg -12 168 12 168",
        "AddPin P 1 0 -120",
        "AddPin N 2 0 120",
        "AddProp ref V? 26 80 -40",
        "EndSym",
        "",
        'CreateSym simvis_isrc "visual current source" analog',
        "AddCirc -60 -60 60 60",
        "AddSeg 0 -120 0 -60",
        "AddSeg 0 60 0 120",
        "AddSeg 0 -30 0 30",
        "AddSeg -18 12 0 30",
        "AddSeg 18 12 0 30",
        "AddPin P 1 0 -120",
        "AddPin N 2 0 120",
        "AddProp ref I? 26 80 -40",
        "EndSym",
        "",
        'CreateSym simvis_res "visual resistor" analog',
        "AddSeg 0 -120 0 -80",
        "AddSeg 0 -80 -40 -60",
        "AddSeg -40 -60 40 -20",
        "AddSeg 40 -20 -40 20",
        "AddSeg -40 20 40 60",
        "AddSeg 40 60 0 80",
        "AddSeg 0 80 0 120",
        "AddPin P 1 0 -120",
        "AddPin N 2 0 120",
        "AddProp ref R? 26 80 -40",
        "EndSym",
        "",
        'CreateSym simvis_cap "visual capacitor" analog',
        "AddSeg 0 -120 0 -25",
        "AddSeg -60 -25 60 -25",
        "AddSeg -60 25 60 25",
        "AddSeg 0 25 0 120",
        "AddPin P 1 0 -120",
        "AddPin N 2 0 120",
        "AddProp ref C? 26 80 -40",
        "EndSym",
        "",
        'CreateSym simvis_ind "visual inductor" analog',
        "AddSeg -120 0 -80 0",
        "AddArc -80 -40 0 40 180 180",
        "AddArc 0 -40 80 40 180 180",
        "AddSeg 80 0 120 0",
        "AddPin P 1 -120 0",
        "AddPin N 2 120 0",
        "AddProp ref L? 26 -20 -90",
        "EndSym",
        "",
        'CreateSym simvis_diode "visual diode" analog',
        "AddSeg 0 -120 0 -50",
        "AddSeg -50 -50 0 35",
        "AddSeg 50 -50 0 35",
        "AddSeg -50 -50 50 -50",
        "AddSeg -50 40 50 40",
        "AddSeg 0 40 0 120",
        "AddPin A 1 0 -120",
        "AddPin K 2 0 120",
        "AddProp ref D? 26 80 -40",
        "EndSym",
        "",
        'CreateSym simvis_switch "visual controlled switch" analog',
        "AddSeg 0 -120 0 -45",
        "AddSeg 0 -45 70 45",
        "AddSeg 0 45 0 120",
        "AddSeg -120 -60 -25 -20",
        "AddSeg -120 60 -25 20",
        "AddPin P 1 0 -120",
        "AddPin N 2 0 120",
        "AddPin CP 3 -120 -60",
        "AddPin CN 4 -120 60",
        "AddProp ref S? 26 90 -35",
        "EndSym",
        "",
        'CreateSym simvis_block "visual function block" analog',
        "AddSeg -80 -60 80 -60",
        "AddSeg 80 -60 80 60",
        "AddSeg 80 60 -80 60",
        "AddSeg -80 60 -80 -60",
        "AddPin P1 1 -120 -40",
        "AddPin P2 2 -120 40",
        "AddPin P3 3 120 -40",
        "AddPin P4 4 120 40",
        "AddProp ref U? 26 -50 -90",
        "EndSym",
        "",
    ]


def render_readable_simetrix_script(parsed: ParsedNetlist, output_sxsch: Path) -> str:
    vars_found = parsed_vars(parsed)
    refs = element_by_ref(parsed)
    output_sxsch = output_sxsch.resolve()

    lines: list[str] = [
        "* Generated by simplis_netlist_converter.py",
        "* Readable schematic view. The runnable model remains synchronous_buck_tran.net.",
        "NewSchem /simulator SIMPLIS Synchronous_Buck_SIMPLIS_Readable",
        *symbol_definitions(),
        label_cmd("Synchronous Buck Converter - SIMPLIS readable schematic", -1880, -1320, 14),
        label_cmd("Visual schematic with wide spacing. Automation runs synchronous_buck_tran.net.", -1880, -1200, 10),
        label_cmd("Input", -1880, -960, 11),
        *inst_cmd("simvis_vsrc", "VDC", -1800, -660, value_of(vars_found, "Vdc", "{Vdc}"), "vin to 0"),
        *inst_cmd("simvis_cap", "Cin", -1440, -660, "not in SIMPLIS netlist", "visual input bypass"),
        label_cmd("Synchronous half bridge", -960, -960, 11),
        *inst_cmd("simvis_switch", "S_HI", -820, -700, element_value(refs.get("S_HI"), "swhi"), "vin -> sw, ctrl/ramp"),
        *inst_cmd("simvis_switch", "S_LO", -820, -240, element_value(refs.get("S_LO"), "swlo"), "sw -> 0, ramp/ctrl"),
        *inst_cmd("simvis_diode", "D_HI", -340, -700, "body_diode", "sw to vin"),
        *inst_cmd("simvis_diode", "D_LO", -340, -240, "body_diode", "0 to sw"),
        label_cmd("Output filter", 120, -960, 11),
        *inst_cmd("simvis_ind", "L1", 240, -700, value_of(vars_found, "Lval", "{Lval}"), "sw to lx"),
        *inst_cmd("simvis_res", "Rl", 620, -700, value_of(vars_found, "Rl", "{Rl}"), "lx to vout"),
        *inst_cmd("simvis_res", "Rc", 980, -700, value_of(vars_found, "Rc", "{Rc}"), "ncap to vout"),
        *inst_cmd("simvis_cap", "Cout", 1340, -700, value_of(vars_found, "Cout", "{Cout}"), "ncap to 0"),
        label_cmd("Load step", 120, -240, 11),
        *inst_cmd("simvis_res", "RLOAD", 220, 80, value_of(vars_found, "Rload", "{Rload}"), "vout to 0"),
        *inst_cmd("simvis_switch", "S_LOAD", 620, 80, element_value(refs.get("S_LOAD"), "loadsw"), "load_ctl enables step load"),
        *inst_cmd("simvis_res", "RSTEP", 1080, 80, value_of(vars_found, "Rload", "{Rload}"), "nload to 0"),
        *inst_cmd("simvis_vsrc", "VLOADCTL", 1480, 80, "PUL", "load_ctl timing"),
        label_cmd("PWM and compensator", -1880, 620, 11),
        *inst_cmd("simvis_vsrc", "VRAMP", -1800, 920, f"{value_of(vars_found, 'fsw', '{fsw}')} SAW", "ramp"),
        *inst_cmd("simvis_vsrc", "VREF", -1440, 920, value_of(vars_found, "Vout", "{Vout}"), "vref"),
        *inst_cmd("simvis_block", "EERR", -1040, 920, "Vref - Vout", "err"),
        *inst_cmd("simvis_isrc", "GINT", -580, 920, f"Ki={value_of(vars_found, 'Ki', '{Ki}')}", "integrator source"),
        *inst_cmd("simvis_cap", "CINT", -220, 920, "1", "integrator state"),
        *inst_cmd("simvis_res", "RINT", 120, 920, "1G", "leak"),
        *inst_cmd("simvis_isrc", "GEF", 500, 920, f"Kf={value_of(vars_found, 'Kf', '{Kf}')}", "derivative-filter source"),
        *inst_cmd("simvis_cap", "CEF", 880, 920, "1", "derivative filter state"),
        *inst_cmd("simvis_block", "E_KP/E_INT/E_DER", 1280, 920, "PID sum", "ctrl"),
        *inst_cmd("simvis_res", "RCTRL", 1700, 920, "1G", "ctrl to 0"),
        *inst_cmd("simvis_vsrc", "Bgatehi", 1700, 500, "ctrl > ramp", "gate_hi"),
        *inst_cmd("simvis_vsrc", "Bgatelo", 1700, 1340, "ctrl < ramp", "gate_lo"),
        label_cmd("Full formulas are in synchronous_buck_tran.net:", -1880, 1680, 10),
        label_cmd("err=Vout_ref-Vout, int'=Ki*err, ef'=Kf*(err-ef), ctrl=Kp*err+int+Kd*Kf*(err-ef)", -1880, 1800, 10),
        label_cmd(".include synchronous_buck_tran.net", -1880, 2020, 10),
        label_cmd("Visual schematic only: the .net file is the single source of truth for simulation.", -1880, 2140, 10),
        label_cmd("Node aliases: 1=vin, 2=sw, 3=vout, 4=ncap, 5=lx, 10=vref, 11=err, 12=int, 13=ef, 16=ctrl, 17=ramp, 40=load_ctl, 41=nload", -1880, 2260, 9),
        f"SaveAs /ascii /force {script_quote(str(output_sxsch))}",
        "Quit",
    ]
    return "\n".join(lines) + "\n"


def sx_escape(text: str) -> str:
    return text.replace("\\", "/").replace('"', '\\"')


def sx_wire(x1: int, y1: int, x2: int, y2: int) -> str:
    return f"""Wire x1={x1} y1={y1} x2={x2} y2={y2}
.Wire
Attributes x1={x1} y1={y1} x2={x2} y2={y2}
Property name="StyleNormal" value="DefaultWire" visible=0
Property name="StyleSelected" value="DefaultSelected" visible=0
.EndWire
"""


def sx_text(text: str, x: int, y: int) -> str:
    text = sx_escape(text)
    return f""".TextAnnotation
Attributes orient="N0" x={x} y={y}
TextInfo text="{text}"
Property name="StyleNormal" value="DefaultAnnotation" autopos=1 normal="Left" rotated="Left" visible=0 font="Default"
Property name="StyleSelected" value="DefaultSelected" autopos=1 normal="Left" rotated="Left" visible=0 font="Default"
Property name="TextAlignment" value="AlignLeft" autopos=1 normal="Left" rotated="Left" visible=0 font="Default"
.EndTextAnnotation
--SXVERSION1.5
.Instance
Attributes type="symbol" name="Free_text" y={y} x={x} orient="N0"
Property name="value" value="{text}" x=0 y=156 align="LeftBase" selectable=1 font="FreeText"
.EndInstance
--EndSXVERSION1.5
"""


def sx_box(x: int, y: int, w: int, h: int, title: str, detail: str, pins: str = "") -> str:
    content = [
        sx_wire(x, y, x + w, y),
        sx_wire(x + w, y, x + w, y + h),
        sx_wire(x + w, y + h, x, y + h),
        sx_wire(x, y + h, x, y),
        sx_text(title, x + 60, y + 60),
    ]
    if detail:
        content.append(sx_text(detail, x + 60, y + 210))
    if pins:
        content.append(sx_text(pins, x + 60, y + 360))
    return "".join(content)


def render_readable_sxsch(parsed: ParsedNetlist) -> str:
    vars_found = parsed_vars(parsed)
    refs = element_by_ref(parsed)

    def v(name: str, fallback: str) -> str:
        return value_of(vars_found, name, fallback)

    def ev(ref: str, fallback: str = "") -> str:
        return element_value(refs.get(ref.upper()), fallback)

    pieces: list[str] = [
        "SIMetrixFile type=schematic format=1.0 revision=9\n",
        ".Component\n",
        ".Schematic\n",
        ".SymbolLibrary\n",
        ".EndSymbolLibrary\n",
        ".StyleLibrary\n",
        ".EndStyleLibrary\n",
        sx_text("Synchronous Buck Converter - SIMPLIS readable schematic", -2040, -1560),
        sx_text("Visual schematic with wide spacing. Automation runs synchronous_buck_tran.net.", -2040, -1320),
        sx_text("Input", -2040, -960),
        sx_box(-2040, -720, 360, 420, "VDC", v("Vdc", "{Vdc}"), "vin to 0"),
        sx_box(-1560, -720, 360, 420, "Cin", "not in SIMPLIS netlist", "visual input bypass"),
        sx_text("Synchronous half bridge", -960, -960),
        sx_box(-960, -720, 420, 420, "S_HI", ev("S_HI", "swhi"), "vin to sw; ctrl/ramp"),
        sx_box(-960, -180, 420, 420, "S_LO", ev("S_LO", "swlo"), "sw to 0; ramp/ctrl"),
        sx_box(-420, -720, 360, 420, "D_HI", "body_diode", "sw to vin"),
        sx_box(-420, -180, 360, 420, "D_LO", "body_diode", "0 to sw"),
        sx_text("Output filter", 240, -960),
        sx_box(240, -720, 360, 420, "L1", v("Lval", "{Lval}"), "sw to lx"),
        sx_box(720, -720, 360, 420, "Rl", v("Rl", "{Rl}"), "lx to vout"),
        sx_box(1200, -720, 360, 420, "Rc", v("Rc", "{Rc}"), "ncap to vout"),
        sx_box(1680, -720, 360, 420, "Cout", v("Cout", "{Cout}"), "ncap to 0"),
        sx_text("Load step", 240, -180),
        sx_box(240, 60, 360, 420, "RLOAD", v("Rload", "{Rload}"), "vout to 0"),
        sx_box(720, 60, 420, 420, "S_LOAD", ev("S_LOAD", "loadsw"), "vout to nload; load_ctl"),
        sx_box(1260, 60, 360, 420, "RSTEP", v("Rload", "{Rload}"), "nload to 0"),
        sx_box(1740, 60, 420, 420, "VLOADCTL", "PUL", "load_ctl timing"),
        sx_text("PWM and compensator", -2040, 720),
        sx_box(-2040, 960, 360, 420, "VRAMP", f"{v('fsw', '{fsw}')} SAW", "ramp"),
        sx_box(-1560, 960, 360, 420, "VREF", v("Vout", "{Vout}"), "vref"),
        sx_box(-1080, 960, 420, 420, "EERR", "Vref - Vout", "vref, vout to err"),
        sx_box(-540, 960, 420, 420, "GINT", f"Ki={v('Ki', '{Ki}')}", "err to int"),
        sx_box(0, 960, 360, 420, "CINT", "1", "int state"),
        sx_box(480, 960, 360, 420, "RINT", "1G", "int leak"),
        sx_box(960, 960, 420, 420, "GEF", f"Kf={v('Kf', '{Kf}')}", "err-ef to ef"),
        sx_box(1500, 960, 360, 420, "CEF", "1", "ef state"),
        sx_box(1980, 960, 520, 420, "E_KP/E_INT/E_DER", "PID sum", "err, int, ef to ctrl"),
        sx_box(2640, 960, 360, 420, "RCTRL", "1G", "ctrl to 0"),
        sx_box(2640, 360, 420, 420, "Bgatehi", "ctrl > ramp", "gate_hi"),
        sx_box(2640, 1560, 420, 420, "Bgatelo", "ctrl < ramp", "gate_lo"),
        sx_text("Full formulas are in synchronous_buck_tran.net:", -2040, 2280),
        sx_text("err=Vout_ref-Vout, int'=Ki*err, ef'=Kf*(err-ef), ctrl=Kp*err+int+Kd*Kf*(err-ef)", -2040, 2520),
        sx_text(".include synchronous_buck_tran.net", -2040, 2880),
        sx_text("Visual schematic only: the .net file is the single source of truth for simulation.", -2040, 3120),
        sx_text("Node aliases: 1=vin, 2=sw, 3=vout, 4=ncap, 5=lx, 10=vref, 11=err, 12=int, 13=ef, 16=ctrl, 17=ramp, 40=load_ctl, 41=nload", -2040, 3360),
        'Text value=""\n',
        'LicenseInfo version="9.10a" feats="Schematic|SIMPLIS_IF|Scripts|Sim" serial="unknown" user="unknown" code="unknown" binarch="x64" system="WINNT" product="SIMetrix/SIMPLIS"\n',
        'SimulatorMode value="simplis"\n',
        "View x=0 y=0 snapgrid=120 offsetx=0 offsety=0 zoomlevel=31 zoom=9\n",
        'Property name="CreateProduct" value="SIMetrix/SIMPLIS" type="value"\n',
        'Property name="UserVersion" value=0 type="value"\n',
        'Property name="WriteLog" value="0/0" type="value"\n',
        ".EndSchematic\n",
        ".EndComponent\n",
    ]
    return "".join(pieces)


def find_default_simetrix_exe() -> Path | None:
    candidates: list[Path] = []
    for root in [Path("C:/Program Files"), Path("C:/Program Files (x86)")]:
        if root.exists():
            candidates.extend(root.glob("SIMetrix*/bin64/SIMetrix.exe"))
            candidates.extend(root.glob("SIMetrix*/bin/SIMetrix.exe"))
    existing = sorted((path for path in candidates if path.exists()), reverse=True)
    return existing[0] if existing else None


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8", newline="\n")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Create SIMetrix/SIMPLIS visual schematics from netlists.")
    parser.add_argument("netlist", type=Path, help="Input SIMPLIS/SPICE netlist path")
    parser.add_argument("--sxsch", type=Path, help="Output visual .sxsch path")
    parser.add_argument("--sxscr", type=Path, help="Output SIMetrix script path")
    parser.add_argument("--run", action="store_true", help="Run SIMetrix to create the .sxsch file")
    parser.add_argument("--simetrix-exe", type=Path, help="Path to SIMetrix.exe")
    parser.add_argument("--dot", type=Path, help="Optional Graphviz DOT connectivity output")
    parser.add_argument("--json", type=Path, help="Optional parsed JSON output")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    netlist = args.netlist.resolve()
    if not netlist.exists():
        raise FileNotFoundError(netlist)

    parsed = parse_netlist(netlist)
    sxsch_path = (args.sxsch or netlist.with_name("synchronous_buck.sxsch")).resolve()
    write_text(sxsch_path, render_readable_sxsch(parsed))
    outputs = [sxsch_path]

    if args.sxscr is not None:
        sxscr_path = args.sxscr.resolve()
        write_text(sxscr_path, render_readable_simetrix_script(parsed, sxsch_path))
        outputs.append(sxscr_path)
    if args.dot is not None:
        write_text(args.dot, render_dot(parsed))
        outputs.append(args.dot)
    if args.json is not None:
        write_text(args.json, render_json(parsed))
        outputs.append(args.json)

    if args.run:
        sxscr_path = (args.sxscr or sxsch_path.with_suffix(".sxscr")).resolve()
        if not sxscr_path.exists():
            write_text(sxscr_path, render_readable_simetrix_script(parsed, sxsch_path))
        simetrix_exe = args.simetrix_exe or find_default_simetrix_exe()
        if simetrix_exe is None or not simetrix_exe.exists():
            raise FileNotFoundError("SIMetrix.exe was not found. Pass --simetrix-exe.")
        completed = subprocess.run(
            [str(simetrix_exe), "/s", str(sxscr_path)],
            cwd=str(netlist.parent),
            text=True,
            capture_output=True,
            timeout=120,
        )
        if completed.returncode != 0:
            raise RuntimeError(
                f"SIMetrix returned {completed.returncode}\nSTDOUT:\n{completed.stdout}\nSTDERR:\n{completed.stderr}"
            )
        outputs.append(sxsch_path)

    print(f"Parsed {len(parsed.elements)} elements, {len(scope_net_counts(parsed.elements))} nets, {len(parsed.controls)} controls.")
    if parsed.warnings:
        print(f"Warnings: {len(parsed.warnings)}")
        for warning in parsed.warnings[:5]:
            print(f"  - {warning}")
    for output in outputs:
        print(f"Wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
