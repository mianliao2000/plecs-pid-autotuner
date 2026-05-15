from pathlib import Path
import tempfile
import unittest

from simplis_netlist_converter import (
    parse_netlist,
    render_dot,
    render_json,
    render_readable_simetrix_script,
)


class SimplisNetlistConverterTests(unittest.TestCase):
    def parse_text(self, text: str):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "fixture.net"
            path.write_text(text, encoding="utf-8")
            return parse_netlist(path)

    def test_parses_top_level_elements_and_controls(self):
        parsed = self.parse_text(
            """
            * comment
            .SIMULATOR SIMPLIS
            .VAR RVAL={10}
            R1 1 0 {RVAL}
            V1 1 0 DC 12
            .TRAN 1m 0
            .END
            """
        )

        self.assertEqual([element.ref for element in parsed.elements], ["R1", "V1"])
        self.assertEqual(parsed.elements[0].nodes, ["1", "GND"])
        self.assertEqual(parsed.elements[1].value, "DC")
        self.assertEqual(len(parsed.controls), 4)

    def test_handles_continuation_lines_and_subcircuit_scope(self):
        parsed = self.parse_text(
            """
            .SUBCKT AMP IN OUT GND
            XU1 IN OUT GND MYCELL VARS:
            + GAIN={2}
            .ENDS
            XTOP 1 2 0 AMP PARAMS: R={1k}
            .END
            """
        )

        self.assertIn("AMP", parsed.scopes())
        self.assertEqual(parsed.subcircuits["AMP"].pins, ["IN", "OUT", "GND"])
        inner = parsed.elements_in_scope("AMP")[0]
        outer = parsed.elements_in_scope("TOP")[0]
        self.assertEqual(inner.ref, "XU1")
        self.assertEqual(inner.value, "MYCELL")
        self.assertEqual(inner.params, ["VARS:", "GAIN={2}"])
        self.assertEqual(outer.value, "AMP")
        self.assertEqual(outer.params, ["PARAMS:", "R={1k}"])

    def test_renderers_include_core_content(self):
        parsed = self.parse_text("R1 A 0 10\nC1 A 0 1u\n.END\n")

        self.assertIn("R1", render_dot(parsed))
        self.assertIn('"ref": "R1"', render_json(parsed))

    def test_simetrix_script_saves_sxsch(self):
        parsed = self.parse_text("R1 A 0 10\nC1 A 0 1u\n.END\n")
        script = render_readable_simetrix_script(parsed, Path("out.sxsch"))

        self.assertIn("NewSchem /simulator SIMPLIS", script)
        self.assertIn("SaveAs /ascii /force", script)


if __name__ == "__main__":
    unittest.main()
