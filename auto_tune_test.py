import os
import tempfile
import subprocess
import sys
import unittest
from pathlib import Path

from auto_tune import (
    AutoTuner,
    CompensatorDesign,
    GridRefinePidTuner,
    PlecsModelEditor,
    ResponseAnalyzer,
    TuningConfig,
    TuningResult,
    select_best_result,
)
from gui import validate_config_values
from ltspice_backend import LtspiceRawParser, LtspiceSimulationRunner, analytic_loop_gain, normalize_backend
from simplis_backend import SimplisExportParser, SimplisSimulationRunner


class CompensatorDesignTests(unittest.TestCase):
    def test_reference_design_matches_known_values(self):
        kp, ki, kd, kf = CompensatorDesign().reference_design()

        self.assertAlmostEqual(kp, 0.31703, places=5)
        self.assertAlmostEqual(ki, 3764.63, places=2)
        self.assertAlmostEqual(kd, 4.785e-6, places=8)
        self.assertAlmostEqual(kf, 551785.74, places=2)

    def test_kd_is_never_negative_near_low_margin_edge(self):
        cfg = TuningConfig()
        _, _, kd, _ = CompensatorDesign().compute(cfg.wc_min, cfg.phi_m_min)

        self.assertGreaterEqual(kd, 0.0)


class ResponseAnalyzerTests(unittest.TestCase):
    def test_flat_response_has_no_transient_error(self):
        analyzer = ResponseAnalyzer(expected_step_times=[0.001])
        data = [[i * 1e-6, 1.0, 5.0] for i in range(2000)]

        overshoot, undershoot, osc_count, settling_time = analyzer.analyze(
            ["Time", "IL", "Vout"],
            data,
        )

        self.assertEqual(overshoot, 0.0)
        self.assertEqual(undershoot, 0.0)
        self.assertEqual(osc_count, 0)
        self.assertLessEqual(settling_time, analyzer.transient_window)


class GridRefinePidTunerTests(unittest.TestCase):
    def test_first_adjust_moves_from_bootstrap_to_coarse_grid(self):
        tuner = GridRefinePidTuner(TuningConfig())
        initial = tuner.get_initial_params()
        next_params = tuner.adjust(*initial, overshoot=10.0, undershoot=2.0, osc_count=1, settling_time=0.001)

        self.assertEqual(tuner.phase, "coarse_grid")
        self.assertNotEqual(next_params, initial)


class BestResultSelectionTests(unittest.TestCase):
    def test_best_result_prefers_pass_with_lowest_os_plus_us(self):
        results = [
            TuningResult(0, 0.1, 1.0, 0.0, 10.0, 0.2, 0.2, 1, 0.001, "FAIL"),
            TuningResult(1, 0.1, 1.0, 0.0, 10.0, 2.0, 1.5, 0, 0.002, "PASS"),
            TuningResult(2, 0.1, 1.0, 0.0, 10.0, 1.0, 1.0, 0, 0.003, "PASS"),
        ]

        best = select_best_result(results)

        self.assertIsNotNone(best)
        self.assertEqual(best.iter_num, 2)

    def test_best_result_falls_back_to_fail_ranking_when_no_pass_exists(self):
        results = [
            TuningResult(0, 0.1, 1.0, 0.0, 10.0, 3.0, 4.0, 1, 0.001, "FAIL"),
            TuningResult(1, 0.1, 1.0, 0.0, 10.0, 2.0, 2.0, 2, 0.002, "FAIL"),
        ]

        best = select_best_result(results)

        self.assertIsNotNone(best)
        self.assertEqual(best.iter_num, 1)


class PlecsModelEditorTests(unittest.TestCase):
    @staticmethod
    def _seed_old_run_dirs(root: Path, count: int = 20) -> None:
        root.mkdir(parents=True, exist_ok=True)
        base_time = 1_700_000_000
        for idx in range(count):
            run_dir = root / f"run_20200101_0000{idx:02d}"
            run_dir.mkdir()
            (run_dir / "marker.txt").write_text(str(idx), encoding="utf-8")
            os.utime(run_dir, (base_time + idx, base_time + idx))

    def test_prepare_working_model_patches_copy_only(self):
        source_text = """
Plecs {
  Analysis {
    Name "Loop Gain (Frequency Response)"
    FrequencyRange "[1 2]"
    NumPoints "3"
    ExtractionCycles "4"
  }
  Analysis {
    Name "Loop Gain (Peak Dense)"
    NumPoints "5"
    ExtractionCycles "6"
  }
}
"""
        cfg = TuningConfig(
            plecs_exe="",
            bode_freq_start_hz=100.0,
            bode_freq_stop_hz=200000.0,
            bode_coarse_num_points=51,
            bode_dense_num_points=71,
            bode_extraction_cycles=30,
        )

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            source_path = tmp_path / "source.plecs"
            work_dir = tmp_path / "work"
            source_path.write_text(source_text, encoding="utf-8")

            work_model = PlecsModelEditor().prepare_working_model(str(source_path), str(work_dir), cfg)

            self.assertEqual(source_path.read_text(encoding="utf-8"), source_text)
            patched = work_model.read_text(encoding="utf-8")
            self.assertIn('FrequencyRange "[100 200000]"', patched)
            self.assertIn('NumPoints "51"', patched)
            self.assertIn('NumPoints "71"', patched)
            self.assertIn('ExtractionCycles "30"', patched)

    def test_plecs_work_dir_keeps_only_twenty_runs(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            source_path = tmp_path / "source.plecs"
            work_dir = tmp_path / "work"
            source_path.write_text("Plecs {}", encoding="utf-8")
            self._seed_old_run_dirs(work_dir)

            cfg = TuningConfig(plecs_exe="", run_bode_analysis=False)
            work_model = PlecsModelEditor().prepare_working_model(str(source_path), str(work_dir), cfg)

            run_dirs = sorted(path.name for path in work_dir.iterdir() if path.is_dir() and path.name.startswith("run_"))
            self.assertEqual(len(run_dirs), 20)
            self.assertNotIn("run_20200101_000000", run_dirs)
            self.assertTrue(work_model.exists())

    def test_ltspice_work_dir_keeps_only_twenty_runs(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            source_dir = tmp_path / "source"
            source_dir.mkdir()
            asc = source_dir / "model.asc"
            tran = source_dir / "tran.cir"
            bode_ac = source_dir / "bode_ac.cir"
            asc.write_text("Version 4\n", encoding="utf-8")
            tran.write_text(".tran 1m\n.end\n", encoding="utf-8")
            bode_ac.write_text(".ac dec 10 1 1k\n.end\n", encoding="utf-8")
            work_dir = tmp_path / "work"
            self._seed_old_run_dirs(work_dir)

            cfg = TuningConfig(
                sim_backend="ltspice",
                ltspice_exe="",
                ltspice_asc_model=str(asc),
                ltspice_netlist_model=str(tran),
                ltspice_bode_ac_netlist_model=str(bode_ac),
                ltspice_bode_mode="ac",
                ltspice_work_dir=str(work_dir),
            )
            work_netlist = LtspiceSimulationRunner(cfg).prepare_working_model()

            run_dirs = sorted(path.name for path in work_dir.iterdir() if path.is_dir() and path.name.startswith("run_"))
            self.assertEqual(len(run_dirs), 20)
            self.assertNotIn("run_20200101_000000", run_dirs)
            self.assertTrue(work_netlist.exists())

    def test_simplis_work_dir_keeps_only_twenty_runs(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            schematic = tmp_path / "model.sxsch"
            netlist = tmp_path / "model.net"
            schematic.write_text("SIMetrixFile type=schematic\n", encoding="utf-8")
            netlist.write_text(".VAR Kp=1\n.END\n", encoding="utf-8")
            work_dir = tmp_path / "work"
            self._seed_old_run_dirs(work_dir)

            cfg = TuningConfig(
                sim_backend="simplis",
                simplis_schematic_model=str(schematic),
                simplis_netlist_model=str(netlist),
                simplis_work_dir=str(work_dir),
            )
            work_netlist = SimplisSimulationRunner(cfg).prepare_working_model()

            run_dirs = sorted(path.name for path in work_dir.iterdir() if path.is_dir() and path.name.startswith("run_"))
            self.assertEqual(len(run_dirs), 20)
            self.assertNotIn("run_20200101_000000", run_dirs)
            self.assertTrue(work_netlist.exists())

    def test_ltspice_setup_reuses_work_dir_for_same_history(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            source_dir = tmp_path / "source"
            source_dir.mkdir()
            asc = source_dir / "model.asc"
            tran = source_dir / "tran.cir"
            bode_ac = source_dir / "bode_ac.cir"
            asc.write_text("Version 4\n", encoding="utf-8")
            tran.write_text(".tran 1m\n.end\n", encoding="utf-8")
            bode_ac.write_text(".ac dec 10 1 1k\n.end\n", encoding="utf-8")
            work_dir = tmp_path / "work"
            cfg = TuningConfig(
                sim_backend="ltspice",
                ltspice_exe="",
                ltspice_asc_model=str(asc),
                ltspice_netlist_model=str(tran),
                ltspice_bode_ac_netlist_model=str(bode_ac),
                ltspice_bode_mode="ac",
                ltspice_work_dir=str(work_dir),
            )
            tuner = AutoTuner(cfg)

            tuner.setup()
            first_path = tuner.work_model_path
            tuner.setup(reuse_work_model=True)

            run_dirs = [path for path in work_dir.iterdir() if path.is_dir() and path.name.startswith("run_")]
            self.assertEqual(len(run_dirs), 1)
            self.assertEqual(tuner.work_model_path, first_path)

    def test_ltspice_first_single_iteration_creates_new_history_folder(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            source_dir = tmp_path / "source"
            source_dir.mkdir()
            asc = source_dir / "model.asc"
            tran = source_dir / "tran.cir"
            bode_ac = source_dir / "bode_ac.cir"
            asc.write_text("Version 4\n", encoding="utf-8")
            tran.write_text(".tran 1m\n.end\n", encoding="utf-8")
            bode_ac.write_text(".ac dec 10 1 1k\n.end\n", encoding="utf-8")
            work_dir = tmp_path / "work"
            cfg = TuningConfig(
                sim_backend="ltspice",
                ltspice_exe="",
                ltspice_asc_model=str(asc),
                ltspice_netlist_model=str(tran),
                ltspice_bode_ac_netlist_model=str(bode_ac),
                ltspice_bode_mode="ac",
                ltspice_work_dir=str(work_dir),
            )
            tuner = AutoTuner(cfg)

            tuner.setup()
            first_path = tuner.work_model_path
            tuner.setup(reuse_work_model=False)

            run_dirs = [path for path in work_dir.iterdir() if path.is_dir() and path.name.startswith("run_")]
            self.assertEqual(len(run_dirs), 2)
            self.assertNotEqual(tuner.work_model_path, first_path)

    def test_simplis_setup_reuses_work_dir_for_same_history(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            schematic = tmp_path / "model.sxsch"
            netlist = tmp_path / "model.net"
            schematic.write_text("SIMetrixFile type=schematic\n", encoding="utf-8")
            netlist.write_text(".VAR Kp=1\n.END\n", encoding="utf-8")
            work_dir = tmp_path / "work"
            cfg = TuningConfig(
                sim_backend="simplis",
                simplis_schematic_model=str(schematic),
                simplis_netlist_model=str(netlist),
                simplis_work_dir=str(work_dir),
            )
            tuner = AutoTuner(cfg)

            tuner.setup()
            first_path = tuner.work_model_path
            tuner.setup(reuse_work_model=True)

            run_dirs = [path for path in work_dir.iterdir() if path.is_dir() and path.name.startswith("run_")]
            self.assertEqual(len(run_dirs), 1)
            self.assertEqual(tuner.work_model_path, first_path)


class GuiValidationTests(unittest.TestCase):
    def test_bode_stop_must_be_greater_than_start(self):
        with tempfile.TemporaryDirectory() as tmp:
            model_path = Path(tmp) / "model.plecs"
            model_path.write_text("Plecs {}", encoding="utf-8")
            cfg = TuningConfig(
                plecs_exe="",
                plecs_model=str(model_path),
                run_bode_analysis=True,
                bode_freq_start_hz=1000.0,
                bode_freq_stop_hz=1000.0,
            )

            errors = validate_config_values(cfg)

        self.assertTrue(any("Stop f" in err for err in errors))

    def test_valid_minimal_config_has_no_errors(self):
        with tempfile.TemporaryDirectory() as tmp:
            model_path = Path(tmp) / "model.plecs"
            model_path.write_text("Plecs {}", encoding="utf-8")
            cfg = TuningConfig(
                plecs_exe="",
                plecs_model=str(model_path),
                run_bode_analysis=False,
            )

            errors = validate_config_values(cfg)

        self.assertEqual(errors, [])

    def test_ltspice_validation_reports_missing_executable(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            asc = tmp_path / "model.asc"
            tran = tmp_path / "model.cir"
            bode = tmp_path / "bode.cir"
            bode_ac = tmp_path / "bode_ac.cir"
            for path in (asc, tran, bode, bode_ac):
                path.write_text("* model\n.end\n", encoding="utf-8")
            cfg = TuningConfig(
                sim_backend="ltspice",
                ltspice_exe=str(tmp_path / "missing.exe"),
                ltspice_asc_model=str(asc),
                ltspice_netlist_model=str(tran),
                ltspice_bode_netlist_model=str(bode),
                ltspice_bode_ac_netlist_model=str(bode_ac),
            )

            errors = validate_config_values(cfg)

        self.assertTrue(any("LTspice executable" in err for err in errors))


class LtspiceBackendTests(unittest.TestCase):
    def test_backend_normalization_accepts_ltspice_alias(self):
        self.assertEqual(normalize_backend("LT-Spice"), "ltspice")
        self.assertEqual(normalize_backend("PLECS"), "plecs")
        self.assertEqual(normalize_backend("SIMetrix-SIMPLIS"), "simplis")

    def test_default_ltspice_templates_exist(self):
        cfg = TuningConfig()
        self.assertTrue(Path(cfg.ltspice_asc_model).exists())
        self.assertTrue(Path(cfg.ltspice_netlist_model).exists())
        self.assertTrue(Path(cfg.ltspice_bode_netlist_model).exists())
        self.assertTrue(Path(cfg.ltspice_bode_ac_netlist_model).exists())

    def test_trace_name_matching_is_case_insensitive(self):
        names = ["time", "V(vout)", "I(L1)"]
        self.assertEqual(LtspiceRawParser.find_trace_name(names, ["v(vout)"], ["vout"]), "V(vout)")
        self.assertEqual(LtspiceRawParser.find_trace_name(names, ["i(l1)"], ["l1"]), "I(L1)")

    def test_synthetic_zero_initial_sample_is_replaced(self):
        time_vals = [0.0, 1e-13]
        vout_vals = [0.0, 5.0]
        il_vals = [0.0, 1.0]
        if abs(time_vals[0]) < 1e-18 and abs(vout_vals[0]) < 1e-9 and abs(vout_vals[1]) > 1.0:
            vout_vals[0] = vout_vals[1]
            il_vals[0] = il_vals[1]

        self.assertEqual(vout_vals[0], 5.0)
        self.assertEqual(il_vals[0], 1.0)

    def test_analytic_loop_gain_returns_complex_response(self):
        response = analytic_loop_gain(0.166507, 1419.38, 1.586e-6, 156357, [1000.0, 10000.0])

        self.assertEqual(len(response), 2)
        self.assertTrue(all(isinstance(value, complex) for value in response))


class SimplisBackendTests(unittest.TestCase):
    def test_default_simplis_templates_exist(self):
        cfg = TuningConfig()
        self.assertTrue(Path(cfg.simplis_schematic_model).exists())
        self.assertTrue(Path(cfg.simplis_netlist_model).exists())

    def test_default_simplis_netlist_is_closed_loop_switching_model(self):
        cfg = TuningConfig()
        text = Path(cfg.simplis_netlist_model).read_text(encoding="utf-8")

        self.assertIn("S_HI 1 2 16 17 swhi", text)
        self.assertIn("S_LO 2 0 17 16 swlo", text)
        self.assertIn("VRAMP 17 0 SAW", text)
        self.assertIn("GINT 0 12 11 0 {Ki}", text)
        self.assertIn("GEF 0 13 11 13 {Kf}", text)
        self.assertNotIn("VSW 2 0 PUL", text)

    def test_simplis_validation_reports_missing_executable(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            schematic = tmp_path / "model.sxsch"
            netlist = tmp_path / "model.net"
            schematic.write_text("SIMetrixFile type=schematic\n", encoding="utf-8")
            netlist.write_text(".SIMULATOR SIMPLIS\n.END\n", encoding="utf-8")
            cfg = TuningConfig(
                sim_backend="simplis",
                simplis_exe=str(tmp_path / "missing.exe"),
                simplis_schematic_model=str(schematic),
                simplis_netlist_model=str(netlist),
                run_bode_analysis=True,
            )

            errors = validate_config_values(cfg)

        self.assertTrue(any("SIMetrix/SIMPLIS executable" in err for err in errors))
        self.assertFalse(any("Bode" in err for err in errors))

    def test_simplis_export_parser_accepts_csv(self):
        header, rows = SimplisExportParser.parse_text(
            "Time,IL,Vout\n"
            "0,1.0,5.0\n"
            "1e-6,1.1,5.01\n"
        )

        self.assertEqual(header, ["Time", "IL", "Vout"])
        self.assertEqual(rows[1], [1e-6, 1.1, 5.01])

    def test_simplis_export_parser_accepts_simetrix_show_text(self):
        header, rows = SimplisExportParser.parse_text(
            "time          Vec('#VOUT')   Vec('L1#P')\n"
            "0             5              1\n"
            "1e-06         5.00939134     1.23135967\n"
        )

        self.assertEqual(header, ["Time", "IL", "Vout"])
        self.assertEqual(rows[1], [1e-6, 1.23135967, 5.00939134])

    def test_simplis_runner_writes_script_and_reports_missing_export(self):
        class NoOutputRunner(SimplisSimulationRunner):
            def _run_simetrix_script(self, script_path: Path) -> subprocess.CompletedProcess[str]:
                return subprocess.CompletedProcess([str(script_path)], 0, "", "")

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            schematic = tmp_path / "model.sxsch"
            netlist = tmp_path / "model.net"
            schematic.write_text("SIMetrixFile type=schematic\n", encoding="utf-8")
            netlist.write_text(".VAR Kp=1\n.VAR Ki=2\n.VAR Kd=3\n.VAR Kf=4\n.END\n", encoding="utf-8")
            cfg = TuningConfig(
                sim_backend="simplis",
                simplis_exe=sys.executable,
                simplis_schematic_model=str(schematic),
                simplis_netlist_model=str(netlist),
                simplis_work_dir=str(tmp_path / "work"),
            )
            runner = NoOutputRunner(cfg)

            with self.assertRaisesRegex(RuntimeError, "export was not created"):
                runner.run_transient(0, 0.5, 6.0, 7e-6, 800.0)

            self.assertIsNotNone(runner.work_netlist_path)
            self.assertIsNotNone(runner.script_path)
            patched = runner.work_netlist_path.read_text(encoding="utf-8")
            script = runner.script_path.read_text(encoding="utf-8")
            self.assertIn(".VAR Kp=0.5", patched)
            self.assertIn("PreProcessNetlist /simulator SIMPLIS", script)
            self.assertIn("RunSIMPLIS", script)


if __name__ == "__main__":
    unittest.main()
