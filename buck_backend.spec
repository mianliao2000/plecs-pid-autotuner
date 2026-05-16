# -*- mode: python ; coding: utf-8 -*-

from pathlib import Path

block_cipher = None
root = Path(SPECPATH)
sim_root = root / "simulation files"

datas = [
    (str(sim_root / "plecs" / "synchronous buck.plecs"), "simulation files/plecs"),
    (str(sim_root / "plecs" / "synchronous buck.png"), "simulation files/plecs"),
    (str(sim_root / "ltspice" / "synchronous_buck.asc"), "simulation files/ltspice"),
    (str(sim_root / "ltspice" / "synchronous_buck_tran.cir"), "simulation files/ltspice"),
    (str(sim_root / "ltspice" / "synchronous_buck_bode.cir"), "simulation files/ltspice"),
    (str(sim_root / "ltspice" / "synchronous_buck_bode_ac.cir"), "simulation files/ltspice"),
    (str(sim_root / "simplis" / "synchronous_buck.sxsch"), "simulation files/simplis"),
    (str(sim_root / "simplis" / "synchronous_buck_tran.net"), "simulation files/simplis"),
]

hiddenimports = [
    "uvicorn.loops.auto",
    "uvicorn.loops.asyncio",
    "uvicorn.protocols.http.auto",
    "uvicorn.protocols.http.h11_impl",
    "uvicorn.protocols.websockets.auto",
    "uvicorn.protocols.websockets.websockets_impl",
    "uvicorn.lifespan.on",
    "websockets",
    "PyLTSpice.sim.sim_runner",
    "PyLTSpice.editor.spice_editor",
    "spicelib.raw.raw_read",
    "spicelib.sim.sim_runner",
    "spicelib.editor.spice_editor",
]

a = Analysis(
    ["web_backend.py"],
    pathex=[str(root)],
    binaries=[],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        "tkinter",
        "pytest",
        "IPython",
        "pandas",
        "scipy",
        "PyQt5",
        "spicelib.scripts",
    ],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name="buck_web_backend",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=None,
)
