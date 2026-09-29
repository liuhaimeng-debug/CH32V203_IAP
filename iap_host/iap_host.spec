# -*- mode: python ; coding: utf-8 -*-
"""
CH32V203 IAP 上位机 PyInstaller 打包脚本
========================================
生成单文件可执行 exe（Windows）。

构建方式（在 iap_host/ 目录下）：
    python -m PyInstaller iap_host.spec

产物：
    dist/iap_host.exe
"""

block_cipher = None

a = Analysis(
    ['iap_host.py'],
    pathex=[],
    binaries=[],
    datas=[],
    hiddenimports=['serial', 'serial.tools', 'serial.tools.list_ports'],
    hookspath=[],
    hooks_extras=[],
    runtime_hooks=[],
    excludes=[],
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
    name='iap_host',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,           # GUI 程序，不弹控制台黑窗
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=None,
)
