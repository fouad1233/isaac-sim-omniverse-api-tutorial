#!/usr/bin/env python3
"""Generate .vscode/settings.json so Pylance can resolve isaacsim.* / omni.* imports.

Why this exists: Isaac Sim doesn't `pip install` its Python packages. Each
extension ships its own fragment of the `isaacsim` and `omni` namespace
packages under exts/<dotted.ext.name>/isaacsim/... (or .../omni/...), and
Kit's extension manager stitches ~600 of these fragments together at
*runtime* using its own import machinery -- not a static PYTHONPATH. Pylance
(and any other static analyzer) has no way to see that machinery, so
"go to definition" on anything from these libraries just fails silently,
with no error, no red squiggle, nothing to search for.

The fix is boring but exact: list every one of those fragment directories
in python.analysis.extraPaths so Pylance can merge them into the same
namespace packages Kit builds at runtime. There's no shortcut -- adding just
"exts/" doesn't work, because "exts/" itself doesn't contain an isaacsim/ or
omni/ folder, only ~600 dotted-name subfolders that each contain one.

Usage:
    python tools/setup_vscode.py /path/to/isaacsim-6.0.1

Run this once per machine, from the repo root, before lecture 1. It's
read-only against your Isaac Sim install; it only writes .vscode/settings.json
in this repo.
"""

import json
import sys
from pathlib import Path


def find_package_roots(isaac_sim_path: Path) -> list[str]:
    """Every ext/extscache subfolder that actually contributes to isaacsim/omni."""
    roots = []
    for parent_name in ("exts", "extsDeprecated", "extscache"):
        parent = isaac_sim_path / parent_name
        if not parent.is_dir():
            continue
        for ext_dir in sorted(parent.iterdir()):
            if not ext_dir.is_dir():
                continue
            if (ext_dir / "isaacsim").is_dir() or (ext_dir / "omni").is_dir():
                roots.append(str(ext_dir))
    return roots


def write_simulation_app_stub(vscode_dir: Path) -> str:
    """A one-symbol stub so `from isaacsim import SimulationApp` resolves too.

    Excluding python_packages/ (see the comment above) fixes every
    isaacsim.* submodule except this exact top-level import, because
    that file exposes SimulationApp with `AppFramework, SimulationApp =
    expose_api()` -- a runtime function call, not a static `from .x import
    y` a type checker can trace back to the real class in
    isaacsim.simulation_app.simulation_app, even when that file IS on
    extraPaths. A `.pyi` stub is the standard fix for exactly this shape
    of problem: it's a pure type-checking overlay (python.analysis.stubPath),
    resolved separately from extraPaths' namespace-package merge, so it
    can claim the single symbol `isaacsim` needs at the top level without
    reintroducing the "regular __init__.py blocks the merge" problem the
    real bootstrap file causes. Confirmed with pyright directly: without
    this stub, `from isaacsim import SimulationApp` reports "unknown
    import symbol" while every other isaacsim.* import already resolves;
    with it, zero unresolved imports across every lecture script.
    """
    stub_dir = vscode_dir / "typings" / "isaacsim"
    stub_dir.mkdir(parents=True, exist_ok=True)
    stub_path = stub_dir / "__init__.pyi"
    stub_path.write_text(
        "from isaacsim.simulation_app import AppFramework as AppFramework, SimulationApp as SimulationApp\n"
    )
    return str(vscode_dir / "typings")


def main() -> None:
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)

    isaac_sim_path = Path(sys.argv[1]).expanduser().resolve()
    python_exe = isaac_sim_path / "kit" / "python" / "bin" / "python3"
    if not python_exe.is_file():
        print(f"error: no python3 at {python_exe} -- is {isaac_sim_path} really an Isaac Sim install root?")
        sys.exit(1)

    # Fixed pieces from setup_python_env.sh's PYTHONPATH -- these are single
    # directories, not fragments, so they don't need scanning.
    #
    # Deliberately NOT included: python_packages/. It contains exactly one
    # thing -- isaacsim/__init__.py, Kit's own bootstrap loader that decides
    # which extensions to enable and wires up their sys.path entries at
    # *runtime*. That file is a real (non-namespace) package. Python's
    # import rule is: the first search-path entry that has a regular
    # __init__.py for a name wins outright and stops the namespace-package
    # merge for every other entry -- so if this directory is anywhere in
    # extraPaths, Pylance commits to this one empty-looking bootstrap file
    # as the entire "isaacsim" package and never merges in the ~600 exts/
    # fragments below that actually define isaacsim.sensors, isaacsim.core,
    # etc. Every isaacsim.* submodule import then silently fails to resolve
    # -- which is exactly the "go to definition does nothing" symptom this
    # script exists to fix. Confirmed by bisecting extraPaths with pyright
    # until this single directory was isolated as the cause.
    fixed_paths = [
        isaac_sim_path / "kit" / "python" / "lib" / "python3.12" / "site-packages",
        isaac_sim_path / "exts" / "isaacsim.simulation_app",
        isaac_sim_path / "kit" / "kernel" / "py",
        isaac_sim_path / "kit" / "bindings-python",
        isaac_sim_path / "kit" / "scripting-python-3.12" / "libs",
    ]
    fixed_paths = [str(p) for p in fixed_paths if p.is_dir()]

    fragment_paths = find_package_roots(isaac_sim_path)
    extra_paths = fixed_paths + fragment_paths

    print(f"Found {len(fragment_paths)} isaacsim/omni package fragments under {isaac_sim_path}")
    print(f"Plus {len(fixed_paths)} fixed site-packages-style directories")

    repo_root = Path(__file__).resolve().parent.parent
    vscode_dir = repo_root / ".vscode"
    vscode_dir.mkdir(exist_ok=True)
    settings_path = vscode_dir / "settings.json"

    settings = {}
    if settings_path.is_file():
        settings = json.loads(settings_path.read_text())

    stub_path = write_simulation_app_stub(vscode_dir)

    settings["python.defaultInterpreterPath"] = str(python_exe)
    settings["python.analysis.extraPaths"] = extra_paths
    settings["python.analysis.stubPath"] = stub_path
    # Kit's own packages use it; without this Pylance flags half the codebase
    # as an error before it even gets to your own files.
    settings["python.analysis.diagnosticMode"] = "openFilesOnly"

    settings_path.write_text(json.dumps(settings, indent=2) + "\n")
    print(f"Wrote {settings_path} ({len(extra_paths)} extraPaths total)")
    print(f"Wrote {stub_path}/isaacsim/__init__.pyi (fixes 'from isaacsim import SimulationApp' specifically)")
    print("Reload the VS Code window (Ctrl+Shift+P -> Developer: Reload Window) to pick it up.")


if __name__ == "__main__":
    main()
