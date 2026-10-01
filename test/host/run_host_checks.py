"""Run the production math/protocol/audio/settings modules without flashing a board."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source_root = root / "src"
test_root = Path(__file__).resolve().parent
sources = [source_root / name for name in (
    "sensor_fusion.cpp", "shared_state.cpp", "ble_protocol.cpp", "buzzer.cpp", "audio_pcm.cpp", "generated/voice_prompts.cpp", "tunables.cpp", "flight_state.cpp", "xctrack_protocol.cpp"
)]
with tempfile.TemporaryDirectory(prefix="vario-cpp-checks-") as directory:
    out = Path(directory)
    test = out / "firmware_checks.cpp"
    shutil.copyfile(test_root / "firmware_checks.cpp.in", test)
    sources.append(test)
    xc_test = out / "xctrack_checks.cpp"
    shutil.copyfile(test_root / "xctrack_checks.cpp.in", xc_test)
    sources.append(xc_test)
    stability_test = out / "stability_checks.cpp"
    shutil.copyfile(test_root / "stability_checks.cpp.in", stability_test)
    sources.append(stability_test)
    if os.name == "nt":
        vswhere = Path(os.environ["ProgramFiles(x86)"]) / "Microsoft Visual Studio/Installer/vswhere.exe"
        vs = Path(subprocess.check_output([str(vswhere), "-latest", "-products", "*", "-requires",
            "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"], text=True).strip())
        vc = sorted((vs / "VC/Tools/MSVC").iterdir())[-1]
        kits = Path(os.environ["ProgramFiles(x86)"]) / "Windows Kits/10"
        sdk = sorted((kits / "Include").iterdir())[-1].name
        binary = vc / "bin/Hostx64/x64"
        env = os.environ.copy()
        env["PATH"] = str(binary) + os.pathsep + env["PATH"]
        includes = [test_root / "stubs", source_root, vc / "include"] + [kits / "Include" / sdk / x for x in ("ucrt", "shared", "um")]
        libs = [vc / "lib/x64"] + [kits / "Lib" / sdk / x / "x64" for x in ("ucrt", "um")]
        executable = out / "firmware_checks.exe"
        command = [str(binary / "cl.exe"), "/nologo", "/EHsc", "/std:c++17", "/W4", "/DNOMINMAX"]
        command += ["/I" + str(p) for p in includes] + [str(p) for p in sources]
        command += ["/Fo" + str(out) + "\\", "/Fe" + str(executable), "/link"] + ["/LIBPATH:" + str(p) for p in libs]
    else:
        executable = out / "firmware_checks"
        env = None
        command = ["c++", "-std=c++17", "-Wall", "-Wextra", "-I" + str(test_root / "stubs"), "-I" + str(source_root)]
        command += [str(p) for p in sources] + ["-o", str(executable)]
    subprocess.run(command, cwd=out, env=env, check=True)
    subprocess.run([str(executable)], cwd=out, check=True)
