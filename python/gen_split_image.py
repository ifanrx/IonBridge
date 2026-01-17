import argparse
import importlib.util
import types
from pathlib import Path

script_dir = Path(__file__).absolute().parent


def import_dependency_script() -> types.ModuleType:
    project_dir = script_dir.parent
    dependency_script = project_dir.joinpath(
        "managed_components", "espressif__esp_mmap_assets", "spiffs_assets_gen.py",
    )
    if not dependency_script.exists():
        msg = f"Dependency script not found: {dependency_script}"
        raise FileNotFoundError(msg)
    module_name = "assets_gen"
    spec = importlib.util.spec_from_file_location(module_name, str(dependency_script))
    if spec is None or spec.loader is None:
        msg = f"Could not load module: {module_name}"
        raise ImportError(msg)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> types.NoneType:
    parser = argparse.ArgumentParser(description="Convert jpg/png to sjpg/spng/sqoi")
    parser.add_argument("--input", required=True, help="Input file directory")
    parser.add_argument("--height", required=True, help="Split height")
    parser.add_argument("--output", required=True, help="Output file path")
    parser.add_argument("--qoi", action="store_true", help="Convert to QOI format")
    args = parser.parse_args()

    output_dir = Path(args.output).absolute()
    output_dir.mkdir(parents=True, exist_ok=True)

    module = import_dependency_script()
    copy_config = module.AssetCopyConfig(
        assets_path=args.input,
        target_path=str(output_dir),
        spng_enable=True,
        sjpg_enable=True,
        qoi_enable=False,
        sqoi_enable=False,
        row_enable=False,
        support_format=[".png", ".jpg"],
        split_height=int(args.height),
    )
    if args.qoi:
        copy_config.qoi_enable = True
        copy_config.sqoi_enable = True
        copy_config.sjpg_enable = False
        copy_config.spng_enable = False
    module.copy_assets(copy_config)


if __name__ == "__main__":
    main()
