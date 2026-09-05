from pathlib import Path
from typing import Union
from .catalog import SCENE_CATALOG, SceneMetadata, list_available_scenes, get_scene_metadata
from .downloader import get_cache_dir, download_scene, resolve_scene_path


def get_asset_path(subpath: Union[str, Path] = "") -> Path:
    """
    Resolve path to packaged asset data or local development assets.
    Prioritizes the installed package directory (hesim3d/assets_data),
    falling back to local development repository structure (repo_root/assets).
    """
    sub_str = str(subpath).lstrip("/")
    # 1. Check inside installed package directory (wheel/site-packages)
    pkg_assets = Path(__file__).resolve().parent.parent / "assets_data"
    if pkg_assets.exists():
        p = pkg_assets / sub_str if sub_str else pkg_assets
        if p.exists():
            return p

    # 2. Check repository root (development / editable mode)
    repo_assets = Path(__file__).resolve().parent.parent.parent.parent / "assets"
    if repo_assets.exists():
        p = repo_assets / sub_str if sub_str else repo_assets
        if p.exists():
            return p

    # Default fallback to pkg_assets path
    return pkg_assets / sub_str if sub_str else pkg_assets


__all__ = [
    "SCENE_CATALOG",
    "SceneMetadata",
    "list_available_scenes",
    "get_scene_metadata",
    "get_cache_dir",
    "download_scene",
    "resolve_scene_path",
    "get_asset_path",
]

