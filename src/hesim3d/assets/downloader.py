from __future__ import annotations

import hashlib
import os
from pathlib import Path
from typing import Optional, Union
import requests
from tqdm import tqdm
from .catalog import SCENE_CATALOG, SceneMetadata, get_scene_metadata


def get_cache_dir() -> Path:
    """Return local user cache directory for 3D simulation assets."""
    cache_base = os.environ.get("HESIM3D_CACHE_DIR")
    if cache_base:
        p = Path(cache_base)
    else:
        p = Path.home() / ".cache" / "hesim3d" / "scenes"
    p.mkdir(parents=True, exist_ok=True)
    return p


def compute_sha256(file_path: Union[str, Path]) -> str:
    """Calculate SHA-256 hex digest of a local file."""
    hasher = hashlib.sha256()
    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            hasher.update(chunk)
    return hasher.hexdigest()


from concurrent.futures import ThreadPoolExecutor


def download_scene(
    scene_id_or_meta: Union[str, SceneMetadata],
    force: bool = False,
    show_progress: bool = True,
) -> Path:
    """
    Download or resolve a 3D scene asset on demand with progress reporting and checksum verification.
    Supports both single-file (.glb) and directory multi-file (.gltf) assets.
    """
    if isinstance(scene_id_or_meta, str):
        meta = get_scene_metadata(scene_id_or_meta)
    else:
        meta = scene_id_or_meta

    # 1. Built-in scene resolution
    if meta.built_in:
        repo_root = Path(__file__).resolve().parent.parent.parent.parent
        if meta.local_rel_path:
            local_p = repo_root / meta.local_rel_path
            if local_p.exists():
                return local_p

    # 2. Check cache directory
    cache_dir = get_cache_dir()

    # Handle multi-file glTF scenes (e.g. Sponza Atrium)
    if meta.format == "gltf" or "contents" in meta.url:
        target_dir = cache_dir / meta.id
        target_dir.mkdir(parents=True, exist_ok=True)

        for candidate in target_dir.glob("*.gltf"):
            if candidate.exists() and not force:
                return candidate

        headers = {"User-Agent": "hesim3d"}
        response = requests.get(meta.url, headers=headers, timeout=30)
        response.raise_for_status()
        items = response.json()

        total_bytes = sum(item.get("size", 0) for item in items if item.get("type") == "file")

        with tqdm(
            desc=f"Downloading {meta.name}",
            total=total_bytes,
            unit="iB",
            unit_scale=True,
            unit_divisor=1024,
            disable=not show_progress,
        ) as bar:
            def download_one(item: dict) -> None:
                if item.get("type") != "file":
                    return
                dst = target_dir / item["name"]
                if dst.exists() and not force and dst.stat().st_size == item["size"]:
                    bar.update(item["size"])
                    return
                r = requests.get(item["download_url"], headers=headers, timeout=60)
                r.raise_for_status()
                with open(dst, "wb") as f:
                    f.write(r.content)
                bar.update(len(r.content))

            with ThreadPoolExecutor(max_workers=8) as pool:
                list(pool.map(download_one, items))

        for candidate in target_dir.glob("*.gltf"):
            return candidate
        return target_dir / f"{meta.id}.gltf"

    # Single-file asset (.glb)
    target_path = cache_dir / f"{meta.id}.{meta.format}"

    if target_path.exists() and not force:
        if meta.sha256:
            current_sha = compute_sha256(target_path)
            if current_sha == meta.sha256:
                return target_path
        else:
            return target_path

    # 3. Stream download with tqdm progress
    if not meta.url:
        raise ValueError(f"Scene '{meta.id}' has no download URL configured.")

    temp_path = cache_dir / f"{meta.id}.tmp"
    response = requests.get(meta.url, stream=True, timeout=30)
    response.raise_for_status()

    total_size = int(response.headers.get("content-length", 0))

    with open(temp_path, "wb") as f, tqdm(
        desc=f"Downloading {meta.name}",
        total=total_size,
        unit="iB",
        unit_scale=True,
        unit_divisor=1024,
        disable=not show_progress,
    ) as bar:
        for chunk in response.iter_content(chunk_size=8192):
            size = f.write(chunk)
            bar.update(size)

    # 4. Verify checksum
    if meta.sha256:
        downloaded_sha = compute_sha256(temp_path)
        if downloaded_sha != meta.sha256:
            temp_path.unlink(missing_ok=True)
            raise ValueError(
                f"Checksum mismatch for '{meta.id}': expected {meta.sha256}, got {downloaded_sha}"
            )

    temp_path.replace(target_path)
    return target_path


def resolve_scene_path(scene_identifier: str) -> Path:
    """
    Resolve a scene identifier to an absolute path on disk.
    Can be a catalog ID (e.g. 'sponza', 'checkerboard_room') or a direct local file path.
    """
    p = Path(scene_identifier)
    if p.exists():
        return p.resolve()

    scene_key = scene_identifier.strip().lower()
    if scene_key in SCENE_CATALOG:
        return download_scene(scene_key)

    raise FileNotFoundError(
        f"Could not find or resolve scene '{scene_identifier}'. Check file path or catalog ID."
    )
