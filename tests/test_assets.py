from pathlib import Path
import pytest
from hesim3d.assets import (
    SCENE_CATALOG,
    list_available_scenes,
    get_scene_metadata,
    resolve_scene_path,
)


def test_catalog_listing():
    scenes = list_available_scenes()
    assert len(scenes) >= 3
    ids = [s.id for s in scenes]
    assert "checkerboard_room" in ids
    assert "sponza" in ids
    assert "cornell_box" in ids


def test_builtin_scene_resolution():
    path = resolve_scene_path("checkerboard_room")
    assert path.exists()
    assert path.suffix == ".glb"
    assert path.stat().st_size > 0


def test_invalid_scene_raises():
    with pytest.raises(FileNotFoundError):
        resolve_scene_path("non_existent_scene_xyz")
