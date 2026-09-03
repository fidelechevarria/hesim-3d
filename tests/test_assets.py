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
    assert len(scenes) >= 4
    ids = [s.id for s in scenes]
    assert "chessboard" in ids
    assert "checkerboard_room" in ids
    assert "sponza" in ids
    assert "cornell_box" in ids

    chess_meta = get_scene_metadata("chessboard")
    assert chess_meta.format == "glb"
    assert chess_meta.size_mb > 40.0
    assert "ABeautifulGame.glb" in chess_meta.url
    assert chess_meta.sha256 == "bd7133b4b322aae97c589b8839dae8155ad2546acb35ae32a127e722a959d007"


def test_alias_resolution():
    meta1 = get_scene_metadata("chessboard")
    meta2 = get_scene_metadata("a_beautiful_game")
    meta3 = get_scene_metadata("chess")
    assert meta1.id == meta2.id == meta3.id == "chessboard"


def test_builtin_scene_resolution():
    path = resolve_scene_path("checkerboard_room")
    assert path.exists()
    assert path.suffix == ".glb"
    assert path.stat().st_size > 0


def test_invalid_scene_raises():
    with pytest.raises(FileNotFoundError):
        resolve_scene_path("non_existent_scene_xyz")


def test_scene_bounds_estimation():
    from hesim3d._hesim3d_core import FilamentRenderer
    path = resolve_scene_path("checkerboard_room")
    renderer = FilamentRenderer(320, 240, "opengl")
    loaded = renderer.load_scene(str(path))
    assert loaded is True
    bounds = renderer.get_scene_bounds()
    assert bounds.valid is True
    assert bounds.radius > 0.1
    assert bounds.extent[0] > 0.1
    assert bounds.extent[1] > 0.1
    assert bounds.extent[2] > 0.1
