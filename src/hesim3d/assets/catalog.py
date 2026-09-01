from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional


@dataclass
class SceneMetadata:
    """Metadata describing a 3D simulation environment."""
    id: str
    name: str
    description: str
    url: str
    sha256: str
    size_mb: float
    license: str
    attribution: str
    format: str = "glb"
    built_in: bool = False
    local_rel_path: Optional[str] = None


# Official scene benchmark catalog
SCENE_CATALOG: Dict[str, SceneMetadata] = {
    "checkerboard_room": SceneMetadata(
        id="checkerboard_room",
        name="Checkerboard Calibration Room",
        description="Built-in synthetic calibration fixture with planar geometric patterns",
        url="",
        sha256="",
        size_mb=0.01,
        license="CC0 / Public Domain",
        attribution="hesim-3d built-in calibration asset",
        built_in=True,
        local_rel_path="assets/scenes/checkerboard_room.glb",
    ),
    "sponza": SceneMetadata(
        id="sponza",
        name="Sponza Atrium (Khronos / Crytek)",
        description="Classic architectural atrium benchmark with arches, columns and rich textures",
        url="https://raw.githubusercontent.com/KhronosGroup/glTF-Sample-Assets/main/Models/Sponza/glTF-Binary/Sponza.glb",
        sha256="d0408ca586071bc0a693c1b64ad74d4ebf3303dbccaf77174e2b0aeef97171e2",
        size_mb=35.0,
        license="CC-BY 3.0",
        attribution="Frank Meinl, Crytek / Khronos Group",
        built_in=False,
    ),
    "cornell_box": SceneMetadata(
        id="cornell_box",
        name="Cornell Box",
        description="Standard photometric test environment with red and green walls and dual diffuse blocks",
        url="https://raw.githubusercontent.com/KhronosGroup/glTF-Sample-Assets/main/Models/CornellBox/glTF-Binary/CornellBox.glb",
        sha256="4d715d08595906ef89b3f360706509fca059ee20b080fb05c868eb2a2c14041d",
        size_mb=0.5,
        license="CC0 / Public Domain",
        attribution="Cornell Program of Computer Graphics / Khronos Group",
        built_in=False,
    ),
    "bistro": SceneMetadata(
        id="bistro",
        name="Amazon Lumberyard Bistro Exterior",
        description="Urban Parisian street environment with detailed shopfronts, tables, and cobblestones",
        url="https://raw.githubusercontent.com/KhronosGroup/glTF-Sample-Assets/main/Models/CommercialStore/glTF-Binary/CommercialStore.glb",
        sha256="",
        size_mb=18.0,
        license="CC-BY 4.0",
        attribution="Amazon Lumberyard / Khronos Group",
        built_in=False,
    ),
}


def list_available_scenes() -> List[SceneMetadata]:
    """Return all registered scenes in the catalog."""
    return list(SCENE_CATALOG.values())


def get_scene_metadata(scene_id: str) -> SceneMetadata:
    """Retrieve scene metadata by ID."""
    scene_key = scene_id.strip().lower()
    if scene_key in SCENE_CATALOG:
        return SCENE_CATALOG[scene_key]
    raise KeyError(f"Scene '{scene_id}' not found in catalog. Available: {list(SCENE_CATALOG.keys())}")
