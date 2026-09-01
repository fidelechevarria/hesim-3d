from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Optional, Union
import numpy as np
import cv2
from mcap.writer import Writer as MCAPWriterCore
from hesim3d.sensor.config import SensorConfig


class MCAPDatasetWriter:
    """
    Export simulator streams into the open MCAP container format for Foxglove Studio and ROS2 workflows.
    """

    def __init__(self, output_path: Union[str, Path], sensor_config: Optional[SensorConfig] = None):
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.sensor_config = sensor_config or SensorConfig()

        self._file = open(self.output_path, "wb")
        self._writer = MCAPWriterCore(self._file)
        self._writer.start()

        # Register JSON Schema
        schema_json = json.dumps({
            "type": "object",
            "properties": {
                "timestamp_us": {"type": "integer"},
                "data": {"type": "object"}
            }
        }).encode("utf-8")

        self._schema_id = self._writer.register_schema(
            name="hesim3d.Message",
            encoding="jsonschema",
            data=schema_json,
        )

        # Register Channels
        self._ch_events = self._writer.register_channel(
            topic="/sensor/events",
            message_encoding="json",
            schema_id=self._schema_id,
        )
        self._ch_image = self._writer.register_channel(
            topic="/sensor/image",
            message_encoding="json",
            schema_id=self._schema_id,
        )
        self._ch_imu = self._writer.register_channel(
            topic="/sensor/imu",
            message_encoding="json",
            schema_id=self._schema_id,
        )
        self._ch_pose = self._writer.register_channel(
            topic="/camera/pose",
            message_encoding="json",
            schema_id=self._schema_id,
        )

    def write_events(self, events: np.ndarray) -> None:
        """Write event packets to MCAP stream."""
        if len(events) == 0:
            return

        t_log_ns = int(events["t"][0]) * 1000
        msg_dict = {
            "timestamp_us": int(events["t"][0]),
            "num_events": len(events),
            "events": [
                {"t": int(e["t"]), "x": int(e["x"]), "y": int(e["y"]), "p": int(e["p"])}
                for e in events[:5000] # Subsample batch for real-time JSON channel
            ]
        }
        self._writer.add_message(
            channel_id=self._ch_events,
            log_time=t_log_ns,
            data=json.dumps(msg_dict).encode("utf-8"),
            publish_time=t_log_ns,
        )

    def write_frame(self, image: np.ndarray, timestamp_us: int) -> None:
        """Encode and write an image frame."""
        t_log_ns = int(timestamp_us) * 1000
        # Fast JPEG encoding for compact MCAP transport
        _, enc_buf = cv2.imencode(".jpg", cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if image.ndim == 3 else image)
        
        msg_dict = {
            "timestamp_us": int(timestamp_us),
            "width": int(image.shape[1]),
            "height": int(image.shape[0]),
            "encoding": "jpeg",
            "data_base64": enc_buf.tobytes().hex(),
        }
        self._writer.add_message(
            channel_id=self._ch_image,
            log_time=t_log_ns,
            data=json.dumps(msg_dict).encode("utf-8"),
            publish_time=t_log_ns,
        )

    def write_imu(self, timestamp_us: int, angular_velocity: np.ndarray, linear_acceleration: np.ndarray) -> None:
        """Write an IMU telemetry sample."""
        t_log_ns = int(timestamp_us) * 1000
        msg_dict = {
            "timestamp_us": int(timestamp_us),
            "angular_velocity": [float(x) for x in angular_velocity],
            "linear_acceleration": [float(x) for x in linear_acceleration],
        }
        self._writer.add_message(
            channel_id=self._ch_imu,
            log_time=t_log_ns,
            data=json.dumps(msg_dict).encode("utf-8"),
            publish_time=t_log_ns,
        )

    def write_ground_truth_pose(self, timestamp_us: int, position: np.ndarray, orientation_xyzw: np.ndarray) -> None:
        """Write 6-DoF ground truth pose."""
        t_log_ns = int(timestamp_us) * 1000
        msg_dict = {
            "timestamp_us": int(timestamp_us),
            "position": [float(x) for x in position],
            "orientation_xyzw": [float(x) for x in orientation_xyzw],
        }
        self._writer.add_message(
            channel_id=self._ch_pose,
            log_time=t_log_ns,
            data=json.dumps(msg_dict).encode("utf-8"),
            publish_time=t_log_ns,
        )

    def close(self) -> None:
        self._writer.finish()
        self._file.close()

    def __enter__(self) -> MCAPDatasetWriter:
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()
