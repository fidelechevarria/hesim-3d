#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "buffer_utils.h"
#include "filament_renderer.h"
#include "spline_se3.h"

#ifdef HESIM3D_ENABLE_GUI
#include <iostream>

#include "gui/gui_app.h"
#endif

namespace nb = nanobind;
using namespace nb::literals;

NB_MODULE(_hesim3d_core, m) {
  m.doc() = "C++20 high-performance core extension for hesim-3d";

  // ------------------------------------------------------------------------
  // Trajectory & Splines
  // ------------------------------------------------------------------------
  nb::class_<hesim3d::TrajectorySample>(m, "TrajectorySample")
      .def(nb::init<>())
      .def_rw("timestamp_sec", &hesim3d::TrajectorySample::timestamp_sec)
      .def_rw("position", &hesim3d::TrajectorySample::position)
      .def_prop_rw(
          "orientation_xyzw",
          [](const hesim3d::TrajectorySample& s) {
            return Eigen::Vector4d(s.orientation.x(), s.orientation.y(), s.orientation.z(),
                                   s.orientation.w());
          },
          [](hesim3d::TrajectorySample& s, const Eigen::Vector4d& q) {
            s.orientation = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
          })
      .def_rw("linear_velocity", &hesim3d::TrajectorySample::linear_velocity)
      .def_rw("linear_acceleration", &hesim3d::TrajectorySample::linear_acceleration)
      .def_rw("angular_velocity_body", &hesim3d::TrajectorySample::angular_velocity_body)
      .def_rw("imu_acceleration", &hesim3d::TrajectorySample::imu_acceleration);

  nb::class_<hesim3d::Keyframe>(m, "Keyframe")
      .def(nb::init<>())
      .def_rw("timestamp_sec", &hesim3d::Keyframe::timestamp_sec)
      .def_rw("position", &hesim3d::Keyframe::position)
      .def_prop_rw(
          "orientation_xyzw",
          [](const hesim3d::Keyframe& kf) {
            return Eigen::Vector4d(kf.orientation.x(), kf.orientation.y(), kf.orientation.z(),
                                   kf.orientation.w());
          },
          [](hesim3d::Keyframe& kf, const Eigen::Vector4d& q) {
            kf.orientation = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
          });

  nb::class_<hesim3d::SE3Spline>(m, "SE3Spline")
      .def(nb::init<double>(), "dt_sec"_a = 0.1)
      .def("set_control_interval", &hesim3d::SE3Spline::set_control_interval, "dt_sec"_a)
      .def("set_gravity", &hesim3d::SE3Spline::set_gravity, "g"_a)
      .def("set_keyframes", &hesim3d::SE3Spline::set_keyframes, "keyframes"_a)
      .def(
          "build_from_waypoints",
          [](hesim3d::SE3Spline& self, const std::vector<Eigen::Vector3d>& positions,
             const std::vector<Eigen::Vector4d>& orientations_xyzw, double total_duration_sec) {
            std::vector<Eigen::Quaterniond> quats;
            quats.reserve(orientations_xyzw.size());
            for (const auto& q : orientations_xyzw) {
              quats.emplace_back(q.w(), q.x(), q.y(), q.z());
            }
            self.build_from_waypoints(positions, quats, total_duration_sec);
          },
          "positions"_a, "orientations_xyzw"_a, "total_duration_sec"_a)
      .def("evaluate", &hesim3d::SE3Spline::evaluate, "t_sec"_a)
      .def("evaluate_batch", &hesim3d::SE3Spline::evaluate_batch, "timestamps"_a)
      .def_static("create_circle_lookat", &hesim3d::SE3Spline::create_circle_lookat, "center"_a,
                  "radius"_a, "height"_a, "duration_sec"_a, "dt_ctrl"_a = 0.1)
      .def_static("create_eight_loop", &hesim3d::SE3Spline::create_eight_loop, "center"_a,
                  "extent"_a, "duration_sec"_a, "speed_factor"_a = 1.0, "dt_ctrl"_a = 0.05)
      .def("min_time", &hesim3d::SE3Spline::min_time)
      .def("max_time", &hesim3d::SE3Spline::max_time)
      .def("num_control_points", &hesim3d::SE3Spline::num_control_points);

  // ------------------------------------------------------------------------
  // Camera Intrinsics
  // ------------------------------------------------------------------------
  nb::class_<hesim3d::CameraIntrinsics>(m, "CameraIntrinsics")
      .def(nb::init<>())
      .def_rw("width", &hesim3d::CameraIntrinsics::width)
      .def_rw("height", &hesim3d::CameraIntrinsics::height)
      .def_rw("fx", &hesim3d::CameraIntrinsics::fx)
      .def_rw("fy", &hesim3d::CameraIntrinsics::fy)
      .def_rw("cx", &hesim3d::CameraIntrinsics::cx)
      .def_rw("cy", &hesim3d::CameraIntrinsics::cy)
      .def_rw("near_plane", &hesim3d::CameraIntrinsics::near_plane)
      .def_rw("far_plane", &hesim3d::CameraIntrinsics::far_plane);

  // ------------------------------------------------------------------------
  // Scene Bounds
  // ------------------------------------------------------------------------
  nb::class_<hesim3d::SceneBounds>(m, "SceneBounds")
      .def(nb::init<>())
      .def_rw("min_point", &hesim3d::SceneBounds::min_point)
      .def_rw("max_point", &hesim3d::SceneBounds::max_point)
      .def_rw("center", &hesim3d::SceneBounds::center)
      .def_rw("extent", &hesim3d::SceneBounds::extent)
      .def_rw("radius", &hesim3d::SceneBounds::radius)
      .def_rw("valid", &hesim3d::SceneBounds::valid);

  // ------------------------------------------------------------------------
  // Filament Renderer Wrapper
  // ------------------------------------------------------------------------
  nb::class_<hesim3d::FilamentRenderer>(m, "FilamentRenderer")
      .def(nb::init<uint32_t, uint32_t, const std::string&>(), "width"_a, "height"_a,
           "backend_type"_a = "vulkan")
      .def("load_scene", &hesim3d::FilamentRenderer::load_scene, "glb_path"_a)
      .def("get_scene_bounds", &hesim3d::FilamentRenderer::get_scene_bounds)
      .def("load_environment", &hesim3d::FilamentRenderer::load_environment, "ibl_path"_a = "")
      .def("setup_default_lighting", &hesim3d::FilamentRenderer::setup_default_lighting)
      .def("set_intrinsics", &hesim3d::FilamentRenderer::set_intrinsics, "intrinsics"_a)
      .def(
          "set_camera_pose",
          [](hesim3d::FilamentRenderer& self, const Eigen::Vector3d& position,
             const Eigen::Vector4d& orientation_xyzw) {
            Eigen::Quaterniond q(orientation_xyzw.w(), orientation_xyzw.x(), orientation_xyzw.y(),
                                 orientation_xyzw.z());
            self.set_camera_pose(position, q);
          },
          "position"_a, "orientation_xyzw"_a)
      .def(
          "render_frame",
          [](hesim3d::FilamentRenderer& self, nb::ndarray<uint8_t, nb::c_contig> out_buffer,
             uint64_t timestamp_us) {
            return self.render_frame(out_buffer.data(), out_buffer.size(), timestamp_us);
          },
          "out_buffer"_a, "timestamp_us"_a = 0)
      .def("resize_camera", &hesim3d::FilamentRenderer::resize_camera, "width"_a, "height"_a)
      .def("width", &hesim3d::FilamentRenderer::width)
      .def("height", &hesim3d::FilamentRenderer::height);

  // ------------------------------------------------------------------------
  // GUI Launch Binding
  // ------------------------------------------------------------------------
#ifdef HESIM3D_ENABLE_GUI
  nb::class_<hesim3d::GuiConfig>(m, "GuiConfig")
      .def(nb::init<>())
      .def_rw("scene_path", &hesim3d::GuiConfig::scene_path)
      .def_rw("sensor_name", &hesim3d::GuiConfig::sensor_name)
      .def_rw("trajectory_path", &hesim3d::GuiConfig::trajectory_path)
      .def_rw("project_path", &hesim3d::GuiConfig::project_path)
      .def_rw("window_width", &hesim3d::GuiConfig::window_width)
      .def_rw("window_height", &hesim3d::GuiConfig::window_height)
      .def_rw("duration_sec", &hesim3d::GuiConfig::duration_sec)
      .def_rw("sim_fps", &hesim3d::GuiConfig::sim_fps)
      .def_rw("event_threshold", &hesim3d::GuiConfig::event_threshold)
      .def_rw("refractory_period_us", &hesim3d::GuiConfig::refractory_period_us)
      .def_rw("exposure_ms", &hesim3d::GuiConfig::exposure_ms)
      .def_rw("accumulation_window_ms", &hesim3d::GuiConfig::accumulation_window_ms)
      .def_rw("font_dir", &hesim3d::GuiConfig::font_dir);

  m.def("launch_gui", &hesim3d::launch_gui, "config"_a, nb::call_guard<nb::gil_scoped_release>(),
        "Launch interactive Dear ImGui desktop visualizer");

  m.def(
      "_test_cpp_scientific_bake_roundtrip",
      [](const std::string& sensor_name, int w, int h) {
        std::string dev, model;
        bool init_ok =
            hesim3d::init_scientific_bake_bridge(sensor_name, w, h, 0.20, 10, dev, model);
        if (!init_ok) return false;

        std::vector<uint8_t> dummy_frames(w * h * 3 * 2, 128);
        std::vector<uint64_t> ts = {0, 5000};
        std::vector<hesim3d::SimulatedEvent> events;
        std::vector<uint8_t> aps_frame;

        bool step_ok = hesim3d::step_scientific_bake_bridge(
            dummy_frames.data(), dummy_frames.size(), ts, 5000, events, aps_frame);

        hesim3d::reset_scientific_bake_bridge();
        return step_ok && aps_frame.size() == static_cast<size_t>(w * h * 3);
      },
      "sensor_name"_a, "w"_a, "h"_a);
#endif
}

#ifdef HESIM3D_ENABLE_GUI
namespace hesim3d {

bool export_simulation_to_hdf5(const std::string& path, const std::string& sensor_name,
                               const std::vector<SimulatedEvent>& events,
                               const std::vector<SimulatedApsFrame>& frames, int width, int height,
                               const SE3Spline& spline, double duration_sec) {
  try {
    nb::gil_scoped_acquire gil;

    size_t n_ev = events.size();
    std::vector<uint64_t> ev_t(n_ev);
    std::vector<uint16_t> ev_x(n_ev);
    std::vector<uint16_t> ev_y(n_ev);
    std::vector<int8_t> ev_p(n_ev);
    for (size_t i = 0; i < n_ev; ++i) {
      ev_t[i] = static_cast<uint64_t>(events[i].timestamp_sec * 1e6);
      ev_x[i] = events[i].x;
      ev_y[i] = events[i].y;
      ev_p[i] = events[i].polarity;
    }

    size_t n_frames = frames.size();
    std::vector<uint64_t> frame_ts(n_frames);
    std::vector<uint8_t> frame_bytes;
    frame_bytes.reserve(n_frames * width * height * 3);
    for (size_t i = 0; i < n_frames; ++i) {
      frame_ts[i] = static_cast<uint64_t>(frames[i].timestamp_sec * 1e6);
      frame_bytes.insert(frame_bytes.end(), frames[i].rgb_preview.begin(),
                         frames[i].rgb_preview.end());
    }

    double dur = std::max(0.1, duration_sec);
    int imu_samples = static_cast<int>(std::ceil(dur * 200.0)) + 1;
    std::vector<uint64_t> imu_ts(imu_samples);
    std::vector<double> imu_gyro(imu_samples * 3);
    std::vector<double> imu_acc(imu_samples * 3);
    std::vector<uint64_t> gt_ts(imu_samples);
    std::vector<double> gt_pos(imu_samples * 3);
    std::vector<double> gt_quat(imu_samples * 4);

    for (int i = 0; i < imu_samples; ++i) {
      double t = std::min(dur, i * (dur / std::max(1, imu_samples - 1)));
      uint64_t t_us = static_cast<uint64_t>(t * 1e6);
      imu_ts[i] = t_us;
      gt_ts[i] = t_us;

      hesim3d::TrajectorySample s = spline.evaluate(t);
      imu_gyro[i * 3 + 0] = s.angular_velocity_body.x();
      imu_gyro[i * 3 + 1] = s.angular_velocity_body.y();
      imu_gyro[i * 3 + 2] = s.angular_velocity_body.z();

      imu_acc[i * 3 + 0] = s.imu_acceleration.x();
      imu_acc[i * 3 + 1] = s.imu_acceleration.y();
      imu_acc[i * 3 + 2] = s.imu_acceleration.z();

      gt_pos[i * 3 + 0] = s.position.x();
      gt_pos[i * 3 + 1] = s.position.y();
      gt_pos[i * 3 + 2] = s.position.z();

      gt_quat[i * 4 + 0] = s.orientation.x();
      gt_quat[i * 4 + 1] = s.orientation.y();
      gt_quat[i * 4 + 2] = s.orientation.z();
      gt_quat[i * 4 + 3] = s.orientation.w();
    }

    nb::object writer_mod = nb::module_::import_("hesim3d.io.hdf5_writer");
    nb::object export_fn = writer_mod.attr("export_baked_simulation");

    nb::bytes py_bytes(frame_bytes.data(), frame_bytes.size());

    nb::object result =
        export_fn(path, sensor_name, width, height, static_cast<int>(n_frames), ev_t, ev_x, ev_y,
                  ev_p, py_bytes, frame_ts, imu_ts, imu_gyro, imu_acc, gt_ts, gt_pos, gt_quat);
    return nb::cast<bool>(result);
  } catch (const std::exception& e) {
    std::cerr << "[GuiApp] Failed to export HDF5 dataset: " << e.what() << std::endl;
    return false;
  }
}

bool init_scientific_bake_bridge(const std::string& sensor_name, int width, int height,
                                 double event_threshold, int refractory_period_us,
                                 std::string& out_device_name, std::string& out_model_info,
                                 const std::string& output_h5_path) {
  try {
    nb::gil_scoped_acquire gil;
    nb::object bake_mod = nb::module_::import_("hesim3d.sensor.scientific_bake");
    nb::object init_fn = bake_mod.attr("init_scientific_engine");
    nb::tuple res = nb::cast<nb::tuple>(
        init_fn(sensor_name, width, height, event_threshold, refractory_period_us,
                output_h5_path.empty() ? nb::none() : nb::cast(output_h5_path)));
    out_device_name = nb::cast<std::string>(res[0]);
    out_model_info = nb::cast<std::string>(res[1]);
    return true;
  } catch (const std::exception& e) {
    std::cerr << "[ScientificBake] Failed to initialize engine: " << e.what() << std::endl;
    out_device_name = "CPU (Fallback)";
    out_model_info = "Error initializing PyTorch engine";
    return false;
  }
}

bool step_scientific_bake_bridge(const uint8_t* sub_frames_data, size_t total_bytes,
                                 const std::vector<uint64_t>& sub_timestamps_us,
                                 uint64_t shutter_duration_us,
                                 std::vector<SimulatedEvent>& out_events,
                                 std::vector<uint8_t>& out_aps_frame,
                                 size_t* out_total_physical_events) {
  try {
    nb::gil_scoped_acquire gil;
    nb::object bake_mod = nb::module_::import_("hesim3d.sensor.scientific_bake");
    nb::object step_fn = bake_mod.attr("step_scientific_engine");

    nb::bytes py_bytes(sub_frames_data, total_bytes);
    nb::object res = step_fn(py_bytes, sub_timestamps_us, shutter_duration_us);
    nb::tuple tuple_res = nb::cast<nb::tuple>(res);

    std::vector<uint64_t> ev_t = nb::cast<std::vector<uint64_t>>(tuple_res[0]);
    std::vector<uint16_t> ev_x = nb::cast<std::vector<uint16_t>>(tuple_res[1]);
    std::vector<uint16_t> ev_y = nb::cast<std::vector<uint16_t>>(tuple_res[2]);
    std::vector<int8_t> ev_p = nb::cast<std::vector<int8_t>>(tuple_res[3]);
    nb::bytes blurred_bytes = nb::cast<nb::bytes>(tuple_res[4]);

    if (out_total_physical_events != nullptr) {
      if (tuple_res.size() > 5) {
        *out_total_physical_events = nb::cast<size_t>(tuple_res[5]);
      } else {
        *out_total_physical_events = ev_t.size();
      }
    }

    size_t n_ev = ev_t.size();
    out_events.clear();
    out_events.reserve(n_ev);
    for (size_t i = 0; i < n_ev; ++i) {
      out_events.push_back({static_cast<double>(ev_t[i]) * 1e-6, ev_x[i], ev_y[i], ev_p[i]});
    }

    const char* b_data = blurred_bytes.c_str();
    size_t b_len = blurred_bytes.size();
    out_aps_frame.assign(reinterpret_cast<const uint8_t*>(b_data),
                         reinterpret_cast<const uint8_t*>(b_data) + b_len);

    return true;
  } catch (const std::exception& e) {
    std::cerr << "[ScientificBake] Simulation step error: " << e.what() << std::endl;
    return false;
  }
}

void finalize_scientific_bake_bridge(const std::vector<uint64_t>& imu_timestamps_us,
                                     const std::vector<double>& imu_gyro_flat,
                                     const std::vector<double>& imu_acc_flat,
                                     const std::vector<uint64_t>& gt_timestamps_us,
                                     const std::vector<double>& gt_pos_flat,
                                     const std::vector<double>& gt_quat_flat) {
  try {
    nb::gil_scoped_acquire gil;
    nb::object bake_mod = nb::module_::import_("hesim3d.sensor.scientific_bake");
    nb::object fin_fn = bake_mod.attr("finalize_scientific_engine");
    fin_fn(imu_timestamps_us, imu_gyro_flat, imu_acc_flat, gt_timestamps_us, gt_pos_flat,
           gt_quat_flat);
  } catch (const std::exception& e) {
    std::cerr << "[ScientificBake] Finalize error: " << e.what() << std::endl;
  }
}

void reset_scientific_bake_bridge() {
  try {
    nb::gil_scoped_acquire gil;
    nb::object bake_mod = nb::module_::import_("hesim3d.sensor.scientific_bake");
    nb::object reset_fn = bake_mod.attr("reset_scientific_engine");
    reset_fn();
  } catch (const std::exception& e) {
    std::cerr << "[ScientificBake] Reset error: " << e.what() << std::endl;
  }
}

}  // namespace hesim3d
#endif
