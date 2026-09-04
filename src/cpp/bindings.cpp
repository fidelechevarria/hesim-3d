#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/ndarray.h>

#include "spline_se3.h"
#include "filament_renderer.h"
#include "buffer_utils.h"

#ifdef HESIM3D_ENABLE_GUI
#include "gui/gui_app.h"
#include <iostream>
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
        .def_prop_rw("orientation_xyzw",
            [](const hesim3d::TrajectorySample& s) {
                return Eigen::Vector4d(s.orientation.x(), s.orientation.y(), s.orientation.z(), s.orientation.w());
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
        .def_prop_rw("orientation_xyzw",
            [](const hesim3d::Keyframe& kf) {
                return Eigen::Vector4d(kf.orientation.x(), kf.orientation.y(), kf.orientation.z(), kf.orientation.w());
            },
            [](hesim3d::Keyframe& kf, const Eigen::Vector4d& q) {
                kf.orientation = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
            });

    nb::class_<hesim3d::SE3Spline>(m, "SE3Spline")
        .def(nb::init<double>(), "dt_sec"_a = 0.1)
        .def("set_control_interval", &hesim3d::SE3Spline::set_control_interval, "dt_sec"_a)
        .def("set_gravity", &hesim3d::SE3Spline::set_gravity, "g"_a)
        .def("set_keyframes", &hesim3d::SE3Spline::set_keyframes, "keyframes"_a)
        .def("build_from_waypoints", [](hesim3d::SE3Spline& self,
                                        const std::vector<Eigen::Vector3d>& positions,
                                        const std::vector<Eigen::Vector4d>& orientations_xyzw,
                                        double total_duration_sec) {
            std::vector<Eigen::Quaterniond> quats;
            quats.reserve(orientations_xyzw.size());
            for (const auto& q : orientations_xyzw) {
                quats.emplace_back(q.w(), q.x(), q.y(), q.z());
            }
            self.build_from_waypoints(positions, quats, total_duration_sec);
        }, "positions"_a, "orientations_xyzw"_a, "total_duration_sec"_a)
        .def("evaluate", &hesim3d::SE3Spline::evaluate, "t_sec"_a)
        .def("evaluate_batch", &hesim3d::SE3Spline::evaluate_batch, "timestamps"_a)
        .def_static("create_circle_lookat", &hesim3d::SE3Spline::create_circle_lookat,
                    "center"_a, "radius"_a, "height"_a, "duration_sec"_a, "dt_ctrl"_a = 0.1)
        .def_static("create_eight_loop", &hesim3d::SE3Spline::create_eight_loop,
                    "center"_a, "extent"_a, "duration_sec"_a, "speed_factor"_a = 1.0, "dt_ctrl"_a = 0.05)
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
        .def(nb::init<uint32_t, uint32_t, const std::string&>(),
             "width"_a, "height"_a, "backend_type"_a = "vulkan")
        .def("load_scene", &hesim3d::FilamentRenderer::load_scene, "glb_path"_a)
        .def("get_scene_bounds", &hesim3d::FilamentRenderer::get_scene_bounds)
        .def("load_environment", &hesim3d::FilamentRenderer::load_environment, "ibl_path"_a = "")
        .def("setup_default_lighting", &hesim3d::FilamentRenderer::setup_default_lighting)
        .def("set_intrinsics", &hesim3d::FilamentRenderer::set_intrinsics, "intrinsics"_a)
        .def("set_camera_pose", [](hesim3d::FilamentRenderer& self,
                                   const Eigen::Vector3d& position,
                                   const Eigen::Vector4d& orientation_xyzw) {
            Eigen::Quaterniond q(orientation_xyzw.w(), orientation_xyzw.x(), orientation_xyzw.y(), orientation_xyzw.z());
            self.set_camera_pose(position, q);
        }, "position"_a, "orientation_xyzw"_a)
        .def("render_frame", [](hesim3d::FilamentRenderer& self,
                                nb::ndarray<uint8_t, nb::c_contig> out_buffer,
                                uint64_t timestamp_us) {
            return self.render_frame(out_buffer.data(), out_buffer.size(), timestamp_us);
        }, "out_buffer"_a, "timestamp_us"_a = 0)
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

    m.def("launch_gui", &hesim3d::launch_gui, "config"_a,
          nb::call_guard<nb::gil_scoped_release>(),
          "Launch interactive Dear ImGui desktop visualizer");
#endif
}

#ifdef HESIM3D_ENABLE_GUI
namespace hesim3d {

bool export_simulation_to_hdf5(
    const std::string& path,
    const std::string& sensor_name,
    const std::vector<SimulatedEvent>& events,
    const std::vector<SimulatedApsFrame>& frames,
    int width, int height,
    const SE3Spline& spline,
    double duration_sec
) {
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
            frame_bytes.insert(frame_bytes.end(), frames[i].rgb_preview.begin(), frames[i].rgb_preview.end());
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

        nb::object result = export_fn(
            path,
            sensor_name,
            width,
            height,
            static_cast<int>(n_frames),
            ev_t,
            ev_x,
            ev_y,
            ev_p,
            py_bytes,
            frame_ts,
            imu_ts,
            imu_gyro,
            imu_acc,
            gt_ts,
            gt_pos,
            gt_quat
        );
        return nb::cast<bool>(result);
    } catch (const std::exception& e) {
        std::cerr << "[GuiApp] Failed to export HDF5 dataset: " << e.what() << std::endl;
        return false;
    }
}

} // namespace hesim3d
#endif
