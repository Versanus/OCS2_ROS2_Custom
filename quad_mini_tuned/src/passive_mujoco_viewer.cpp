#include "ament_index_cpp/get_package_share_directory.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <stdexcept>
#include <string>

namespace {

using SteadyClock = std::chrono::steady_clock;

constexpr float kAxisRed[4] = {0.9f, 0.2f, 0.2f, 1.0f};
constexpr float kAxisGreen[4] = {0.2f, 0.8f, 0.2f, 1.0f};
constexpr float kAxisBlue[4] = {0.2f, 0.4f, 0.95f, 1.0f};
constexpr float kGravityYellow[4] = {0.95f, 0.85f, 0.2f, 1.0f};

struct ViewerState {
  mjModel* model = nullptr;
  mjData* data = nullptr;
  mjvCamera camera;
  mjvOption option;
  mjvScene scene;
  mjrContext context;
  bool left_button = false;
  bool middle_button = false;
  bool right_button = false;
  bool paused = false;
  bool contacts_enabled = false;
  double sim_speed = 1.0;
  double last_x = 0.0;
  double last_y = 0.0;
  SteadyClock::time_point last_wall_time = SteadyClock::now();
};

ViewerState* g_viewer = nullptr;

std::string defaultUrdfPath() {
  const std::filesystem::path share_dir = ament_index_cpp::get_package_share_directory("new_robot");
  const std::filesystem::path workspace_source_urdf =
      share_dir.parent_path().parent_path().parent_path().parent_path() /
      "quad_mini_tuned" / "robot_raw.urdf";
  if (std::filesystem::exists(workspace_source_urdf)) {
    return workspace_source_urdf.string();
  }

  return (share_dir / "robot_raw.urdf").string();
}

struct ViewerConfig {
  std::filesystem::path urdf_path;
  std::string base_rpy = "0 0 0";
  std::string base_rotation_label = "none";
};

ViewerConfig parseCommandLine(int argc, char** argv) {
  ViewerConfig config;
  config.urdf_path = std::filesystem::path(defaultUrdfPath());

  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--rotate-base-x90") {
      config.base_rpy = "1.57079632679 0 0";
      config.base_rotation_label = "x +90deg";
    } else if (argument == "--rotate-base-y90") {
      config.base_rpy = "0 1.57079632679 0";
      config.base_rotation_label = "y +90deg";
    } else if (argument == "--rotate-base-z90") {
      config.base_rpy = "0 0 1.57079632679";
      config.base_rotation_label = "z +90deg";
    } else if (!argument.empty() && argument[0] == '-') {
      throw std::runtime_error(
          "Unknown option: " + argument +
          "\nSupported options: --rotate-base-x90, --rotate-base-y90, --rotate-base-z90");
    } else {
      config.urdf_path = std::filesystem::path(argument);
    }
  }

  return config;
}

std::string readFile(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Could not open file: " + path.string());
  }

  return std::string((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
}

void writeFile(const std::filesystem::path& path, const std::string& content) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Could not write file: " + path.string());
  }

  output << content;
}

void replaceAll(std::string& text, const std::string& from, const std::string& to) {
  if (from.empty()) {
    return;
  }

  std::size_t position = 0;
  while ((position = text.find(from, position)) != std::string::npos) {
    text.replace(position, from.size(), to);
    position += to.size();
  }
}

std::string applyBaseRotationToUrdf(const std::string& urdf_content, const std::string& base_rpy) {
  if (base_rpy == "0 0 0") {
    return urdf_content;
  }

  const std::string joint_begin = "<joint name=\"floating_base\"";
  const std::size_t joint_pos = urdf_content.find(joint_begin);
  if (joint_pos == std::string::npos) {
    throw std::runtime_error("Could not find floating_base joint in URDF to apply base rotation.");
  }

  const std::size_t joint_end = urdf_content.find("</joint>", joint_pos);
  if (joint_end == std::string::npos) {
    throw std::runtime_error("Could not find floating_base closing tag in URDF.");
  }

  std::string rotated_urdf = urdf_content;
  const std::size_t origin_pos = rotated_urdf.find("<origin", joint_pos);
  if (origin_pos == std::string::npos || origin_pos > joint_end) {
    throw std::runtime_error("Could not find floating_base origin in URDF to apply base rotation.");
  }

  const std::string rpy_begin = "rpy=\"";
  const std::size_t rpy_pos = rotated_urdf.find(rpy_begin, origin_pos);
  if (rpy_pos == std::string::npos || rpy_pos > joint_end) {
    throw std::runtime_error("Could not find floating_base rpy attribute in URDF.");
  }

  const std::size_t rpy_value_begin = rpy_pos + rpy_begin.size();
  const std::size_t rpy_value_end = rotated_urdf.find('"', rpy_value_begin);
  if (rpy_value_end == std::string::npos || rpy_value_end > joint_end) {
    throw std::runtime_error("Could not parse floating_base rpy attribute in URDF.");
  }

  rotated_urdf.replace(rpy_value_begin, rpy_value_end - rpy_value_begin, base_rpy);
  return rotated_urdf;
}

std::filesystem::path prepareUrdfForMujoco(const std::filesystem::path& urdf_path, const std::string& base_rpy) {
  const std::filesystem::path urdf_directory = urdf_path.parent_path();
  const std::filesystem::path mesh_directory = urdf_directory / "meshes";
  const std::string package_mesh_prefix = "package://new_robot/meshes/";
  const std::filesystem::path prepared_directory =
      std::filesystem::temp_directory_path() / "new_robot_passive_mujoco_assets";

  std::string urdf_content = readFile(urdf_path);
  // MuJoCo imports the root collision but drops the root visual mesh, so use the base mesh
  // for the root collision geometry in this passive viewer path.
  replaceAll(urdf_content, "<box size=\"0.30 0.15 0.10\"/>", "<mesh filename=\"meshes/base_link.obj\"/>");
  replaceAll(urdf_content, "filename=\"meshes/", "filename=\"");
  replaceAll(urdf_content, "filename=\"" + package_mesh_prefix, "filename=\"");
  urdf_content = applyBaseRotationToUrdf(urdf_content, base_rpy);

  std::error_code error;
  std::filesystem::remove_all(prepared_directory, error);
  std::filesystem::create_directories(prepared_directory);

  for (const auto& entry : std::filesystem::directory_iterator(mesh_directory)) {
    if (!entry.is_regular_file()) {
      continue;
    }

    const std::filesystem::path target = prepared_directory / entry.path().filename();
    std::filesystem::copy_file(entry.path(), target, std::filesystem::copy_options::overwrite_existing);
  }

  const std::filesystem::path prepared_urdf = prepared_directory / "robot_for_mujoco.urdf";
  writeFile(prepared_urdf, urdf_content);
  return prepared_urdf;
}

void resetToZeroPose(ViewerState& viewer) {
  mj_resetData(viewer.model, viewer.data);
  std::fill(viewer.data->qpos, viewer.data->qpos + viewer.model->nq, 0.0);
  std::fill(viewer.data->qvel, viewer.data->qvel + viewer.model->nv, 0.0);
  if (viewer.model->na > 0) {
    std::fill(viewer.data->act, viewer.data->act + viewer.model->na, 0.0);
  }
  if (viewer.model->nu > 0) {
    std::fill(viewer.data->ctrl, viewer.data->ctrl + viewer.model->nu, 0.0);
  }
  mj_forward(viewer.model, viewer.data);
  viewer.last_wall_time = SteadyClock::now();
}

void applyContactMode(ViewerState& viewer) {
  if (viewer.contacts_enabled) {
    viewer.model->opt.disableflags &= ~mjDSBL_CONTACT;
  } else {
    viewer.model->opt.disableflags |= mjDSBL_CONTACT;
  }
}

void resetWallClockSync(ViewerState& viewer) {
  viewer.last_wall_time = SteadyClock::now();
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
  (void)window;
  (void)scancode;
  (void)mods;

  if (g_viewer == nullptr || action != GLFW_PRESS) {
    return;
  }

  if (key == GLFW_KEY_BACKSPACE || key == GLFW_KEY_R) {
    resetToZeroPose(*g_viewer);
  } else if (key == GLFW_KEY_SPACE) {
    g_viewer->paused = !g_viewer->paused;
    resetWallClockSync(*g_viewer);
  } else if (key == GLFW_KEY_C) {
    g_viewer->contacts_enabled = !g_viewer->contacts_enabled;
    applyContactMode(*g_viewer);
  } else if (key == GLFW_KEY_ENTER) {
    mj_step(g_viewer->model, g_viewer->data);
    resetWallClockSync(*g_viewer);
  } else if (key == GLFW_KEY_LEFT_BRACKET || key == GLFW_KEY_MINUS) {
    g_viewer->sim_speed = std::max(0.125, g_viewer->sim_speed * 0.5);
  } else if (key == GLFW_KEY_RIGHT_BRACKET || key == GLFW_KEY_EQUAL) {
    g_viewer->sim_speed = std::min(8.0, g_viewer->sim_speed * 2.0);
  } else if (key == GLFW_KEY_0) {
    g_viewer->sim_speed = 1.0;
  }
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
  (void)button;
  (void)action;
  (void)mods;

  if (g_viewer == nullptr) {
    return;
  }

  g_viewer->left_button = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  g_viewer->middle_button = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
  g_viewer->right_button = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  glfwGetCursorPos(window, &g_viewer->last_x, &g_viewer->last_y);
}

void cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
  if (g_viewer == nullptr) {
    return;
  }

  if (!g_viewer->left_button && !g_viewer->middle_button && !g_viewer->right_button) {
    return;
  }

  const double dx = xpos - g_viewer->last_x;
  const double dy = ypos - g_viewer->last_y;
  g_viewer->last_x = xpos;
  g_viewer->last_y = ypos;

  int width = 0;
  int height = 0;
  glfwGetWindowSize(window, &width, &height);
  if (height <= 0) {
    return;
  }

  const bool shift_down =
      glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

  mjtMouse action;
  if (g_viewer->right_button) {
    action = shift_down ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (g_viewer->left_button) {
    action = shift_down ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  mjv_moveCamera(g_viewer->model, action, dx / height, dy / height, &g_viewer->scene, &g_viewer->camera);
}

void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
  (void)window;
  (void)xoffset;

  if (g_viewer == nullptr) {
    return;
  }

  mjv_moveCamera(g_viewer->model, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &g_viewer->scene, &g_viewer->camera);
}

void configureCamera(ViewerState& viewer) {
  mjv_defaultFreeCamera(viewer.model, &viewer.camera);
  viewer.camera.lookat[0] = 0.0;
  viewer.camera.lookat[1] = 0.0;
  viewer.camera.lookat[2] = 0.0;
  viewer.camera.distance = 1.2;
  viewer.camera.azimuth = 145.0;
  viewer.camera.elevation = -30.0;
}

void addArrow(mjvScene& scene, const float* rgba, const double from[3], const double to[3], double radius) {
  if (scene.ngeom >= scene.maxgeom) {
    return;
  }

  mjvGeom* geom = &scene.geoms[scene.ngeom++];
  mjv_initGeom(geom, mjGEOM_ARROW, nullptr, nullptr, nullptr, rgba);
  mjv_connector(geom, mjGEOM_ARROW, radius, from, to);
  geom->category = mjCAT_DECOR;
  geom->objid = -1;
  geom->segid = -1;
}

void addReferenceDecor(ViewerState& viewer) {
  const double origin[3] = {0.0, 0.0, 0.0};
  const double x_axis[3] = {0.20, 0.0, 0.0};
  const double y_axis[3] = {0.0, 0.20, 0.0};
  const double z_axis[3] = {0.0, 0.0, 0.20};
  const double gravity_tip[3] = {
      0.0,
      0.0,
      viewer.model->opt.gravity[2] > 0.0 ? 0.25 : -0.25,
  };

  addArrow(viewer.scene, kAxisRed, origin, x_axis, 0.006);
  addArrow(viewer.scene, kAxisGreen, origin, y_axis, 0.006);
  addArrow(viewer.scene, kAxisBlue, origin, z_axis, 0.006);
  addArrow(viewer.scene, kGravityYellow, origin, gravity_tip, 0.01);
}

}  // namespace

int main(int argc, char** argv) {
  ViewerConfig config;
  try {
    config = parseCommandLine(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

  const std::filesystem::path& urdf_path = config.urdf_path;
  if (!std::filesystem::exists(urdf_path)) {
    std::cerr << "URDF file not found: " << urdf_path << '\n';
    return 1;
  }

  const std::filesystem::path prepared_urdf = prepareUrdfForMujoco(urdf_path, config.base_rpy);

  char error[1024] = "";
  ViewerState viewer;
  mjv_defaultCamera(&viewer.camera);
  mjv_defaultOption(&viewer.option);
  mjv_defaultScene(&viewer.scene);
  mjr_defaultContext(&viewer.context);

  viewer.model = mj_loadXML(prepared_urdf.c_str(), nullptr, error, sizeof(error));
  if (viewer.model == nullptr) {
    std::cerr << "Failed to load URDF: " << urdf_path << '\n';
    if (std::strlen(error) > 0) {
      std::cerr << error << '\n';
    }
    return 1;
  }

  viewer.data = mj_makeData(viewer.model);
  if (viewer.data == nullptr) {
    std::cerr << "Failed to allocate MuJoCo data.\n";
    mj_deleteModel(viewer.model);
    return 1;
  }

  resetToZeroPose(viewer);
  viewer.model->opt.gravity[0] = 0.0;
  viewer.model->opt.gravity[1] = 0.0;
  viewer.model->opt.gravity[2] = -9.81;
  viewer.contacts_enabled = false;
  applyContactMode(viewer);

  if (!glfwInit()) {
    std::cerr << "Could not initialize GLFW.\n";
    mj_deleteData(viewer.data);
    mj_deleteModel(viewer.model);
    return 1;
  }

  GLFWwindow* window = glfwCreateWindow(1280, 900, "quad_mini passive MuJoCo viewer", nullptr, nullptr);
  if (window == nullptr) {
    std::cerr << "Could not create GLFW window.\n";
    glfwTerminate();
    mj_deleteData(viewer.data);
    mj_deleteModel(viewer.model);
    return 1;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  mjv_makeScene(viewer.model, &viewer.scene, 2000);
  mjr_makeContext(viewer.model, &viewer.context, mjFONTSCALE_150);
  configureCamera(viewer);

  g_viewer = &viewer;
  glfwSetKeyCallback(window, keyCallback);
  glfwSetMouseButtonCallback(window, mouseButtonCallback);
  glfwSetCursorPosCallback(window, cursorPosCallback);
  glfwSetScrollCallback(window, scrollCallback);

  std::cout << "Loaded URDF: " << urdf_path << '\n'
            << "Base rotation: " << config.base_rotation_label << '\n'
            << "Compiled model stats: nq=" << viewer.model->nq
            << " nv=" << viewer.model->nv
            << " nu=" << viewer.model->nu << '\n'
            << "Passive viewer controls:\n"
            << "  space      pause/resume\n"
            << "  enter      single-step when paused\n"
            << "  r/backspace reset to zero joint pose\n"
            << "  c          toggle contacts on/off\n"
            << "  [ / ]      slower / faster sim speed\n"
            << "  0          reset sim speed to 1.0x\n"
            << "  mouse      orbit/pan/zoom camera\n";

  while (!glfwWindowShouldClose(window)) {
    if (!viewer.paused) {
      const auto now = SteadyClock::now();
      double elapsed_wall_seconds =
          std::chrono::duration<double>(now - viewer.last_wall_time).count();
      viewer.last_wall_time = now;

      // Clamp long gaps so focus changes or debugging pauses don't fast-forward the sim.
      elapsed_wall_seconds = std::clamp(elapsed_wall_seconds, 0.0, 0.05);
      const mjtNum target_sim_time =
          viewer.data->time + elapsed_wall_seconds * viewer.sim_speed;

      while (viewer.data->time < target_sim_time) {
        mj_step(viewer.model, viewer.data);
      }
    } else {
      resetWallClockSync(viewer);
    }

    mjrRect viewport{0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    mjv_updateScene(viewer.model, viewer.data, &viewer.option, nullptr, &viewer.camera, mjCAT_ALL, &viewer.scene);
    addReferenceDecor(viewer);
    mjr_render(viewport, &viewer.scene, &viewer.context);

    const mjtNum overlay_grid = mjGRID_TOPLEFT;
    const std::string left_overlay =
        std::string("nu: ") + std::to_string(viewer.model->nu) +
        "\nspeed: " + std::to_string(viewer.sim_speed) + "x" +
        "\ncontacts: " + (viewer.contacts_enabled ? "on" : "off") +
        "\nworld Z: up" +
        "\ngravity: -Z" +
        "\n" + (viewer.paused ? "space: resume" : "space: pause") +
        "\nenter: step\nr: reset zeros\nc: toggle contacts\n[: slower ]: faster";
    mjr_overlay(mjFONT_NORMAL, overlay_grid, viewport, left_overlay.c_str(), "", &viewer.context);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  g_viewer = nullptr;
  mjv_freeScene(&viewer.scene);
  mjr_freeContext(&viewer.context);
  glfwDestroyWindow(window);
  glfwTerminate();
  mj_deleteData(viewer.data);
  mj_deleteModel(viewer.model);
  std::error_code ignored_error;
  std::filesystem::remove_all(prepared_urdf.parent_path(), ignored_error);
  return 0;
}
