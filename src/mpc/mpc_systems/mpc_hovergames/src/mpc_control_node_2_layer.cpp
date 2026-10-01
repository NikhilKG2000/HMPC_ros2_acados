#include <mpc_base/configuration_mpc.h>
#include <mpc_base/controller_base.h>
#include <mpc_base/interface_base.h>
#include <mpc_base/module_handler_base.h>
#include <mpc_base/solver_base.h>

#include <mpc_core/controller.h>
#include <mpc_core/functions/update_mpc_configuration.h>
#include <mpc_modules/loader/module_handler.h>
#include <mpc_tools/data_saver_json.h>
#include <mpc_tools/robot_region.h>
#include <mpc_solver/hovergames/hovergames_solver_interface.h>

#include <rclcpp/rclcpp.hpp>

#include <mpc_hovergames/hovergames_simplesim_interface.h>

#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <cmath>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  const std::vector<std::string> arguments =
    rclcpp::remove_ros_arguments(argc, argv);

  if (arguments.size() != 2 ||
      (arguments[1] != "mpc_0" && arguments[1] != "mpc_1"))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("mpc_hovergames"),
      "Expected one solver argument: mpc_0 (tracker) or mpc_1 (planner)");
    rclcpp::shutdown();
    return 1;
  }

  const int solver_number = arguments[1] == "mpc_0" ? 0 : 1;
  const std::string role = solver_number == 0 ? "tracker" : "planner";

  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("layer_idx", solver_number);
  auto node = std::make_shared<rclcpp::Node>(
    role + "_node", node_options);

  RCLCPP_INFO(
    node->get_logger(),
    "Starting HMPC %s with solver index %d",
    role.c_str(), solver_number);

  // Create the main mpc configuration struct
  ConfigurationMPC configuration_mpc;
  if (!mpc_configuration_initialize(*node, configuration_mpc))
  {
    rclcpp::shutdown();
    return 1;
  }

  if (configuration_mpc.n_layers_ != 2)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "The two-layer controller requires n_layers=2, but received %d",
      configuration_mpc.n_layers_);
    rclcpp::shutdown();
    return 1;
  }

  std::unique_ptr<SolverBase> ptr_solver;
  try
  {
    ptr_solver = returnSolverPointer(solver_number);
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Could not construct the acados solver: %s",
      error.what());
    rclcpp::shutdown();
    return 1;
  }

  if (!ptr_solver)
  {
    RCLCPP_ERROR(node->get_logger(), "No solver exists at the requested index");
    rclcpp::shutdown();
    return 1;
  }

  // Load the original weight configuration supplied by the launch file.
  for (int index = 0; index < ptr_solver->n_par_weights_; ++index)
  {
    const auto& name = ptr_solver->par_weights_names_[index];

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = name + " weight";

    // Same range as the original dynamic-reconfigure definition.
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 1000.0;
    range.step = 0.0;
    descriptor.floating_point_range.push_back(range);

    const double weight = node->declare_parameter<double>(
      name, ptr_solver->par_weights_[index], descriptor);

    if (!std::isfinite(weight) || weight < 0.0 || weight > 1000.0)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Invalid weight for parameter '%s'", name.c_str());
      rclcpp::shutdown();
      return 1;
    }

    ptr_solver->par_weights_[index] = weight;
  }

  ptr_solver->loadWeightsParams();

  // Validate proposed weight updates without changing solver data.
  auto weight_validation_handle =
    node->add_on_set_parameters_callback(
      [solver = ptr_solver.get()](
        const std::vector<rclcpp::Parameter>& parameters)
      {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& parameter : parameters)
        {
          const auto& names = solver->par_weights_names_;
          if (std::find(names.begin(), names.end(),
                        parameter.get_name()) == names.end())
          {
            continue;
          }

          if (parameter.get_type() !=
                rclcpp::ParameterType::PARAMETER_DOUBLE ||
              !std::isfinite(parameter.as_double()))
          {
            result.successful = false;
            result.reason =
              parameter.get_name() + " must be a finite double";
            return result;
          }
        }

        return result;
      });

  // Apply weights after ROS 2 has accepted the parameter change.
  auto parameter_events =
    std::make_shared<rclcpp::ParameterEventHandler>(node);

  auto weight_update_handle =
    parameter_events->add_parameter_event_callback(
      [node, solver = ptr_solver.get()](
        const rcl_interfaces::msg::ParameterEvent& event)
      {
        if (event.node != node->get_fully_qualified_name())
        {
          return;
        }

        bool weight_changed = false;
        const auto& names = solver->par_weights_names_;

        for (const auto& parameter : event.changed_parameters)
        {
          if (std::find(names.begin(), names.end(),
                        parameter.name) != names.end())
          {
            weight_changed = true;
            break;
          }
        }

        if (!weight_changed)
        {
          return;
        }

        const auto weights = node->get_parameters(names);
        for (int index = 0; index < solver->n_par_weights_; ++index)
        {
          solver->par_weights_[index] = weights[index].as_double();
        }

        solver->loadWeightsParams();
      });


  auto ptr_robot_region = std::make_unique<RobotRegion>(
    Eigen::Vector2d(0.0, 0.0),
    0.0,
    ptr_solver->n_discs_,
    configuration_mpc.robot_width_,
    configuration_mpc.robot_length_,
    configuration_mpc.robot_center_of_mass_to_back_);
  auto ptr_data_saver_json = std::make_unique<DataSaverJson>();

  // Create a mutex to ensure thread safety in case of multithreading
  std::mutex mutex;

  auto ptr_module_handler = std::make_unique<ModuleHandler>(
    *node,
    &configuration_mpc,
    ptr_solver.get(),
    ptr_data_saver_json.get());

  std::unique_ptr<InterfaceBase> ptr_interface;
  std::unique_ptr<ControllerBase> ptr_controller = std::make_unique<Controller>(
    *node,
    &configuration_mpc,
    ptr_solver.get(),
    nullptr,
    ptr_module_handler.get(),
    ptr_data_saver_json.get());

  ptr_interface = std::make_unique<HovergamesSimpleSimInterface>(
    *node,
    ptr_controller.get(),
    &configuration_mpc,
    ptr_solver.get(),
    ptr_module_handler.get(),
    ptr_robot_region.get(),
    ptr_data_saver_json.get(),
    &mutex);

  // Complete the circular controller/interface connection only after both
  // objects exist.
  ptr_controller->setInterface(ptr_interface.get());

  rclcpp::spin(node);

  // Controller shutdown accesses the interface, so destroy the controller
  // while the interface is still alive.
  ptr_controller.reset();
  ptr_interface.reset();
  rclcpp::shutdown();

  return 0;
}
