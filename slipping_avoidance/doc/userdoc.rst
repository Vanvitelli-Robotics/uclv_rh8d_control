:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/slipping_avoidance/doc/userdoc.rst

.. _slipping_avoidance_userdoc:

Force Torque Sensor Broadcaster
--------------------------------
Broadcaster of messages from force/torque state interfaces of a robot or sensor.
The published message type is ``geometry_msgs/msg/WrenchStamped``.

The controller is a wrapper around ``ForceTorqueSensor`` semantic component (see ``controller_interface`` package).


Parameters
^^^^^^^^^^^
This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters. The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/slipping_avoidance/src/slipping_avoidance_parameters.yaml>`_ contains descriptions for all the parameters used by the controller.

The interfaces can be defined in two ways, using the ``sensor_name`` or the ``interface_names`` parameter:
Those two parameters cannot be defined at the same time.

Full list of parameters:

.. generate_parameter_library_details:: ../src/slipping_avoidance_parameters.yaml
  parameters_context.yaml

An example parameter file for this controller can be found in `the test directory <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/slipping_avoidance/test/slipping_avoidance_params.yaml>`_:

.. literalinclude:: ../test/slipping_avoidance_params.yaml
   :language: yaml
