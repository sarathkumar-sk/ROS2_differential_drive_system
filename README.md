This project implements a complete ROS2-based software stack for a differential drive robot. It is divided into three functional packages that handle robot simulation/logic, motion control, and system deployment.
Core Functionality

    robot_package: The core "Digital Twin" of the robot.

        State Management: Implements an ON/OFF state machine controlled via custom ROS2 services (TurnRobotOn, TurnRobotOff).

        Kinematics & Odometry: Calculates real-time pose updates x(t) based on differential drive equations of motion at 10 Hz.

        Safety & Constraints: Features velocity clamping and automatic state resetting (zeroing velocities) when transitioning to an OFF state.

    controller_package: The navigation brain.

        Automated Startup: Waits for robot services to become available before initializing.

        Motion Profiling: Executes a two-stage maneuver: a precise angular rotation to a target degree, followed by a 1-meter linear translation with high precision (1 mm tolerance).

    launch_package: Orchestration layer providing standardized entry points for launching the robot logic and controller nodes independently.
