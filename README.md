open-automation-toolkit
a lightweight open-source toolkit of modular workflows, scripts, and control utilities for robotics and AI-driven automation.Overviewopen-automation-toolkit provides dependency-free Python building blocks for engineering students, researchers, and developers working at the intersection of control systems and task automation:Discrete-Time PID Control (PIDController): Configurable Proportional-Integral-Derivative controller featuring integral anti-windup clamping, output saturation limits, and structured step-by-step telemetry recording.Resilient Workflow Pipelines (WorkflowRunner): Sequential task automation engine supporting shared execution context, configurable retry policies for transient hardware or API timeouts, and JSON telemetry export.Repository StructureFileDescriptionautomation_controller.pyCore library containing PIDController, WorkflowRunner, telemetry dataclasses, and a closed-loop robotic joint calibration demo.test_automation_controller.pyUnit test suite (unittest) covering proportional control, output saturation clamping, retry recovery, and closed-loop convergence.README.mdProject documentation, quickstart guide, and usage examples.Quickstart1. Clone the repositorygit clone https://github.com/moulzmara-lang/open-automation-toolkit.git
cd open-automation-toolkit
2. Run the closed-loop automation simulationpython3 automation_controller.py
3. Run the unit test suitepython3 -m unittest test_automation_controller.py -v
Usage Examplefrom automation_controller import PIDConfig, PIDController, WorkflowRunner

# 1. Configure a PID controller with anti-windup and saturation limits
config = PIDConfig(
    kp=2.4,
    ki=0.8,
    kd=0.15,
    setpoint=45.0,
    output_min=-120.0,
    output_max=120.0,
)
controller = PIDController(config)

# 2. Compute control effort for a sensor reading
control_signal = controller.compute(measurement=32.5, dt=0.05)
print(f"Control output: {control_signal:.2f}")

# 3. Orchestrate multi-step automation tasks with automatic retries
runner = WorkflowRunner("sensor-calibration-workflow")
runner.add_task("verify_control_signal", lambda ctx: f"Signal ready: {control_signal:.2f}")
summary = runner.run()
print(runner.export_report_json())
LicenseReleased under the MIT License. Contributions, bug reports, and pull requests are welcom
