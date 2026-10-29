#!/usr/bin/env python3
"""
open-automation-toolkit: Core Control & Workflow Automation Module
==================================================================
Provides lightweight, dependency-free utilities for robotics control
loops (PID with anti-windup) and resilient task automation pipelines.

Repository: https://github.com/moulzmara-lang/open-automation-toolkit
License: MIT
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Callable, Dict, List, Optional

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
)
logger = logging.getLogger("open_automation_toolkit")


@dataclass
class PIDConfig:
    """Configuration parameters for a discrete-time PID controller."""

    kp: float
    ki: float
    kd: float
    setpoint: float = 0.0
    output_min: Optional[float] = -100.0
    output_max: Optional[float] = 100.0
    integral_limit: Optional[float] = 50.0


@dataclass
class TelemetrySample:
    """Single time-step telemetry record for control loop diagnostics."""

    step: int
    timestamp: float
    setpoint: float
    measurement: float
    error: float
    control_output: float


class PIDController:
    """
    Discrete-time PID controller with integral anti-windup and output clamping.

    Designed for lightweight robotic actuator loops, temperature regulation,
    and automated physical system simulations.
    """

    def __init__(self, config: PIDConfig) -> None:
        self.config = config
        self._integral: float = 0.0
        self._prev_error: Optional[float] = None
        self._history: List[TelemetrySample] = []

    def reset(self) -> None:
        """Reset internal controller state and telemetry history."""
        self._integral = 0.0
        self._prev_error = None
        self._history.clear()

    def compute(self, measurement: float, dt: float) -> float:
        """
        Compute the control signal for a given process measurement and time step.

        Args:
            measurement: Current process variable value.
            dt: Time delta in seconds since the last update (must be > 0).

        Returns:
            Clamped control output signal.
        """
        if dt <= 0:
            raise ValueError("Time step dt must be strictly positive.")

        error = self.config.setpoint - measurement

        # Proportional term
        p_term = self.config.kp * error

        # Integral term with anti-windup clamping
        self._integral += error * dt
        if self.config.integral_limit is not None:
            limit = abs(self.config.integral_limit)
            self._integral = max(-limit, min(limit, self._integral))
        i_term = self.config.ki * self._integral

        # Derivative term
        if self._prev_error is None:
            d_term = 0.0
        else:
            d_term = self.config.kd * ((error - self._prev_error) / dt)
        self._prev_error = error

        output = p_term + i_term + d_term

        # Output saturation clamping
        if self.config.output_min is not None:
            output = max(self.config.output_min, output)
        if self.config.output_max is not None:
            output = min(self.config.output_max, output)

        sample = TelemetrySample(
            step=len(self._history) + 1,
            timestamp=round(time.time(), 4),
            setpoint=self.config.setpoint,
            measurement=round(measurement, 4),
            error=round(error, 4),
            control_output=round(output, 4),
        )
        self._history.append(sample)
        return output

    @property
    def history(self) -> List[TelemetrySample]:
        """Return recorded telemetry samples."""
        return list(self._history)


@dataclass
class StepResult:
    """Execution summary for a single workflow task."""

    name: str
    success: bool
    attempts: int
    duration_ms: float
    output: Any = None
    error: Optional[str] = None


@dataclass
class WorkflowTask:
    """Represents a modular step inside an automation workflow pipeline."""

    name: str
    action: Callable[[Dict[str, Any]], Any]
    max_retries: int = 2
    retry_delay_sec: float = 0.05
    metadata: Dict[str, Any] = field(default_factory=dict)


class WorkflowRunner:
    """
    Sequential workflow automation engine with retry policies, shared state
    context, and JSON report generation.
    """

    def __init__(self, workflow_name: str) -> None:
        self.workflow_name = workflow_name
        self.tasks: List[WorkflowTask] = []
        self.context: Dict[str, Any] = {}
        self.results: List[StepResult] = []

    def add_task(
        self,
        name: str,
        action: Callable[[Dict[str, Any]], Any],
        max_retries: int = 2,
        retry_delay_sec: float = 0.05,
    ) -> "WorkflowRunner":
        """Register a task step in the automation pipeline."""
        self.tasks.append(
            WorkflowTask(
                name=name,
                action=action,
                max_retries=max_retries,
                retry_delay_sec=retry_delay_sec,
            )
        )
        return self

    def run(self, initial_context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Execute all registered workflow tasks in sequence.

        Args:
            initial_context: Optional dictionary of input parameters shared across tasks.

        Returns:
            Dictionary containing overall workflow status, shared context, and step metrics.
        """
        self.context = dict(initial_context or {})
        self.results.clear()
        logger.info("Starting workflow: %s", self.workflow_name)

        workflow_success = True
        for task in self.tasks:
            start_ts = time.perf_counter()
            attempts = 0
            last_error: Optional[str] = None
            task_output: Any = None
            succeeded = False

            while attempts <= task.max_retries:
                attempts += 1
                try:
                    task_output = task.action(self.context)
                    succeeded = True
                    break
                except Exception as exc:  # noqa: BLE001
                    last_error = str(exc)
                    logger.warning(
                        "Task '%s' failed on attempt %d/%d: %s",
                        task.name,
                        attempts,
                        task.max_retries + 1,
                        last_error,
                    )
                    if attempts <= task.max_retries:
                        time.sleep(task.retry_delay_sec)

            duration_ms = round((time.perf_counter() - start_ts) * 1000.0, 3)
            result = StepResult(
                name=task.name,
                success=succeeded,
                attempts=attempts,
                duration_ms=duration_ms,
                output=task_output,
                error=None if succeeded else last_error,
            )
            self.results.append(result)

            if not succeeded:
                workflow_success = False
                logger.error("Workflow '%s' halted at task '%s'.", self.workflow_name, task.name)
                break

        summary = {
            "workflow_name": self.workflow_name,
            "success": workflow_success,
            "completed_steps": sum(1 for r in self.results if r.success),
            "total_steps": len(self.tasks),
            "steps": [asdict(r) for r in self.results],
        }
        logger.info(
            "Workflow '%s' finished (success=%s, steps=%d/%d)",
            self.workflow_name,
            workflow_success,
            summary["completed_steps"],
            summary["total_steps"],
        )
        return summary

    def export_report_json(self, indent: int = 2) -> str:
        """Serialize the latest workflow execution results to formatted JSON."""
        return json.dumps(
            {
                "workflow_name": self.workflow_name,
                "steps": [asdict(r) for r in self.results],
            },
            indent=indent,
        )


def run_demo_simulation() -> Dict[str, Any]:
    """
    Demonstrate an end-to-end robotics control simulation wrapped inside
    an automated workflow pipeline.
    """
    runner = WorkflowRunner("robotic-joint-calibration-pipeline")

    def step_initialize(ctx: Dict[str, Any]) -> str:
        ctx["config"] = PIDConfig(
            kp=3.5,
            ki=0.02,
            kd=2.8,
            setpoint=45.0,  # Target joint angle in degrees
            output_min=-120.0,
            output_max=120.0,
        )
        ctx["controller"] = PIDController(ctx["config"])
        return "PID controller initialized for 45.0 deg target"

    def step_simulate_control_loop(ctx: Dict[str, Any]) -> Dict[str, float]:
        controller: PIDController = ctx["controller"]
        position = 0.0
        velocity = 0.0
        dt = 0.05

        for _ in range(200):
            control_effort = controller.compute(measurement=position, dt=dt)
            # Damped second-order robotic joint actuator model
            acceleration = (control_effort - 4.0 * velocity) * 0.8
            velocity += acceleration * dt
            position += velocity * dt

        final_sample = controller.history[-1]
        ctx["final_position"] = final_sample.measurement
        ctx["final_error"] = final_sample.error
        return {
            "final_position_deg": final_sample.measurement,
            "steady_state_error_deg": final_sample.error,
            "samples_recorded": len(controller.history),
        }

    def step_validate_convergence(ctx: Dict[str, Any]) -> str:
        error = abs(ctx["final_error"])
        if error > 1.0:
            raise RuntimeError(f"Steady-state error {error:.3f} exceeds 1.0 deg tolerance")
        return f"Convergence verified (residual error: {error:.4f} deg)"

    runner.add_task("initialize_controller", step_initialize)
    runner.add_task("simulate_closed_loop", step_simulate_control_loop)
    runner.add_task("validate_convergence", step_validate_convergence)

    return runner.run()


if __name__ == "__main__":
    report = run_demo_simulation()
    print(json.dumps(report, indent=2))