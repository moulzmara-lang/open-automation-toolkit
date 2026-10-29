#!/usr/bin/env python3
"""
Unit tests for open-automation-toolkit.
Run with: python3 -m unittest test_automation_controller.py
"""

import unittest
from automation_controller import (
    PIDConfig,
    PIDController,
    WorkflowRunner,
    run_demo_simulation,
)


class TestPIDController(unittest.TestCase):
    """Test suite for discrete-time PIDController."""

    def test_proportional_response(self) -> None:
        config = PIDConfig(kp=2.0, ki=0.0, kd=0.0, setpoint=10.0)
        pid = PIDController(config)
        output = pid.compute(measurement=4.0, dt=0.1)
        self.assertAlmostEqual(output, 12.0)

    def test_output_saturation_clamping(self) -> None:
        config = PIDConfig(
            kp=10.0,
            ki=0.0,
            kd=0.0,
            setpoint=50.0,
            output_min=-25.0,
            output_max=25.0,
        )
        pid = PIDController(config)
        self.assertEqual(pid.compute(measurement=0.0, dt=0.1), 25.0)
        self.assertEqual(pid.compute(measurement=100.0, dt=0.1), -25.0)

    def test_invalid_dt_raises_value_error(self) -> None:
        pid = PIDController(PIDConfig(kp=1.0, ki=0.1, kd=0.01))
        with self.assertRaises(ValueError):
            pid.compute(measurement=1.0, dt=0.0)


class TestWorkflowRunner(unittest.TestCase):
    """Test suite for sequential WorkflowRunner and retry policies."""

    def test_workflow_retries_and_succeeds(self) -> None:
        runner = WorkflowRunner("retry-test-pipeline")
        counter = {"calls": 0}

        def flaky_task(ctx: dict) -> str:
            counter["calls"] += 1
            if counter["calls"] < 2:
                raise RuntimeError("Transient sensor timeout")
            ctx["recovered"] = True
            return "ok"

        runner.add_task("flaky_sensor_read", flaky_task, max_retries=2, retry_delay_sec=0.001)
        summary = runner.run()

        self.assertTrue(summary["success"])
        self.assertEqual(summary["steps"][0]["attempts"], 2)
        self.assertTrue(runner.context.get("recovered"))

    def test_demo_simulation_converges(self) -> None:
        report = run_demo_simulation()
        self.assertTrue(report["success"])
        self.assertEqual(report["completed_steps"], 3)


if __name__ == "__main__":
    unittest.main()