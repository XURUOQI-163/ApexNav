"""Exercise real runner/CLI with a controlled scene; no EGL or model loading."""
from contextlib import ExitStack, redirect_stdout
from io import StringIO
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4]/'tests'))
from ros2_custom_target import SceneFixture, config
import habitat_evaluation as entry
import habitat_vel_control as runner


def main():
    with patch.object(runner, 'run_continuous') as run:
        entry.run_cli(['--mode', 'continuous', '--planner', 'TOPOLOGY', 'test_epi_num=1'])
        assert run.call_args.args[0].navigation_planner == 'TOPOLOGY'
        assert run.call_args.args[0].habitat.simulator.forward_step_size == 0
    with TemporaryDirectory() as temp:
        cfg = config(temp)
        cfg.navigation_planner = 'TOPOLOGY'
        cfg.continuous.initial_scan_seconds = 3.
        cfg.continuous.command_timeout = .5
        fixture = SceneFixture()
        fixture.get_metrics = MagicMock(side_effect=AssertionError('semantic evaluation used'))
        node = MagicMock(); node.stop_requested = False
        node.command.sample.return_value = (.15, .2)
        control = MagicMock()
        control.integrate_transform.return_value = SimpleNamespace(translation=np.zeros(3), rotation='rotation')
        output = StringIO()
        with ExitStack() as stack:
            stack.enter_context(patch.object(runner, 'patch_config', side_effect=lambda c:c))
            stack.enter_context(patch.object(runner.habitat, 'Env', return_value=fixture))
            stack.enter_context(patch.object(runner.rclpy, 'ok', return_value=True))
            stack.enter_context(patch.object(runner.rclpy, 'spin_once'))
            stack.enter_context(patch.object(runner, 'resolve_navigation_target', side_effect=AssertionError('semantic target requested')))
            stack.enter_context(patch.object(runner.habitat_sim.physics, 'VelocityControl', return_value=control))
            stack.enter_context(patch.object(runner.habitat_sim, 'RigidState', side_effect=lambda q,p:SimpleNamespace(translation=p)))
            stack.enter_context(patch.object(runner.utils, 'quat_to_magnum', return_value='rotation'))
            stack.enter_context(patch.object(runner.utils, 'quat_from_magnum', return_value='rotation'))
            stack.enter_context(patch.object(runner.time, 'sleep'))
            stack.enter_context(redirect_stdout(output))
            runner.run_simulation(cfg, node, None)
        assert fixture.steps == 1
        np.testing.assert_allclose(control.angular_velocity, [0., .2, 0.])
        np.testing.assert_allclose(control.linear_velocity, [0., 0., -.15])
        node.label_pub.publish.assert_not_called()
        node.trigger_pub.publish.assert_not_called()
        assert 'Geometric exploration ended' in output.getvalue()
        with ExitStack() as stack:
            stack.enter_context(patch.object(runner, 'patch_config', side_effect=lambda c:c))
            stack.enter_context(patch.object(runner, 'HabitatVelControlNode', return_value=node))
            stack.enter_context(patch.object(runner.rclpy, 'init'))
            stack.enter_context(patch.object(runner.rclpy, 'ok', return_value=True))
            stack.enter_context(patch.object(runner.rclpy, 'shutdown'))
            stack.enter_context(patch.object(runner.signal, 'signal'))
            reset = stack.enter_context(patch.object(runner, 'reset_episode'))
            finish = stack.enter_context(patch.object(runner, 'finish_episode'))
            stack.enter_context(patch.object(runner.multiprocessing, 'get_context', side_effect=AssertionError('visual process started')))
            simulate = stack.enter_context(patch.object(runner, 'run_simulation'))
            runner.run_continuous(cfg)
            assert simulate.call_args.args[2] is None
            assert reset.call_count == 1 and finish.call_count == 1
            reset.reset_mock(); finish.reset_mock()
            simulate.side_effect = KeyboardInterrupt
            runner.run_continuous(cfg)
            assert reset.call_count == 1 and finish.call_count == 1
        print('PASS geometric runner: CLI selection, no visual process/semantic target/labels/triggers, cmd_vel controls initial motion')


if __name__ == '__main__':
    main()
