"""Mode selection and velocity-command tests, without Habitat, ROS or GPU."""
import importlib
import unittest


def require(test, name):
    try:
        return importlib.import_module(name)
    except ModuleNotFoundError as exc:
        test.fail(str(exc))


class SimulationModesTests(unittest.TestCase):
    def test_default_mode_keeps_discrete_hydra_overrides(self):
        cli = require(self, 'basic_utils.simulation_cli')
        args, overrides = cli.parse_simulation_args(['--dataset', 'hm3dv2', 'test_epi_num=150'])
        self.assertEqual(args.mode, 'discrete')
        self.assertEqual(cli.config_overrides(args.mode, overrides), ['test_epi_num=150'])

    def test_continuous_has_zero_step_and_preserves_user_settings(self):
        cli = require(self, 'basic_utils.simulation_cli')
        args, overrides = cli.parse_simulation_args([
            '--mode', 'continuous', '--dataset', 'mp3d',
            'test_epi_num=150', 'habitat.environment.max_episode_steps=600',
            'habitat.dataset.data_path=/data/mp3d/val.json.gz'])
        result = cli.config_overrides(args.mode, overrides)
        from omegaconf import OmegaConf
        cfg = OmegaConf.from_dotlist([item.lstrip('+') for item in result])
        self.assertEqual(args.dataset, 'mp3d')
        self.assertEqual(cfg.habitat.simulator.forward_step_size, 0.0)
        self.assertEqual(cfg.habitat.simulator.turn_angle, 0)
        self.assertEqual(cfg.habitat.environment.max_episode_steps, 600)
        self.assertEqual(cfg.habitat.dataset.data_path, '/data/mp3d/val.json.gz')
        self.assertEqual(cfg.test_epi_num, 150)

    def test_invalid_mode_is_rejected(self):
        cli = require(self, 'basic_utils.simulation_cli')
        with self.assertRaises(SystemExit):
            cli.parse_simulation_args(['--mode', 'teleport'])

    def test_discrete_delay_is_not_silently_ignored_in_continuous_mode(self):
        cli = require(self, 'basic_utils.simulation_cli')
        with self.assertRaises(SystemExit):
            cli.parse_simulation_args(['--mode', 'continuous', '--delay', '1'])


class VelocityCommandTests(unittest.TestCase):
    def command(self):
        return require(self, 'basic_utils.velocity_command').VelocityCommand(timeout=.5)

    def test_no_command_and_expired_command_stop_motion(self):
        command = self.command()
        self.assertEqual(command.sample(now=0.), (0., 0.))
        command.update(.4, -.2, now=1.)
        self.assertEqual(command.sample(now=1.2), (.4, -.2))
        self.assertEqual(command.sample(now=1.6), (0., 0.))

    def test_new_command_after_timeout_resumes_motion(self):
        command = self.command()
        command.update(.4, 0., now=1.)
        command.update(.2, .3, now=2.)
        self.assertEqual(command.sample(now=2.1), (.2, .3))

    def test_invalid_velocity_stops_instead_of_reusing_previous_command(self):
        command = self.command()
        command.update(.4, .1, now=1.)
        command.update(float('nan'), .2, now=1.1)
        self.assertEqual(command.sample(now=1.2), (0., 0.))

    def test_explicit_stop_clears_last_command(self):
        command = self.command()
        command.update(.4, .1, now=1.)
        command.stop()
        self.assertEqual(command.sample(now=1.2), (0., 0.))
