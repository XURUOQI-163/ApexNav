"""Shared command line for discrete evaluation and continuous Habitat simulation."""
import argparse


def parse_simulation_args(argv=None, default_mode='discrete'):
    parser = argparse.ArgumentParser(description='ApexNav Habitat simulation')
    parser.add_argument('--mode', choices=['discrete', 'continuous'], default=default_mode,
                        help='Action interface; the ROS2 planner must use the same mode (default: %(default)s)')
    parser.add_argument('--semantic-model', choices=['clip', 'blip2'], default='clip',
                        help='Scene similarity backend; requires the corresponding HTTP service')
    parser.add_argument('--dataset', choices=['hm3dv1', 'hm3dv2', 'mp3d'], default='hm3dv2')
    parser.add_argument('--delay', type=float, default=0., help='Extra delay per discrete action, in seconds')
    args, overrides = parser.parse_known_args(argv)
    if args.delay < 0:
        parser.error('--delay must be nonnegative')
    if args.mode == 'continuous' and args.delay != 0:
        parser.error('--delay only applies to discrete mode; use continuous.control_hz for continuous mode')
    for item in overrides:
        if item.startswith('--'):
            parser.error(f'unrecognized argument: {item}')
    return args, overrides


def config_overrides(mode, overrides):
    """Mode defaults precede user overrides, so dataset paths/budgets still work."""
    defaults = []
    if mode == 'continuous':
        defaults = [
            'habitat.simulator.forward_step_size=0.0',
            'habitat.simulator.turn_angle=0',
            'habitat.simulator.habitat_sim_v0.enable_physics=false',
            'habitat.environment.max_episode_steps=9000',
            '+continuous.control_hz=30.0',
            '+continuous.command_timeout=0.5',
            '+continuous.initial_scan_seconds=4.0',
        ]
    return defaults + list(overrides)
