"""Shared command line for discrete evaluation and continuous Habitat simulation."""
import argparse


def parse_simulation_args(argv=None, default_mode='discrete'):
    parser = argparse.ArgumentParser(description='ApexNav Habitat simulation')
    parser.add_argument('--mode', choices=['discrete', 'continuous'], default=default_mode,
                        help='Action interface; the ROS2 planner must use the same mode (default: %(default)s)')
    parser.add_argument('--planner', choices=['APEXNAV', 'TOPOLOGY'], default='APEXNAV',
                        help='Must match ROS planner; TOPOLOGY is geometric exploration with MPC')
    parser.add_argument('--semantic-model', choices=['clip', 'blip2'], default='clip',
                        help='Scene similarity backend; requires the corresponding HTTP service')
    parser.add_argument('--dataset', choices=['hm3dv1', 'hm3dv2', 'mp3d'], default='hm3dv2')
    parser.add_argument('--target', default=None, metavar='DESCRIPTION',
                        help='Custom target, e.g. "yellow chair"; run one scene without dataset evaluation')
    parser.add_argument('--delay', type=float, default=0., help='Extra delay per discrete action, in seconds')
    args, overrides = parser.parse_known_args(argv)
    if args.target is not None:
        args.target = args.target.strip()
        if not args.target or '\n' in args.target or '\r' in args.target:
            parser.error('--target must be a nonempty, single-line description')
    if args.planner == 'TOPOLOGY' and args.mode != 'continuous':
        parser.error('TOPOLOGY requires --mode continuous')
    if args.planner == 'TOPOLOGY' and args.target is not None:
        parser.error('TOPOLOGY is geometric exploration; omit --target')
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
