"""Controlled runner checks inside the Habitat/ROS venv; no scene/model loading.

Run: PYTHONPATH="$PWD:$PYTHONPATH" python tests/ros2_custom_target.py
"""
from contextlib import ExitStack, redirect_stdout
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np
from builtin_interfaces.msg import Time
from omegaconf import OmegaConf
import habitat_evaluation as discrete
import habitat_vel_control as continuous
from params import ACTION, ROS_STATE


class SceneFixture:
    def __init__(self):
        self.episodes = [SimpleNamespace(object_category='chair', scene_id=f'scene-{i}',
                          episode_id=str(i), goals=[]) for i in range(2)]
        self.current_episode = self.episodes[0]
        self.episode_iterator = iter(self.episodes)
        self.number_of_episodes = 2
        self.episode_over = False
        self.steps = 0
        self.sim = MagicMock()
        self.sim.agents = [MagicMock()]
        self.sim.agents[0].state.position = np.zeros(3)
        self.sim.agents[0].state.rotation = 'rotation'
        self.sim.step_filter.side_effect = lambda start, end:end

    def __enter__(self):return self
    def __exit__(self,*args):self.close()
    def close(self):pass
    def reset(self):return {'rgb':np.zeros((4,4,3), np.uint8)}
    def step(self,action):
        self.steps += 1
        self.episode_over = True
        return self.reset()
    def get_metrics(self):return {}  # Custom runs must not require success/SPL/distance.


def config(tmp):
    return OmegaConf.create({
        'test_epi_num':1, 'need_video':False,
        'video_output_path':tmp, 'record_file_name':'record.txt', 'continue_file_name':'continue.txt',
        'detector':{}, 'continuous':{'control_hz':30., 'initial_scan_seconds':0.},
        'llm':{'llm_answer_path':str(Path(tmp)/'answers.txt'),
               'llm_response_path':str(Path(tmp)/'responses.txt'), 'llm_client':{'llm_client':'none'}},
        'habitat':{'dataset':{'split':'val','data_path':'/data/hm3d/val.json.gz'},
                   'task':{'measurements':{'success':{'success_distance':.2}}},
                   'simulator':{'forward_step_size':0.}, 'environment':{'max_episode_steps':50}}})


def main():
    target='red bookshelf near the desk'
    # Exercise real Hydra composition and shared CLI, intercept before simulation.
    with patch.object(continuous,'run_continuous') as run:
        discrete.run_cli(['--mode','continuous','--semantic-model','blip2','--target',target,
                          'test_epi_num=1','llm.llm_client.llm_client=none'])
        assert run.call_args.kwargs['target_description']==target
        assert run.call_args.args[0].semantic_model=='blip2'
    with TemporaryDirectory() as tmp:
        for need_video in (False, True):
            cfg=config(tmp)
            cfg.need_video=need_video
            fixture=SceneFixture();node=MagicMock();output=StringIO()
            if need_video:
                fixture.get_metrics=lambda:{'success':1.,'spl':1.,'distance_to_goal':0.,'top_down_map':None}
            node.get_clock.return_value.now.return_value.to_msg.return_value=Time(sec=1)
            def spin(current,**kwargs):
                current.ros_state=ROS_STATE.PUB_ACTION
                current.global_action=ACTION.MOVE_FORWARD
            with ExitStack() as stack:
                stack.enter_context(patch.object(discrete,'patch_config',side_effect=lambda c:c))
                stack.enter_context(patch.object(discrete.habitat,'Env',return_value=fixture))
                stack.enter_context(patch.object(discrete.rclpy,'ok',return_value=True))
                stack.enter_context(patch.object(discrete.rclpy,'spin_once',side_effect=spin))
                stack.enter_context(patch.object(discrete,'reset_episode'))
                detect=stack.enter_context(patch.object(discrete,'get_object',side_effect=lambda label,img,*a:(img,[],[])))
                stack.enter_context(patch.object(discrete,'get_object_point_cloud',return_value=[]))
                stack.enter_context(patch.object(discrete,'is_on_same_floor',side_effect=AssertionError('Dataset goal used')))
                stack.enter_context(patch.object(discrete,'write_record',side_effect=AssertionError('Evaluation record written')))
                stack.enter_context(patch.object(discrete,'check_failure',side_effect=AssertionError('Dataset failure classified')))
                stack.enter_context(patch.object(discrete,'overlay_frame',side_effect=AssertionError('Dataset metrics overlaid')))
                def visual_frame(observation, info):
                    assert 'ros_stamp' not in observation, 'ROS metadata passed as a visual sensor'
                    return observation['rgb']
                stack.enter_context(patch.object(discrete,'observations_to_image',side_effect=visual_frame))
                frame_counts=[]
                video=stack.enter_context(patch.object(discrete,'images_to_video',side_effect=lambda frames,*a,**k:frame_counts.append(len(frames))))
                stack.enter_context(redirect_stdout(output))
                discrete.main(cfg,node,target_description=target)
            assert video.called==need_video
            if need_video:assert frame_counts==[2]
            assert fixture.steps==1, 'Custom discrete target incorrectly skipped scene with no dataset goals'
            assert fixture.current_episode.episode_id=='1'
            assert detect.call_args.args[0]==target
            assert node.semantic_bridge.set_target.call_args.args[:2]==(target,'everywhere')
            assert 'Custom-target simulation ended' in output.getvalue()
            assert 'Average Success' not in output.getvalue()
            assert not (Path(tmp)/'record.txt').exists()
            print('PASS: discrete custom target routes intact, uses one selected scene, ignores old-goal filtering/reporting')

        cfg.need_video=False
        fixture=SceneFixture();node=MagicMock();node.stop_requested=False
        node.command.sample.return_value=(0.,0.)
        control=MagicMock();control.integrate_transform.return_value=SimpleNamespace(translation=np.zeros(3),rotation='rotation')
        output=StringIO()
        with ExitStack() as stack:
            stack.enter_context(patch.object(continuous,'patch_config',side_effect=lambda c:c))
            stack.enter_context(patch.object(continuous.habitat,'Env',return_value=fixture))
            stack.enter_context(patch.object(continuous.rclpy,'ok',return_value=True))
            stack.enter_context(patch.object(continuous.rclpy,'spin_once'))
            stack.enter_context(patch.object(continuous.habitat_sim.physics,'VelocityControl',return_value=control))
            stack.enter_context(patch.object(continuous.habitat_sim,'RigidState',side_effect=lambda q,p:SimpleNamespace(translation=p)))
            stack.enter_context(patch.object(continuous.utils,'quat_to_magnum',return_value='rotation'))
            stack.enter_context(patch.object(continuous.utils,'quat_from_magnum',return_value='rotation'))
            stack.enter_context(patch.object(continuous.time,'sleep'))
            stack.enter_context(redirect_stdout(output))
            continuous.run_simulation(cfg,node,MagicMock(),target_description=target)
        assert fixture.steps==1 and fixture.current_episode.episode_id=='1'
        assert node.label_pub.publish.call_args.args[0].data==target
        assert 'Custom-target simulation ended' in output.getvalue()
        assert 'Continuous episode metrics' not in output.getvalue()
        print('PASS: continuous custom target reaches ROS label publication, uses selected scene, skips dataset metrics')


if __name__=='__main__':main()
