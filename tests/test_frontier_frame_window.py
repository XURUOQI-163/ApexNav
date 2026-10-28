import unittest
from basic_utils.frontier_frame_window import Frame, FrameWindow, project_cells, evaluate_frame

def frame(stamp=1):
    return Frame(stamp=stamp,image=f'rgb-{stamp}',camera_x=0.,camera_y=0.,camera_yaw=0.,
                 resolution=1.,origin_x=-.5,origin_y=-.5,max_distance=5.,
                 free_cells={(1,0),(2,0)},camera_pose=(0,0,.88,0,0,0,1),
                 robot_pose=(0,0,0,0,0,0,1))

class FrontierFrameWindowTests(unittest.TestCase):
    def test_window_holds_last_twenty_original_frames(self):
        w=FrameWindow()
        for i in range(1,26):w.add(frame(i))
        self.assertEqual([f.stamp for f in w.snapshot()],list(range(6,26)))
        duplicate=frame(25);duplicate.image='annotated'
        w.add(duplicate)
        self.assertEqual(w.snapshot()[-1].image,'rgb-25')
        self.assertEqual(w.snapshot()[-1].robot_pose,(0,0,0,0,0,0,1))
    def test_projection_uses_actual_free_neighbors_original_pose_and_distance(self):
        f=frame()
        weights=dict(project_cells(f,[(10,2,0),(11,3,0),(12,-2,0),(13,2,2),(14,50,0)]))
        self.assertEqual(set(weights),{10,11})
        self.assertAlmostEqual(weights[10],1/(1+(2/5)**2))
        self.assertAlmostEqual(weights[11],1/(1+(3/5)**2))
        f.camera_yaw=3.141592653589793
        self.assertEqual(project_cells(f,[(10,2,0)]),[])
    def test_cached_image_score_reused_for_new_cells(self):
        f=frame();calls=[]
        def infer(image):calls.append(image);return .3
        result=evaluate_frame(f,[(10,2,0)],infer)
        self.assertEqual(result[0],.3)
        f.sent_ids.update(i for i,w in result[1])
        result=evaluate_frame(f,[(10,2,0),(11,3,0)],infer)
        self.assertEqual([i for i,w in result[1]],[11])
        self.assertEqual(calls,['rgb-1'])
        f.sent_ids.add(11)
        self.assertIsNone(evaluate_frame(f,[(10,2,0),(11,3,0)],infer))
    def test_missing_ray_support_never_infers(self):
        self.assertIsNone(evaluate_frame(frame(),[(10,30,0)],lambda _:self.fail('unseen frame inferred')))
    def test_frame_waits_until_a_visible_frontier_appears(self):
        f=frame();calls=[]
        def infer(image):calls.append(image);return .4
        self.assertIsNone(evaluate_frame(f,[],infer))
        self.assertEqual(calls,[])
        self.assertIsNone(f.score)
        self.assertEqual(evaluate_frame(f,[(10,2,0)],infer)[0],.4)
        self.assertEqual(calls,['rgb-1'])
    def test_nonfinite_model_score_rejected(self):
        with self.assertRaises(ValueError):evaluate_frame(frame(),[(10,2,0)],lambda _:float('nan'))
