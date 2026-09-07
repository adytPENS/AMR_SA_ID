import sys
from pathlib import Path
import unittest
from unittest.mock import Mock
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import vision_yolo
import object_feature_detector as vision


class YoloRulesTests(unittest.TestCase):
    def test_multiple_colors_one_object(self):
        rule = dict(name='Two color block', shape='cuboid', colors=['red','blue'], enabled=True,
                    label='{name} | {shape} | {colors} | {confidence:.2f}')
        vision_yolo.validate_objects([rule], vision.format_label)
        self.assertIs(vision_yolo.match_objects('cuboid',['red','blue'],[rule]),rule)
        self.assertIsNone(vision_yolo.match_objects('cylinder',['red','blue'],[rule]))
        self.assertIsNone(vision_yolo.match_objects('cuboid',['red'],[rule]))
        app = vision.App.__new__(vision.App)
        app.object_rules=[rule]
        app.rois=[dict(name='red',low=[170,100,100],high=[10,255,255],enabled=True),
                  dict(name='blue',low=[100,100,100],high=[130,255,255],enabled=True)]
        app.valid_min_area=lambda: 100
        app.label_template=vision.DEFAULT_LABEL
        frame=np.zeros((60,100,3),np.uint8)
        frame[:,:50]=(0,0,255)
        frame[:,50:]=(255,0,0)
        detections=[dict(shape='cuboid',confidence=.9,bbox=(0,0,100,60),mask=None)]
        lines=app.draw_yolo(frame,detections,frame.copy(),True,True,True)
        self.assertEqual(lines[0],'YOLO objects: 1')
        self.assertIn('Two color block | cuboid | red, blue | 0.90',lines[1])

    def test_disabled_and_priority(self):
        rule=dict(name='A',shape='Any',colors=[],enabled=False,label='')
        self.assertIsNone(vision_yolo.match_objects('cube',[],[rule]))
        rule['enabled']=True
        self.assertIs(vision_yolo.match_objects('cube',[],[rule,dict(rule,name='B')]),rule)

    def test_stale_frame_rejected(self):
        app=vision.App.__new__(vision.App)
        app.yolo_future=Mock()
        app.yolo_future.done.return_value=True
        app.yolo_future.result.return_value=('old frame',[])
        app.yolo_result=None
        app.generation=2
        app.future_generation=1
        app.yolo_var=Mock()
        app.yolo_var.get.return_value=False
        frame=np.zeros((10,10,3),np.uint8)
        shown, detections=app.yolo_detections(frame)
        self.assertIs(shown,frame)
        self.assertEqual(detections,[])

    def test_missing_model_error(self):
        with self.assertRaises(ValueError):
            vision_yolo.load_model('/tmp/nonexistent-vision-test-model.pt')

if __name__=='__main__':
    unittest.main()
