#!/usr/bin/env python3

import asyncio
import sys

import cv2
import numpy as np

sys.path.insert(0, '../lab11')
from imgclassification import ImageClassifier
import time
import cozmo
from cozmo.util import degrees
from collections import Counter
try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')

img_clf = ImageClassifier()

#Define a decorator as a subclass of Annotator; displays battery voltage
class BatteryAnnotator(cozmo.annotate.Annotator):
    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)
        batt = self.world.robot.battery_voltage
        text = cozmo.annotate.ImageText('BATT %.1fv' % batt, color='green')
        text.render(d, bounds)

async def run(robot: cozmo.robot.Robot):

    robot.world.image_annotator.add_annotator('battery', BatteryAnnotator)

    try:
        #Reset the robot's lift and head
        robot.move_lift(-3)
        robot.set_head_angle(degrees(0)).wait_for_completed()

        batch_predictions = []
        last_batch_start = int(round(time.time() * 1000))
        while True:
            cur_millis = int(round(time.time() * 1000))
            #Batch predictions by time to smooth out false predictions
            #If a batch has completed, take the mode prediction
            if (cur_millis - last_batch_start >= 500):
                data = Counter(batch_predictions)
                mode_prediction = data.most_common()[0][0]
                print(mode_prediction)
                if mode_prediction != "none":
                    await robot.play_anim_trigger(cozmo.anim.Triggers.AcknowledgeObject).wait_for_completed()
                    await robot.say_text(mode_prediction).wait_for_completed()
                last_batch_start = cur_millis
                batch_predictions.clear()

            #Make a prediction against the current camera image, and add it to the current batch
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)
            features = img_clf.extract_image_features([np.asarray(event.image)])
            predicted_labels = img_clf.predict_labels(features)
            batch_predictions.append(predicted_labels[0])

    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)

if __name__ == '__main__':
    #Train the classifier on the training data
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    train_data = img_clf.extract_image_features(train_raw)
    img_clf.train_classifier(train_data, train_labels)

    #Dispatch the program to Cozmo
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)