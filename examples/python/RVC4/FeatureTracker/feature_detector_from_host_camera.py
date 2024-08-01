#!/usr/bin/env python3

import depthai as dai
import cv2
import time
import numpy as np
from pathlib import Path

def drawFeatures(frame, features):
        pointColor = (0, 0, 255)
        greenColor = (0, 255, 0)
        circleRadius = 2
        for feature in features:
            # descriptor = feature.descriptor
            # print(descriptor)
            if feature.age > 2:
                cv2.circle(frame, (int(feature.position.x), int(feature.position.y)), circleRadius, pointColor, -1, cv2.LINE_AA, 0)
            # elif feature.age < 255:
            #     cv2.circle(frame, (int(feature.position.x), int(feature.position.y)), circleRadius, (0, 0, feature.age), -1, cv2.LINE_AA, 0)
            else:
                cv2.circle(frame, (int(feature.position.x), int(feature.position.y)), circleRadius, greenColor, -1, cv2.LINE_AA, 0)

examplesRoot = Path(__file__).parent / Path('../../').resolve()
models = examplesRoot / Path('models')
videoPath = models / Path('construction_vest.mp4')

# Create pipeline
info = dai.DeviceInfo("127.0.0.1")
info.protocol = dai.X_LINK_TCP_IP
info.state = dai.X_LINK_GATE
info.platform = dai.X_LINK_RVC3


initialThreshold = 1000
initialRobustness = 100

def on_trackbar(val):
    cfg = dai.FeatureTrackerConfig()
    cornerDetector = dai.FeatureTrackerConfig.CornerDetector()
    cornerDetector.numMaxFeatures = cv2.getTrackbarPos('numMaxFeatures', 'Features')

    thresholds = dai.FeatureTrackerConfig.CornerDetector.Thresholds()
    thresholds.initialValue = cv2.getTrackbarPos('harris_score','Features')
    cornerDetector.thresholds = thresholds

    cfg.setCornerDetector(cornerDetector)
    cfg.setMotionEstimator(motionEstimator)

    inputConfigQueue.send(cfg)

cv2.namedWindow('Features')

# create trackbars threshold and robustness change
cv2.createTrackbar('harris_score','Features',2000,25000, on_trackbar)
# cv2.createTrackbar('robustness','Features',100,127, on_trackbar)
cv2.createTrackbar('numMaxFeatures','Features',256,1024, on_trackbar)


with dai.Device(info) as device:
    with dai.Pipeline(device) as pipeline:
        cnt = 0
        replay = pipeline.create(dai.node.ReplayVideo)
        replay.setReplayVideoFile(videoPath)
        replay.setSize(640, 640)
        replay.setOutFrameType(dai.ImgFrame.Type.GRAY8)
        
        featureTrackerLeft = pipeline.create(dai.node.FeatureTracker)
       
        # featureTrackerLeft.initialConfig.setHarrisCornerDetectorThreshold(initialThreshold)
        # featureTrackerLeft.initialConfig.setHarrisCornerDetectorRobustness(initialRobustness)
        featureTrackerLeft.initialConfig.setCornerDetector(dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS)
        featureTrackerLeft.initialConfig.setMotionEstimator(False)

        motionEstimator = dai.FeatureTrackerConfig.MotionEstimator()
        motionEstimator.enable = True
        featureTrackerLeft.initialConfig.setMotionEstimator(motionEstimator)

        cornerDetector = dai.FeatureTrackerConfig.CornerDetector()
        cornerDetector.numMaxFeatures = 256




        # featureTrackerLeft.initialConfig.setNumMaxFeatures(256)


        outputFeaturePassthroughQueue = featureTrackerLeft.passthroughInputImage.createOutputQueue()
        outputFeatureQueue = featureTrackerLeft.outputFeatures.createOutputQueue()

        replay.out.link(featureTrackerLeft.inputImage)

        inputConfigQueue = featureTrackerLeft.inputConfig.createInputQueue()

        thresholds = dai.FeatureTrackerConfig.CornerDetector.Thresholds()
        thresholds.initialValue = cv2.getTrackbarPos('harris_score','Features')

        cornerDetector.thresholds = thresholds
        featureTrackerLeft.initialConfig.setCornerDetector(cornerDetector)

        pipeline.start()
        while pipeline.isRunning():
            trackedFeaturesLeft = outputFeatureQueue.get().trackedFeatures

            outputPassthroughImage : dai.ImgFrame = outputFeaturePassthroughQueue.get()

            passthroughImage = outputPassthroughImage.getCvFrame()
            passthroughImage = cv2.cvtColor(passthroughImage, cv2.COLOR_GRAY2BGR)


            drawFeatures(passthroughImage, trackedFeaturesLeft)

            # Show the frame
            cv2.imshow("Features", passthroughImage)
            key = cv2.waitKey(1)
      
            if key == ord('q'):
                break
            elif key == ord('m'):
                cfg = dai.FeatureTrackerConfig()
                cornerDetector = dai.FeatureTrackerConfig.CornerDetector()
                cornerDetector.numMaxFeatures = cv2.getTrackbarPos('numMaxFeatures', 'Features')

                thresholds = dai.FeatureTrackerConfig.CornerDetector.Thresholds()
                thresholds.initialValue = cv2.getTrackbarPos('harris_score','Features')
                cornerDetector.thresholds = thresholds

                cfg.setCornerDetector(cornerDetector)
                cfg.setMotionEstimator(motionEstimator)

                if motionEstimator.enable == False:
                    motionEstimator.enable = True
                    cfg.setMotionEstimator(motionEstimator)
                    print("Enabling motionEstimator")
                else:
                    motionEstimator.enable = False
                    cfg.setMotionEstimator(motionEstimator)
                    print("Disabling motionEstimator")

                inputConfigQueue.send(cfg)
