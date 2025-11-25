
"use strict";

let HogDetection = require('./HogDetection.js');
let DetectedObject = require('./DetectedObject.js');
let Gesture = require('./Gesture.js');
let FaceDetection = require('./FaceDetection.js');
let HeadPanTilt = require('./HeadPanTilt.js');
let DetectedPerson = require('./DetectedPerson.js');
let FollowMeResponse = require('./FollowMeResponse.js');
let FaceDetections = require('./FaceDetections.js');
let HogDetections = require('./HogDetections.js');
let LegDetections = require('./LegDetections.js');
let Rectangle = require('./Rectangle.js');
let FaceRecognitionFeedback = require('./FaceRecognitionFeedback.js');
let FaceRecognitionGoal = require('./FaceRecognitionGoal.js');
let FaceRecognitionActionGoal = require('./FaceRecognitionActionGoal.js');
let FaceRecognitionAction = require('./FaceRecognitionAction.js');
let FaceRecognitionActionFeedback = require('./FaceRecognitionActionFeedback.js');
let FaceRecognitionResult = require('./FaceRecognitionResult.js');
let FaceRecognitionActionResult = require('./FaceRecognitionActionResult.js');

module.exports = {
  HogDetection: HogDetection,
  DetectedObject: DetectedObject,
  Gesture: Gesture,
  FaceDetection: FaceDetection,
  HeadPanTilt: HeadPanTilt,
  DetectedPerson: DetectedPerson,
  FollowMeResponse: FollowMeResponse,
  FaceDetections: FaceDetections,
  HogDetections: HogDetections,
  LegDetections: LegDetections,
  Rectangle: Rectangle,
  FaceRecognitionFeedback: FaceRecognitionFeedback,
  FaceRecognitionGoal: FaceRecognitionGoal,
  FaceRecognitionActionGoal: FaceRecognitionActionGoal,
  FaceRecognitionAction: FaceRecognitionAction,
  FaceRecognitionActionFeedback: FaceRecognitionActionFeedback,
  FaceRecognitionResult: FaceRecognitionResult,
  FaceRecognitionActionResult: FaceRecognitionActionResult,
};
