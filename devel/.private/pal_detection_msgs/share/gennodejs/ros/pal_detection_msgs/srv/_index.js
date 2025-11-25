
"use strict";

let StopEnrollment = require('./StopEnrollment.js')
let SelectTexturedObject = require('./SelectTexturedObject.js')
let SetDatabase = require('./SetDatabase.js')
let Recognizer = require('./Recognizer.js')
let StartEnrollment = require('./StartEnrollment.js')
let ChangeObjectRecognizerModel = require('./ChangeObjectRecognizerModel.js')
let AddTexturedObject = require('./AddTexturedObject.js')

module.exports = {
  StopEnrollment: StopEnrollment,
  SelectTexturedObject: SelectTexturedObject,
  SetDatabase: SetDatabase,
  Recognizer: Recognizer,
  StartEnrollment: StartEnrollment,
  ChangeObjectRecognizerModel: ChangeObjectRecognizerModel,
  AddTexturedObject: AddTexturedObject,
};
