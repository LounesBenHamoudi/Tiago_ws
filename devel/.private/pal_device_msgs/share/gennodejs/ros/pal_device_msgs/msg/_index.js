
"use strict";

let LedBlinkParams = require('./LedBlinkParams.js');
let LedEffectParams = require('./LedEffectParams.js');
let LedEffectViaTopicParams = require('./LedEffectViaTopicParams.js');
let LedFlowParams = require('./LedFlowParams.js');
let LedRainbowParams = require('./LedRainbowParams.js');
let LedProgressParams = require('./LedProgressParams.js');
let LedGroup = require('./LedGroup.js');
let BatteryState = require('./BatteryState.js');
let LedDataArrayParams = require('./LedDataArrayParams.js');
let LedPreProgrammedParams = require('./LedPreProgrammedParams.js');
let LedFadeParams = require('./LedFadeParams.js');
let LedFixedColorParams = require('./LedFixedColorParams.js');
let Bumper = require('./Bumper.js');
let DoTimedLedEffectActionGoal = require('./DoTimedLedEffectActionGoal.js');
let DoTimedLedEffectActionResult = require('./DoTimedLedEffectActionResult.js');
let DoTimedLedEffectFeedback = require('./DoTimedLedEffectFeedback.js');
let DoTimedLedEffectActionFeedback = require('./DoTimedLedEffectActionFeedback.js');
let DoTimedLedEffectAction = require('./DoTimedLedEffectAction.js');
let DoTimedLedEffectResult = require('./DoTimedLedEffectResult.js');
let DoTimedLedEffectGoal = require('./DoTimedLedEffectGoal.js');

module.exports = {
  LedBlinkParams: LedBlinkParams,
  LedEffectParams: LedEffectParams,
  LedEffectViaTopicParams: LedEffectViaTopicParams,
  LedFlowParams: LedFlowParams,
  LedRainbowParams: LedRainbowParams,
  LedProgressParams: LedProgressParams,
  LedGroup: LedGroup,
  BatteryState: BatteryState,
  LedDataArrayParams: LedDataArrayParams,
  LedPreProgrammedParams: LedPreProgrammedParams,
  LedFadeParams: LedFadeParams,
  LedFixedColorParams: LedFixedColorParams,
  Bumper: Bumper,
  DoTimedLedEffectActionGoal: DoTimedLedEffectActionGoal,
  DoTimedLedEffectActionResult: DoTimedLedEffectActionResult,
  DoTimedLedEffectFeedback: DoTimedLedEffectFeedback,
  DoTimedLedEffectActionFeedback: DoTimedLedEffectActionFeedback,
  DoTimedLedEffectAction: DoTimedLedEffectAction,
  DoTimedLedEffectResult: DoTimedLedEffectResult,
  DoTimedLedEffectGoal: DoTimedLedEffectGoal,
};
