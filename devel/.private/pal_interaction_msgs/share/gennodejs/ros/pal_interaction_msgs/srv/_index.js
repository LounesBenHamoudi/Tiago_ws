
"use strict";

let GetSpeechDuration = require('./GetSpeechDuration.js')
let recognizerService = require('./recognizerService.js')
let ASRService = require('./ASRService.js')
let SoundLocalisationService = require('./SoundLocalisationService.js')

module.exports = {
  GetSpeechDuration: GetSpeechDuration,
  recognizerService: recognizerService,
  ASRService: ASRService,
  SoundLocalisationService: SoundLocalisationService,
};
