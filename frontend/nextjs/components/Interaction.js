import React, { useState, useEffect, useReducer, useCallback } from "react";
import {requestConversationResponse, requestSpeech} from "@helpers/apiRequests"
import { getTranscribedSpeech } from "@helpers/apiEventSources";
import Microphone from '@components/Microphone';
import HeadDisplay from './faces/HeadDisplay';
import getExpresionAUs from '@helpers/AUtransformers/Expressions';
import doBehavior from '@helpers/AUtransformers/Behaviors';
import {sourceVisemes, sourceFaceCommands} from "@helpers/apiEventSources";
import { set_dropdown } from "@helpers/controls";
import { behaviors, faces, expressions } from "@constants/choices";
import {head_settings, initialBrowAU, initialEyeAU, initialMouthAU} from "@constants/initialface"
import { conversationPrompts } from "@constants/prompts";
import Face from "./Face";

const Interaction = ({ classes }) => {
  const [conversationHistory, setConversationHistory] = useState([""]);
  const [audioPlaying, setAudioPlaying] = useState(false);
  const [latestSpeech, setLatestSpeech] = useState("");
  const [prompt, setPrompt] = useState("The following is a conversation with an AI assistant. The assistant is helpful, creative, clever, and very friendly.\n");
  const [botResponse, setBotResponse] = useState("");
  const [inputText, setInputText] = useState("");

  const [expression, setExpression] = useState("neutral");
  const [behavior, setBehavior] = useState("focused");
  const [display, setDisplay] = useState("qt");
  const [mouthAU, updateMouthAU] = useReducer((mouthAU, updates) => ({ ...mouthAU, ...updates }),initialMouthAU);
  const [eyeAU, updateEyeAU] = useReducer((eyeAU, updates) => ({ ...eyeAU, ...updates }),initialEyeAU);
  const [browAU, updateBrowAU] = useReducer((browAU, updates) => ({ ...browAU, ...updates }),initialBrowAU);

  
  // const [beginConversation, setBeginConversation] = useState(true);
  const [speakerVoice, setSpeakerVoice] = useState("Emma");
  const [speaker, setSpeaker] = useState("Chris");

  function mouthUpdater (AU) { updateMouthAU({ ...AU })}
  function browUpdater (AU) { updateBrowAU({ ...AU })}
  function eyeUpdater (AU) { updateEyeAU({ ...eyeAU, ...AU })}
  const eyeUpdaterWrapper = useCallback(eyeUpdater,[eyeAU])

  // Respond to human speech by displaying conversation history and the most recent speech
  function respondToHumanSpeech () {
    console.log("Latest speech: ", latestSpeech)
    if (latestSpeech) {console.log("latest speech evaluated to true")}
    if (latestSpeech !== "") {
      console.log("STT process: ", latestSpeech, " from ", speaker)
      setConversationHistory(oldData => [speaker+": "+latestSpeech, ...oldData ]);
      requestConversationResponse(speaker, latestSpeech, prompt, conversationHistory, setBotResponse)
    }
  }
  
  function respondToBotSpeech () {
    console.log("Bot response: ", botResponse)
    if (botResponse) {console.log("bot response evaluated to true")}
    if (botResponse !== "") {
      setConversationHistory(oldData => ["bot: "+botResponse, ...oldData ])
      setInputText("")
      requestSpeech(botResponse, setAudioPlaying, speakerVoice)
    }
  }

  function runExpression() {
    var [MouthAU, EyeAU, BrowAU] = getExpresionAUs(expression)
    mouthUpdater(MouthAU);
    browUpdater(BrowAU);
    eyeUpdaterWrapper(EyeAU)
  }

  function runBehaviors() {
    var count = 0
    var update_interval_ms = 500
    const interval = setInterval(() => {
      count = doBehavior(count, behavior, 
        browUpdater, mouthUpdater, getExpresionAUs, eyeUpdaterWrapper)}, 
      update_interval_ms);
    return () => clearInterval(interval);
  }

  useEffect(() => {sourceVisemes(mouthUpdater)} , []);
  useEffect(runExpression, [expression]);
  useEffect(runBehaviors, [behavior, eyeUpdaterWrapper]);

  useEffect(respondToBotSpeech,[botResponse])
  useEffect(() => {getTranscribedSpeech(setLatestSpeech)}, []);
  useEffect(respondToHumanSpeech,[latestSpeech])

  return (
    <div id="divid" className="Interaction">
      <header className="Interaction-header"></header>
      <Face />
      {/* <HeadDisplay face={display} head_settings={head_settings} eyeAU={eyeAU} browAU={browAU} mouthAU={mouthAU} />
      {set_dropdown("Face", display, setDisplay, faces)}
      {set_dropdown("Expression", expression, setExpression, expressions)}
      {set_dropdown("Behavior", behavior, setBehavior, behaviors)} */}
      <Microphone audioPlaying={audioPlaying}/>
      <label>Enter the starting prompt or select from the following preset options: <br></br>
            <textarea 
                cols={100}
                rows={3}
                value={prompt}
                onChange={(e) => setPrompt(e.target.value)}
            />
      </label><br></br>
      <label>Enter the current speakers name: <br></br>
            <textarea
                value={speaker}
                onChange={(e) => setSpeaker(e.target.value)}
            />
      </label><br></br>
      <label>Enter the text you would like to say (if not using the mic): <br></br>
            <textarea 
                cols={100}
                rows={2}
                value={inputText}
                onChange={(e) => setInputText(e.target.value)}
            /><br></br>
      <input type="button" value="Say it" onClick={() => setLatestSpeech(inputText)}/>
      </label><br></br>
      <div id="transcription">
          <h2>Transcribed Data:</h2>
          {conversationHistory.map((h,ind) => <p key={ind}>{h}</p>)}
      </div>
    </div>
  );
}

export default Interaction;