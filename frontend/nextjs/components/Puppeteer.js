import React, { useState } from "react";
import RobotControls from './Forms/robotControls';
import {requestFaceUpdate, requestSpeech} from "@helpers/apiRequests"


const Puppeteer = ({ setAudioPlaying=()=>{} }) => {

  const [speakerVoice, setSpeakerVoice] = useState("Aria");
  const [viseme, setViseme] = useState("");
  const [expression, setExpression] = useState("neutral");
  const [behavior, setBehavior] = useState("focused");
  const [textToSay, setTextToSay] = useState("This is an example of what I sound like when I am talking.");
  // const [audioPlaying, setAudioPlaying] = useState(false);

  function update_behavior(name){
    setBehavior(name)
    requestFaceUpdate(name, "behavior")
    console.log(name)
  }
  function update_viseme(name){
    setViseme(name)
    requestFaceUpdate(name, "viseme")
    console.log(name)
  }
  function update_expression(name){
    setExpression(name)
    requestFaceUpdate(name, "expression")
    console.log(name)
  }

  function ttsWrapper(text) {
    console.log("Will say"+text)
    requestSpeech(text, setAudioPlaying, speakerVoice)
    return false
  }



  return (
    <div className="Face">
      <header className="Face-header"></header>
      <RobotControls
        showForm={true} 
        do_tts={ttsWrapper} 
        textToSay={textToSay} 
        setTextToSay={setTextToSay} 
        behavior={behavior} 
        update_behavior={update_behavior} 
        expression={expression} 
        update_expression={update_expression} 
        speakerVoice={speakerVoice} 
        setSpeakerVoice={setSpeakerVoice} 
        viseme={viseme} 
        update_viseme={update_viseme} />
    </div>
  );
}

export default Puppeteer;