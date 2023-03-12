import React, { useState } from "react";
import RobotControls from './Forms/robotControls';
import getDeadTime from './timer/utils';
import {requestFaceUpdate, requestSpeech} from "@helpers/apiRequests"


const Controller = ({ classes }) => {

  // Variables with state
  const [timerDeadline, setTimerDeadline] = useState("")
  const [timerLength, setTimerLength] = useState('25');
  const [speakerVoice, setSpeakerVoice] = useState("p270");
  // const [showForm, setFormToggle] = useState(false);
  const [viseme, setViseme] = useState("");
  const [expression, setExpression] = useState("");
  const [behavior, setBehavior] = useState("focused");
  const [textToSay, setTextToSay] = useState("This is an example of what I sound like when I am talking.");
  const [isRecording, setIsRecording] = useState(false);

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
    requestSpeech(text, setIsRecording, speakerVoice)
    return false
  }



  return (
    <div className="Face">
      <header className="Face-header"></header>
      <RobotControls
        setTimerDeadline={setTimerDeadline}
        getDeadTime={getDeadTime}
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
        update_viseme={update_viseme} 
        timerLength={timerLength} 
        setMinuteGoal={setTimerLength}/>
    </div>
  );
}

export default Controller;