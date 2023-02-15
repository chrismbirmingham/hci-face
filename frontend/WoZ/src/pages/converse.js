import React, { useState, useEffect, useCallback } from "react";
import Mic from '../components/mic/Microphone';
import RobotControls from '../components/robotControls';
import Link from 'next/link';


const WoZ = ({ classes }) => {
  // Variables with state
  const [beginConversation, setBeginConversation] = useState(true);
  const [textToSay, setTextToSay] = useState("This is an example of what I sound like when I am talking.");
  const [botResponse, setBotResponse] = useState("This is the bot response");
  const [facilitatorResponse, setFacilitatorResponse] = useState("This is the Facilitator response");

  const [isRecording, setIsRecording] = useState(true);
  const [showForm, setFormToggle] = useState(false);
  const [showWalkthrough, setWalkthroughToggle] = useState(false);
  const [condition, setCondition] = useState(false);

  const [speakerVoice, setSpeakerVoice] = useState("p270");
  const [participantSpeaker, setParticipantSpeaker] = useState("Chris");
  const [priorSpeaker, setPriorSpeaker] = useState("");
  const [latestSpeech, setLatestSpeech] = useState("");
  const [classifications, setClassifications] = useState("");
  const [transcribedData, setTranscribedData] = useState([""]);
  
  // The state for face controls
  const [viseme, setViseme] = useState("");
  const [expression, setExpression] = useState("");
  const [behavior, setBehavior] = useState("focused");
  
	// The state for our timer
    const [timerDeadline, setTimerDeadline] = useState("")
    const [timerLength, setTimerLength] = useState('25');


  // API calling functions
  function getTextStream() {
    const es = new EventSource("http://localhost:8000/api/text_stream");
    es.addEventListener('open', () => {
    });

    es.addEventListener('human_speech', (e) => {
      let speech = e.data;
      if (speech.length>0){
        let full_speech = speech
        console.log("STT process: ", speech, " from ", participantSpeaker)
        if (participantSpeaker === priorSpeaker){
          full_speech = latestSpeech + " " + speech;
        }
        setLatestSpeech(full_speech);
        setTranscribedData(oldData => [participantSpeaker+":"+speech,  <br></br>, ...oldData ]);
        setPriorSpeaker(participantSpeaker);
        setClassifications("");
        setBeginConversation(false);

        if (full_speech.length > 2) {
          console.log(full_speech, full_speech.length)
          requestBotResponseCB(full_speech);
        }
      };
    });
  
    es.addEventListener("bot_response", (e) => {
      let bot_says = e.data
      setBotResponse(bot_says);
      setTranscribedData(oldData => ["bot:"+bot_says,  <br></br>, ...oldData ])
      requestSpeech(bot_says)
    });

    es.addEventListener('error', (e) => {
      console.error('Error: ',  e);
    });

    return () => {
      es.close();
    };
  }
  function requestFaceUpdate(name, type){
    const name_text = name
    const type_text = type
    return fetch(`//localhost:8000/api/face_presets?text=${encodeURIComponent(name_text)}&update_type=${encodeURIComponent(type_text)}`, { cache: 'no-cache' })
      .then(response => response.text())
      .then(message => {console.log(message)})
  }
  function requestBotResponse(human_input) {
    const text = human_input
    const speaker = participantSpeaker
    if (text) {
      return fetch(`//localhost:8000/api/bot_response?text=${encodeURIComponent(text)}&speaker=${encodeURIComponent(speaker)}&reset_conversation=${encodeURIComponent(beginConversation)}&director_condition=${encodeURIComponent(!condition)}`, { cache: 'no-cache' })
      .then(response => response.text())
      .then(message => {console.log(message); return message})
    }
  }
  function requestSpeech(text){
    const speaker_id = [speakerVoice]
    const style_wav = ""
    fetch(`//localhost:8000/api/speech?text=${encodeURIComponent(text)}&speaker_id=${encodeURIComponent(speaker_id)}&style_wav=${encodeURIComponent(style_wav)}`, { cache: 'no-cache' })
    .then(function (res) {
        if (!res.ok) throw Error(res.statusText)
        return res.blob()
    }).then(function (blob) {
        const audioUrl = URL.createObjectURL(blob)
        const audio = new Audio(audioUrl)
        setIsRecording(false)
        audio.play();
        audio.addEventListener('ended', () => {setIsRecording(true)});
    }).catch(function (err) {
  })
  }
  const requestBotResponseCB = useCallback(requestBotResponse,[beginConversation, condition, participantSpeaker])

  /////// Play Speech ///////
  function ttsWrapper(text) {
    console.log("Will say"+text)
    setPriorSpeaker("bot")
    setTranscribedData(oldData => ["bot: "+text, <br></br>, ...oldData ])
    requestSpeech(text)
    return false
  }

  // Wrappers for updating variables
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


  useEffect(getTextStream, [requestBotResponseCB, latestSpeech, participantSpeaker, priorSpeaker]);

  return (
    <div className="WoZ">
      <header className="WoZ-header"></header>
      <Mic isRecording={isRecording}/>
      <button onClick={() => ttsWrapper("Testing, 1, 2, 3. Can you all hear me?")}>Speech Test</button>

      <button id="toggle" onClick={() => setFormToggle(!showForm)}>Show/Hide Form Input:</button>
      <RobotControls
        setTimerDeadline={setTimerDeadline}
        // getDeadTime={getDeadTime}
        showForm={showForm} 
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

      <div id="transcription">
        <h2>Transcribed Data:</h2>
        <p>{transcribedData}</p>
      </div>
      <Link href="/">Back to home</Link>
    </div>
  );
}

export default WoZ;