import './App.css';
import React, { useState, useEffect, useCallback } from "react";
import Mic from './components/mic/Microphone';
import Timer from './components/timer/timer';
import getDeadTime from './components/timer/utils';
import Walkthrough from './components/facilitator/walkthrough';
import FacilitatorControls from './components/facilitator/controls';
import SpeakerMonitor from './components/facilitator/speech';
import RobotControls from './components/robotControls';


const App = ({ classes }) => {
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
  const [participantSpeaker, setParticipantSpeaker] = useState("");
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

  function requestBotReponse (human_input) {
    const text = human_input
    const speaker = participantSpeaker
    if (text) {
      return fetch(`//localhost:8000/api/bot_response?text=${encodeURIComponent(text)}&speaker=${encodeURIComponent(speaker)}&reset_conversation=${encodeURIComponent(beginConversation)}&director_condition=${encodeURIComponent(!condition)}`, { cache: 'no-cache' })
      .then(response => response.text())
      .then(message => {console.log(message);})
    }
  }
  function requestSpeech (e) {
    const text = e
    const speaker_id = [speakerVoice]
    const style_wav = ""
    fetch(`//localhost:8000/api/speech?text=${encodeURIComponent(text)}&speaker_id=${encodeURIComponent(speaker_id)}&style_wav=${encodeURIComponent(style_wav)}`, { cache: 'no-cache' })
      .then(function (res) {
          if (!res.ok) throw Error(res.statusText)
          return res.blob()})
      .then(function (blob) {
          const audioUrl = URL.createObjectURL(blob)
          const audio = new Audio(audioUrl)
          setIsRecording(false)
          audio.play();
          audio.addEventListener('ended', () => {setIsRecording(true)});})
      .catch(function (err) {
    })
  }
  function requestFaceCommand(name, type){
    const name_text = name
    const type_text = type
    return fetch(`//localhost:8000/api/face_presets?text=${encodeURIComponent(name_text)}&update_type=${encodeURIComponent(type_text)}`, { cache: 'no-cache' })
      .then(response => response.text())
      .then(message => {console.log(message)})
  }
  function sourceTextStream () {
    const es = new EventSource("http://localhost:8000/api/text_stream");
    es.addEventListener('open', () => {});
  
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

        if (full_speech.length > 20) {
          console.log(full_speech, full_speech.length)
          get_bot_response(full_speech);
        }
      };
    });
  
    es.addEventListener("bot_response", (e) => {
      let bot_says = e.data
      setBotResponse(bot_says);
    });
  
    es.addEventListener("facilitator_response", (e) => {
      let facilitator_says = e.data
      setFacilitatorResponse(facilitator_says);
    });
  
    es.addEventListener("classifications", (e) => {
      let classifications = e.data
      setClassifications(classifications);
    });

    es.addEventListener('error', (e) => {
      console.error('Error: ',  e);
    });
  
    return () => {es.close();};
  }
  function switch_condition(){setCondition(!condition)}
  /////// Play Speech ///////
  function ttsWrapper(e) {
    console.log("Will say"+e)
    setPriorSpeaker("bot")
    if (e) {
      requestSpeech(e)
      setTranscribedData(oldData => ["bot: "+e, <br></br>, ...oldData ])
    }
    return false
  }

  const get_bot_response = useCallback(requestBotReponse,[beginConversation, condition, participantSpeaker])

  useEffect(sourceTextStream, [get_bot_response, latestSpeech, participantSpeaker, priorSpeaker]);

  useEffect (() => {requestFaceCommand(behavior, "behavior")},[behavior])
  useEffect (() => {requestFaceCommand(viseme, "viseme")},[viseme])
  useEffect (() => {requestFaceCommand(expression, "expression")},[expression])

  return (
    <div className="App">
      <header className="App-header"></header>
      <Mic isRecording={isRecording}/>
      <button onClick={() => ttsWrapper("Testing, 1, 2, 3. Can you all hear me?")}>Speech Test</button>
      
      <br></br>
      <button id="toggle" onClick={() => setWalkthroughToggle(!showWalkthrough)}>Show/Hide Study Walkthrough: </button>
      <div id="walkthrough" hidden={showWalkthrough}>
        <Walkthrough 
          setWalkthroughToggle={setWalkthroughToggle}
          do_tts={ttsWrapper}
          setTimerDeadline={setTimerDeadline}
          switch_condition={switch_condition}
        />
      </div>

      <br></br>
      <Timer timerDeadline={timerDeadline} />

      <SpeakerMonitor
        update_speaker={setParticipantSpeaker}
        participantSpeaker={participantSpeaker}
        latestSpeech={latestSpeech}
        classifications={classifications}
      />

      <FacilitatorControls
        do_tts={ttsWrapper}
        participantSpeaker={participantSpeaker}
        condition={condition}
        botResponse={botResponse}
        facilitatorResponse={facilitatorResponse}
      />


      <button id="toggle" onClick={() => setFormToggle(!showForm)}>Show/Hide Form Input:</button>
      <RobotControls
        setTimerDeadline={setTimerDeadline}
        getDeadTime={getDeadTime}
        showForm={showForm} 
        do_tts={ttsWrapper} 
        textToSay={textToSay} 
        setTextToSay={setTextToSay} 
        behavior={behavior} 
        setBehavior={setBehavior} 
        expression={expression} 
        setExpression={setExpression} 
        speakerVoice={speakerVoice} 
        setSpeakerVoice={setSpeakerVoice} 
        viseme={viseme} 
        setViseme={setViseme} 
        timerLength={timerLength} 
        setMinuteGoal={setTimerLength}
      />

      <div id="transcription">
        <h2>Transcribed Data:</h2>
        <p>{transcribedData}</p>
      </div>
    </div>
  );
}

export default App;