import './App.css';
import React, { useState, useEffect, useCallback } from "react";
import Mic from './mic/Microphone';
import Timer from './timer/timer';
import getDeadTime from './timer/utils';
import Walkthrough from './facilitator/walkthrough';
import FacilitatorControls from './facilitator/controls';
import SpeakerMonitor from './facilitator/speech';
import RobotControls from './robotControls';


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

  const get_bot_response = useCallback(human_input => {
    const text = human_input
    const speaker = participantSpeaker
    if (text) {
      return fetch(`//localhost:8000/api/bot_response?text=${encodeURIComponent(text)}&speaker=${encodeURIComponent(speaker)}&reset_conversation=${encodeURIComponent(beginConversation)}&director_condition=${encodeURIComponent(!condition)}`, { cache: 'no-cache' })
      .then(response => response.text())
      .then(message => {console.log(message); return message})
    }
  },[beginConversation, condition, participantSpeaker]
  )

  useEffect(() => {const es = new EventSource("http://localhost:8000/api/text_stream");
    es.addEventListener('open', () => {
      // console.log('SSE opened@!')
    });

    // faceControls should handle vizemes, eyeAU, browAU, mouthAU
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

    return () => {
      es.close();
    };
  }, [get_bot_response, latestSpeech, participantSpeaker, priorSpeaker]);


  /////// Play Speech ///////
  function do_tts(e) {
    console.log("Will say"+e)
    setPriorSpeaker("bot")
    const text = e
    const speaker_id = [speakerVoice]
    const style_wav = ""
    setTranscribedData(oldData => ["bot: "+text, <br></br>, ...oldData ])

    if (text) {
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
    return false
  }


  function set_face(name, type){
    const name_text = name
    const type_text = type
    return fetch(`//localhost:8000/api/face_presets?text=${encodeURIComponent(name_text)}&update_type=${encodeURIComponent(type_text)}`, { cache: 'no-cache' })
      .then(response => response.text())
      .then(message => {console.log(message)})
  }

  function update_behavior(name){
    setBehavior(name)
    set_face(name, "behavior")
    console.log(name)
  }

  function update_viseme(name){
    setViseme(name)
    set_face(name, "viseme")
    console.log(name)
  }

  function update_expression(name){
    setExpression(name)
    set_face(name, "expression")
    console.log(name)
  }

  function update_speaker(name){
    console.log("Updated speaker to "+name)
    setParticipantSpeaker(name)
  }

  function switch_condition(){setCondition(!condition)}



  return (
    <div className="App">
      <header className="App-header"></header>
      <Mic isRecording={isRecording}/>
      <button onClick={() => do_tts("Testing, 1, 2, 3. Can you all hear me?")}>Speech Test</button>
      
      <br></br>
      <button id="toggle" onClick={() => setWalkthroughToggle(!showWalkthrough)}>Show/Hide Study Walkthrough: </button>
      <div id="walkthrough" hidden={showWalkthrough}>
        <Walkthrough 
          setWalkthroughToggle={setWalkthroughToggle}
          do_tts={do_tts}
          setTimerDeadline={setTimerDeadline}
          switch_condition={switch_condition}
        />
      </div>

      <br></br>
      <Timer timerDeadline={timerDeadline} />

      <SpeakerMonitor
        update_speaker={update_speaker}
        participantSpeaker={participantSpeaker}
        latestSpeech={latestSpeech}
        classifications={classifications}
      />

      <FacilitatorControls
        do_tts={do_tts}
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
        do_tts={do_tts} 
        setTextToSay={setTextToSay} 
        textToSay={textToSay} 
        behavior={behavior} 
        update_behavior={update_behavior} 
        expression={expression} 
        update_expression={update_expression} 
        speakerVoice={speakerVoice} 
        setSpeakerVoice={setSpeakerVoice} 
        viseme={viseme} 
        update_viseme={update_viseme} 
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