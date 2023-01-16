import './App.css';
import React, { useReducer, useState, useEffect, useCallback } from "react";
import { w3cwebsocket as W3CWebSocket } from "websocket";

import { ReactMic } from './react-mic-clone';
import Head from './rendering/head';
import getAUs from './animation/processVisemes';
import getExpresionAUs from './animation/proccessExpressions';

import {EyeForm, BrowForm, MouthForm} from './control/Form';
import {randomFace, boredEyes, focusedEyes} from './animation/proccessBehaviors';

const stt_socket = new W3CWebSocket('ws://localhost:8000/api/stt');
const FACILITOR_INTRO = "Hello and welcome to each of you! Thank you for taking the time to be here today. During today’s support group session I invite you to share your thoughts and experiences with each other, and I hope that you will listen to each other and respond with empathy and compassion. To begin with, I’d like to start with a round of introductions. My name is Q.T. and I am training to be a support group facilitator at the Interaction Lab at USC. I am learning how to facilitate support groups so I can help people support each other better. Who would like to go next?"
const BOT_START = "Hi, I am your virtual personal mental health assistant. How are you doing today?"

const App = ({ classes }) => {
  // Initial positions
  const positions = {
    right_eye: {x:22, y:-10},
    left_eye: {x:-22, y:-10},
    right_brow: {x:20, y:-14, auScaler: 4, flip:-1},
    left_brow: {x:-20, y:-14, auScaler: 4, flip:1},
    mouth: {x:0, y:16, auScaler: 6},
  }
  const initialMouthAU = {
    au10: 0,
    au12: 0.2,
    au13: 0,
    au14: 0,
    au15: 0,
    au16: 0,
    au17: 0,
    au18: 0,
    au20: 0,
    au22: 0,
    au23: 0,
    au24: 0,
    au25: 0,
    au26: 0,
    au27: 0,
    au28: 0
  };
  const initialBrowAU = {
    au1: 0,
    au2: 0,
    au4: 0,
  };  
  const initialEyeAU = {
    au5: 0,
    au6: 0,
    au7: 0,
    au41: 0,
    au42: 0,
    au43: 0,
    au44: 0.6,
    au45: 0,
    au61: 0,
    au62: 0,
    au63: 0,
    au64: 0,
  };  

  // Variables with state
  const [beginConversation, setBeginConversation] = useState(true);
  const [textToSay, setTextToSay] = useState("This is an example of what I sound like when I am talking.");
  const [botResponse, setBotResponse] = useState("This is the bot response");
  const [treeResponse, setTreeResponse] = useState("This is the tree response");
  const [isRecording, setIsRecording] = useState(false);
  const [showForm, setFormToggle] = useState(false);
  const [expression, setExpression] = useState("");
  const [speakerVoice, setSpeakerVoice] = useState("p306");
  const [participantSpeaker, setParticipantSpeaker] = useState("Human");
  const [viseme, setViseme] = useState("");
  const [transcribedData, setTranscribedData] = useState([""]);
  
  const [mouthAU, updateMouthAU] = useReducer((mouthAU, updates) => ({ ...mouthAU, ...updates }),initialMouthAU);
  const [eyeAU, updateEyeAU] = useReducer((eyeAU, updates) => ({ ...eyeAU, ...updates }),initialEyeAU);
  const [browAU, updateBrowAU] = useReducer((browAU, updates) => ({ ...browAU, ...updates }),initialBrowAU);
  function mouthUpdater (AU) { updateMouthAU({ ...AU })}
  function browUpdater (AU) { updateBrowAU({ ...AU })}
  const eyeUpdater = useCallback(function eyeUpdaterInner (AU) { updateEyeAU({ ...eyeAU, ...AU })},[eyeAU])

  // Visemes are separated from the rest of the face control
  // so speaking and expression (aside from the lips) can happen together
  useEffect(() => {const es = new EventSource("http://localhost:8000/api/visemes");
    es.addEventListener('open', () => {
      // console.log('SSE opened@!')
    });

    // faceControls should handle vizemes, eyeAU, browAU, mouthAU
    es.addEventListener('viseme', (e) => {
      var auToUpdate = getAUs(e.data);
      mouthUpdater(auToUpdate);
    });

    es.addEventListener('error', (e) => {
      console.error('Error: ',  e);
    });

    return () => {
      es.close();
    };
  }, []);

  useEffect(() => {const es = new EventSource("http://localhost:8000/api/faceControl");
    es.addEventListener('open', () => {
      console.log('SSE opened@!')
    });

    // faceControls should handle emotion, eyeAU, browAU, mouthAU
    es.addEventListener('expression', (e) => {
      console.log(e.data)
      var [MouthAU, EyeAU, BrowAU] = getExpresionAUs(e.data)
      mouthUpdater(MouthAU);
      browUpdater(BrowAU);
      eyeUpdater(EyeAU);
    });

    es.addEventListener('error', (e) => {
      console.error('Error: ',  e);
    });

    return () => {
      es.close();
    };
  });
  const [behavior, setBehavior] = useState("focused");
  useEffect(() => {
    var count = 0
    var update_interval_ms = 500
    const interval = setInterval(() => {
        switch (behavior)
        {
            case "bored":
                count = boredEyes(count, 10, eyeUpdater, mouthUpdater)
                break;
            case "focused":
                count = focusedEyes(count, 20, eyeUpdater, mouthUpdater)
                break;
            case "random":
                count = randomFace(count, 6, eyeUpdater, browUpdater, mouthUpdater, getExpresionAUs)
                break;
            default:
                break;
        }
    }, update_interval_ms);
    return () => clearInterval(interval);
  }, [behavior, eyeUpdater]);


  /////// Socket Handler ///////
  stt_socket.onopen = () => {console.log({ event: 'onopen' })};
  stt_socket.onclose = () => {console.log({ event: 'onclose' })}
  stt_socket.onerror = (error) => {console.log({ event: 'onerror', error })}
  function onData(recordedBlob) {stt_socket.send(recordedBlob)}
  stt_socket.onmessage = (message) => {
      const received = message.data
      if (received) {
          console.log(received)
          process_response(received)
     }
  }

  /////// Process response fromm STT input ///////
  async function process_response(received) {
    setTranscribedData(oldData => [participantSpeaker+received,  <br></br>, ...oldData ])
    const raw_res = await get_bot_response(received)
    const respArray = raw_res.split("&&&")
    setTreeResponse(respArray[0])
    setBotResponse(respArray[1])
    // setTextToSay(bot_res)
    // do_tts(bot_res)
    setBeginConversation(false)
  }
  async function get_bot_response(human_input) {
    const text = human_input
    const speaker = participantSpeaker
    if (text) {
      return fetch(`//localhost:8000/api/bot_response?text=${encodeURIComponent(text)}&speaker=${encodeURIComponent(speaker)}&reset_conversation=${encodeURIComponent(beginConversation)}`, { cache: 'no-cache' })
      .then(response => response.text())
      .then(message => {console.log(message); return message})
    }
  }

  /////// Fetch and Play Speech ///////
  function do_tts(e) {
    console.log("Will say"+e)
    const text = e
    const speaker_id = [speakerVoice]
    const style_wav = ""
    setTranscribedData(oldData => ["bot: "+text, <br></br>, ...oldData ])

    if (text) {
        fetch(`//localhost:8000/api/tts?text=${encodeURIComponent(text)}&speaker_id=${encodeURIComponent(speaker_id)}&style_wav=${encodeURIComponent(style_wav)}`, { cache: 'no-cache' })
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

  /////// Handle Bot Face and Recorder ///////
  function startBot() {
    // setTranscribedData(BOT_START);
    do_tts(FACILITOR_INTRO)
  }

  const handleFormSubmit = (event) => {
    console.log("Form is submitted")
    event.preventDefault();

    var auToUpdate = getAUs(viseme)
    mouthUpdater(auToUpdate)

    var [MouthAU, EyeAU, BrowAU] = getExpresionAUs(expression)
    mouthUpdater(MouthAU)
    browUpdater(BrowAU)
    eyeUpdater(EyeAU)
    do_tts(textToSay)
  }


  return (
    <div className="App">
      <header className="App-header">
      </header>

      <div id="robot-container">
          <div id="start-button">
            <button onClick={startBot} >Click To Start Conversing With The Bot!</button>
          </div>
        <div id="bot" className="neutral">
          <Head position={positions} eyeAU={eyeAU} browAU={browAU} mouthAU={mouthAU} />
        </div>
      </div>

      <div id="listenbox">
        <div id="mic">
          <ReactMic
            record={isRecording}
            onData={onData}
            sampleRate={16000}
            timeSlice={5000}
            className="sound-wave"
            strokeColor="#fff"
            backgroundColor="#000" 
            height={100}
            width={320} />
        </div>      
      </div>
      <p>
        Text to be said by the robot:<br></br>
        Bot: {botResponse}<br></br>
        Tree: {treeResponse}<br></br>
      </p>
      <button onClick={() => do_tts(botResponse)}>Say Bot Response</button>
      <button onClick={() => do_tts(treeResponse)}>Say Tree Response</button>
      <button onClick={() => do_tts(textToSay)}>Say Form Text</button>
      <br></br>
      <br></br>
      <button onClick={() => do_tts("Yes, that is correct.")}>Say Yes</button>
      <button onClick={() => do_tts("No, that is not correct.")}>Say No</button>
      <button onClick={() => do_tts("I am unsure how to answer that, sorry.")}>Say Unsure</button>
      <br></br>
      <br></br>
      <div onChange={(e) => setParticipantSpeaker(e.target.value)}>
        <input type="radio" value="Chris" name="gender" /> Chris
        <input type="radio" value="Libby" name="gender" /> Libby
        <input type="radio" value="Sasha" name="gender" /> Sasha
        <input type="radio" value="Human" name="gender" /> Default
      </div>


      <button id="toggle" onClick={() => setFormToggle(!showForm)}>Show/Hide Form Input:</button>

      <div id="formcontent" hidden={showForm}>

        <form onSubmit={handleFormSubmit}>
        <label>Enter the text you would like the robot to say:
            <textarea 
              cols={100}
              rows={4}
              // type="text" 
              value={textToSay}
              onChange={(e) => setTextToSay(e.target.value)}
            />
          </label>
          <br></br>
          <br></br>
          <br></br>
          <label>Behavior:
          <select value={behavior} 
              multiple={false}
              onChange={(e) => setBehavior(e.target.value)}>
              <option value="focused">focused</option>
              <option value="random">random</option>
              <option value="bored">bored</option>
            </select>
          </label>
          <label>Expression:
          <select value={expression} 
              multiple={false}
              onChange={(e) => setExpression(e.target.value)}>
              <option value="joy">joy</option>
              <option value="sad">sad</option>
              <option value="surprise">surprise</option>
              <option value="neutral">neutral</option>
            </select>
          </label>
          <label> Voice:
            <select value={speakerVoice} 
              multiple={false}
              onChange={(e) => setSpeakerVoice(e.target.value)}>
              <option value="p267">British Male m</option>
              <option value="p330">British Male s</option>
              <option value="p312">British Male f</option>
              <option value="p287">British Male d</option>
              <option value="p303">British Female s</option>
              <option value="p306">British Female m</option>
              <option value="p295">America Female s</option>
              <option value="p217">America Male s</option>
              <option value="p230">America Male f</option>
              <option value="p313">Male f</option>
            </select>
          </label>
          <br></br>          
          <br></br>
          <div hidden={true}>
          <label>OR Enter viseme to show:
            <input 
              type="text" 
              value={viseme}
              onChange={(e) => setViseme(e.target.value)}
            />
          </label>
          </div>
          <br></br>
          <input type="submit" />
          <br></br>
          <br></br>
        </form>
        <div id="AU control" hidden={true}>
          <EyeForm v={eyeAU} f={eyeUpdater}/>
          <BrowForm v={browAU} f={browUpdater}/>
          <MouthForm v={mouthAU} f={mouthUpdater}/>
        </div>
      </div>
      <div>
        <p>Transcribed Data: <br></br>
          {transcribedData}</p>
      </div>
    </div>
  );
}

export default App;