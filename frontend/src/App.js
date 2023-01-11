import './App.css';
import React, { useReducer, useState, useEffect } from "react";
import { w3cwebsocket as W3CWebSocket } from "websocket";

import { ReactMic } from './react-mic-clone';
import Head from './rendering/head';
import getAUs from './animation/processVisemes';
import getExpresionAUs from './animation/proccessExpressions';

import {EyeForm, BrowForm, MouthForm} from './control/Form';
import DoBehavior from './animation/proccessBehaviors';

const stt_socket = new W3CWebSocket('ws://localhost:8000/api/stt');

const App = ({ classes }) => {
  const BOT_START = "Hi, I am your virtual personal mental health assistant. How are you doing today?"
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
  const [transcribedData, setTranscribedData] = useState([""]);
  const [speakerVoice, setSpeakerVoice] = useState(["p287"]);
  const [isRecording, setIsRecording] = useState(false);
  const [beginConversation, setBeginConversation] = useState(true);
  const [viseme, setViseme] = useState("");
  const [expression, setExpression] = useState("");
  const [showForm, setFormToggle] = useState(false);
  const [textToSay, setTextToSay] = useState("Don't put words in my mouth like that.");
  const [mouthAU, updateMouthAU] = useReducer((mouthAU, updates) => ({ ...mouthAU, ...updates }),initialMouthAU);
  const [eyeAU, updateEyeAU] = useReducer((eyeAU, updates) => ({ ...eyeAU, ...updates }),initialEyeAU);
  const [browAU, updateBrowAU] = useReducer((browAU, updates) => ({ ...browAU, ...updates }),initialBrowAU);
  
  
  // TODO set up eyes/brows for individual control

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

    // TODO implement message passing with dictionary of AUs
      // not currently available
    // es.addEventListener('eye_aus', (e) => {
    //   console.log(e.data)
    //   eyeUpdater(e.data)
    // });

    // es.addEventListener('mouth_aus', (e) => {
    //   console.log(e.data)
    //   mouthUpdater(e.data)
    // });

    // es.addEventListener('brow_aus', (e) => {
    //   console.log(e.data)
    //   browUpdater(e.data)
    // });

    es.addEventListener('error', (e) => {
      console.error('Error: ',  e);
    });

    return () => {
      es.close();
    };
  });

  function mouthUpdater (AU) { updateMouthAU({ ...AU })}
  function browUpdater (AU) { updateBrowAU({ ...AU })}
  function eyeUpdater (AU) { updateEyeAU({ ...eyeAU, ...AU })}

  const handleFormSubmit = (event) => {
    event.preventDefault();

    var auToUpdate = getAUs(viseme)
    mouthUpdater(auToUpdate)

    var [MouthAU, EyeAU, BrowAU] = getExpresionAUs(expression)
    mouthUpdater(MouthAU)
    browUpdater(BrowAU)
    eyeUpdater(EyeAU)
    do_tts(textToSay)
  }

  /////// Socket Handler ///////
  stt_socket.onopen = () => {
      console.log({ event: 'onopen' })
  };
  stt_socket.onclose = () => {
    console.log({ event: 'onclose' })
  }
  stt_socket.onerror = (error) => {
      console.log({ event: 'onerror', error })
  }
  function onData(recordedBlob) {
    stt_socket.send(recordedBlob)
  }
  stt_socket.onmessage = (message) => {
      const received = message.data
      if (received) {
          console.log(received)
          process_response(received)
     }
  }


  /////// Process response fromm STT input ///////
  async function process_response(received) {
    setTranscribedData(oldData => [...oldData, <br></br>, received])
    const bot_res = await get_bot_response(received)

    setTextToSay(bot_res)

    setBeginConversation(false)
  }
  async function get_bot_response(human_input) {
    const text = human_input
    if (text) {
      return fetch(`//localhost:8000/api/bot_response?text=${encodeURIComponent(text)}&reset_conversation=${encodeURIComponent(beginConversation)}`, { cache: 'no-cache' })
      .then(response => response.text())
      .then(message => {console.log(message); return message})
    }
  }

  /////// Fetch and Play Speech ///////
  function do_tts(e) {
    console.log("Will say"+e)
    const text = e
    const speaker_id = speakerVoice
    const style_wav = ""
    setTranscribedData(oldData => [...oldData, <br></br>, text])

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
    setTranscribedData(BOT_START);
    do_tts(BOT_START)
  }

  DoBehavior("focused", mouthUpdater, browUpdater, eyeUpdater, getExpresionAUs)



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
      <p>{textToSay}</p>
      <button id="sayit" onClick={() => do_tts(textToSay)}>* * * * * Say it!* * * * *</button>
      <br></br>
      <br></br>
      <button id="toggle" onClick={() => setFormToggle(!showForm)}>Show/Hide Form Input:</button>

      <div id="formcontent" hidden={showForm}>

        <form onSubmit={handleFormSubmit}>
        <label>Enter the text you would like the robot to say:
            <input 
              size={100}
              type="text" 
              value={textToSay}
              onChange={(e) => setTextToSay(e.target.value)}
            />
          </label>
          <br></br>
          <br></br>
          <p>Possible expressions: anger, joy, sad, fear, disgust, surprise, neutral; or any hypthenated combo</p>
          <label>Enter expression to show:
            <input 
              type="text" 
              value={expression}
              onChange={(e) => setExpression(e.target.value)}
            />
          </label>
          <br></br>
          <br></br>
          <label>Choose robot voice:
          <select value={speakerVoice} 
            multiple={true}
            onChange={(event) => {setSpeakerVoice(event.target.value)}}>
            <option value="p267">British Male m</option>
            <option value="p330">British Male s</option>
            <option value="p312">British Male f</option>
            <option value="p287">British Male d</option>
            <option value="p303">British Female s</option>
            <option value="p306">British Female m</option>
            <option value="p295">America Female s</option>
            <option value="p217">America Male s</option>
            <option value="p230">America Male f</option>
            <option value="p313">? Male f</option>
          </select>
          </label>
          <br></br>          
          <br></br>
          <label>OR Enter viseme to show:
            <input 
              type="text" 
              value={viseme}
              onChange={(e) => setViseme(e.target.value)}
            />
          </label>
          <br></br>
          <input type="submit" />
          <br></br>
          <br></br>
        </form>
        <EyeForm v={eyeAU} f={eyeUpdater}/>
        <BrowForm v={browAU} f={browUpdater}/>
        <MouthForm v={mouthAU} f={mouthUpdater}/>
      </div>
      <div>
        <p>Transcribed Data: 
          {transcribedData}</p>
      </div>
    </div>
  );
}

export default App;