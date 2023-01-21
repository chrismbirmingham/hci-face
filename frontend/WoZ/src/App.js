import './App.css';
import React, { useState } from "react";
import { w3cwebsocket as W3CWebSocket } from "websocket";
import { ReactMic } from './react-mic-clone';


const stt_socket = new W3CWebSocket('ws://localhost:8000/api/stt');

const App = ({ classes }) => {


  // Variables with state
  const [beginConversation, setBeginConversation] = useState(true);
  const [textToSay, setTextToSay] = useState("This is an example of what I sound like when I am talking.");
  const [botResponse, setBotResponse] = useState("This is the bot response");
  const [isRecording, setIsRecording] = useState(false);
  const [showForm, setFormToggle] = useState(false);
  const [speakerVoice, setSpeakerVoice] = useState("p270");
  const [participantSpeaker, setParticipantSpeaker] = useState("");
  const [transcribedData, setTranscribedData] = useState([""]);
  
  const [viseme, setViseme] = useState("");
  const [expression, setExpression] = useState("");
  const [behavior, setBehavior] = useState("focused");
  

  /////// Socket Handler ///////
  stt_socket.onopen = () => {console.log({ event: 'onopen' })};
  stt_socket.onclose = () => {console.log({ event: 'onclose' })}
  stt_socket.onerror = (error) => {console.log({ event: 'onerror', error })}
  function onData(recordedBlob) {stt_socket.send(recordedBlob)}
  stt_socket.onmessage = (message) => {
      const received = message.data
      if (received) {
          console.log(received)
          process_stt(received)
     }
  }

  /////// Process STT input and get bot response ///////
  async function process_stt(received) {
    setTranscribedData(oldData => [participantSpeaker+":"+received,  <br></br>, ...oldData ])
    const raw_res = await get_bot_response(received)
    setBotResponse(raw_res)
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

  /////// Get and Play Speech ///////
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


  function set_face(name, type){
    const name_text = name
    const type_text = type
    return fetch(`//localhost:8000/api/face?text=${encodeURIComponent(name_text)}&update_type=${encodeURIComponent(type_text)}`, { cache: 'no-cache' })
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

  return (
    <div className="App">
      <header className="App-header"></header>
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

      <div id="controls">
        <h2>Interactive Controls:</h2>
        
        <div onChange={(e) => setParticipantSpeaker(e.target.value)}>
          The speaker is:-------- 
          <label> <input type="radio" value="Libby" name="speaker" /> Libby</label>
          <label> <input type="radio" value="Chris" name="speaker" /> Chris</label>
          <label> <input type="radio" value="Lynn" name="speaker" /> Lynn</label>
          <label> <input type="radio" value="Human" name="speaker" /> Human</label>
          <label> <input type="radio" value="" name="speaker" /> None</label>
        </div>
        <br></br>
        <button onClick={() => do_tts(participantSpeaker+". "+botResponse)}>Say Bot Response: {botResponse}</button>
        <br></br><br></br>
        <h4>Utility Responses</h4>
        <button onClick={() => do_tts("Yes.")}>Say Yes</button>---
        <button onClick={() => do_tts("No.")}>Say No</button>---
        <button onClick={() => do_tts("Thank you "+participantSpeaker)}>Say Thank You</button>---
        <button onClick={() => do_tts(participantSpeaker+" can you repeat that? I didn't hear you.")}>Please repeat</button>---
        <button onClick={() => do_tts("I am unsure how to answer that, sorry.")}>Say Unsure</button>
        <br></br>
        <br></br>
      </div>

      <button id="toggle" onClick={() => setFormToggle(!showForm)}>Show/Hide Form Input:</button>
      <div id="formcontent" hidden={showForm}>
          <br></br>
          <label>Behavior:
          <select value={behavior} 
              multiple={false}
              onChange={(e) => update_behavior(e.target.value)}>
              <option value="focused">focused</option>
              <option value="random">random</option>
              <option value="bored">bored</option>
              <option value="none">none</option>
            </select>
          </label>
          <label>Expression:
          <select value={expression} 
              multiple={false}
              onChange={(e) => update_expression(e.target.value)}>
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
              <option value="p308">British Female s2</option>
              <option value="p306">British Female m</option>
              <option value="p295">America Female s</option>
              <option value="p270">America Female s2**</option>
              <option value="p317">America Male s</option>
              <option value="p230">America Male f**</option>
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
              onChange={(e) => update_viseme(e.target.value)}
            />
          </label>
          </div>
          <br></br>
          <br></br>
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
          <button onClick={() => do_tts(textToSay)}>Say Text From Form: {textToSay}</button>
          <br></br>

      </div>

      <div id="transcription">
        <h3>Transcribed Data:</h3>
        <p>{transcribedData}</p>
      </div>
    </div>
  );
}

export default App;