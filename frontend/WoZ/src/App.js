import './App.css';
import React, { useState } from "react";
import { w3cwebsocket as W3CWebSocket } from "websocket";
import { ReactMic } from './react-mic-clone';
import CountdownTimer from './timer';

const stt_socket = new W3CWebSocket('ws://localhost:8000/api/stt');

const App = ({ classes }) => {
  // Variables with state
  const [beginConversation, setBeginConversation] = useState(true);
  const [textToSay, setTextToSay] = useState("This is an example of what I sound like when I am talking.");
  const [botResponse, setBotResponse] = useState("This is the bot response");
  const [treeResponse, setTreeResponse] = useState("This is the tree response");

  const [isRecording, setIsRecording] = useState(false);
  const [showForm, setFormToggle] = useState(false);
  const [showWalkthrough, setWalkthroughToggle] = useState(false);
  const [condition, setCondition] = useState(false);

  const [speakerVoice, setSpeakerVoice] = useState("p270");
  const [participantSpeaker, setParticipantSpeaker] = useState("");
  const [latestSpeech, setLatestSpeech] = useState("");
  const [classifications, setClassifications] = useState("");
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
    setTranscribedData(oldData => [participantSpeaker+":"+received,  <br></br>, ...oldData ]);
    setLatestSpeech(received);
    setClassifications("");
    const raw_res = await get_bot_response(received);
    const respArray = raw_res.split("&&&");
    setTreeResponse(respArray[0]);
    setBotResponse(respArray[1]);
    setClassifications(respArray[2]);
    setBeginConversation(false);
  }
  async function get_bot_response(human_input) {
    const text = human_input
    const speaker = participantSpeaker
    if (text) {
      return fetch(`//localhost:8000/api/bot_response?text=${encodeURIComponent(text)}&speaker=${encodeURIComponent(speaker)}&reset_conversation=${encodeURIComponent(beginConversation)}&director_condition=${encodeURIComponent(!condition)}`, { cache: 'no-cache' })
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

  function get_preset(name) {
    const text = name
    if (text) {
      return fetch(`//localhost:8000/api/facilitator_buttons?text=${encodeURIComponent(text)}`, { cache: 'no-cache' })
      .then(response => response.text())
      .then(message => {console.log(message); do_tts(message)})
    }
  }

  function set_face(name, type){
    const name_text = name
    const type_text = type
    return fetch(`//localhost:8000/api/facilitator_face?text=${encodeURIComponent(name_text)}&update_type=${encodeURIComponent(type_text)}`, { cache: 'no-cache' })
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
            height={60}
            width={320} />
        </div>      
      </div>
      <CountdownTimer/>
      <br></br>
      <button id="toggle" onClick={() => setWalkthroughToggle(!showWalkthrough)}>Show/Hide Walkthrough</button>
      <div id="walkthrough" hidden={showWalkthrough}>
      <h2>Study Walkthrough:</h2>
      
        1- Start by reviewing consent <br></br>
        ---  Next provide link to survey:    https://usc.qualtrics.com/jfe/form/SV_diE2Ow6GQPlSCP4 <br></br>
        ---  WoZ prompt participants to complete first question set. (4 pages)
        <br></br>
        <br></br>
        2- <button onClick={() => get_preset("f_qt-intro")}>QT introduction</button>--
        <button style={{backgroundColor:"grey"}} onClick={() => get_preset("f_survey-prompt")}>survey prompt</button>--
        <button style={{backgroundColor:"grey"}} onClick={() => get_preset("f_survey-return")}>survey return</button>
        <br></br>
        <br></br>
        3- <button onClick={() => get_preset("f_group-intro")}>group introductions</button>--(let them respond)--
        <button style={{backgroundColor:"green"}} onClick={() => get_preset("f_invitation")}>invitation to start</button>--
        <button style={{backgroundColor:"orange"}} onClick={() => get_preset("f_closing")}>closing</button>--
        <button style={{backgroundColor:"red"}} onClick={() => get_preset("f_transition")}>End section</button>--
        <button style={{backgroundColor:"grey"}} onClick={() => get_preset("f_survey-prompt")}>survey-prompt</button>--
        <button style={{backgroundColor:"grey"}} onClick={() => get_preset("f_survey-return")}>survey-return</button>
        <br></br>
        <br></br>
        4- 
        <button id="condition" onClick={() => setCondition(!condition)}>Switch Conditions:</button>--
        <button style={{backgroundColor:"green"}} onClick={() => get_preset("f_invitation")}>invitation</button>--
        <button style={{backgroundColor:"orange"}} onClick={() => get_preset("f_closing")}>closing</button>--
        <button style={{backgroundColor:"red"}} onClick={() => get_preset("f_transition")}>End section</button>--
        <button style={{backgroundColor:"grey"}} onClick={() => get_preset("f_survey-prompt")}>survey-prompt</button>--
        <button style={{backgroundColor:"grey"}} onClick={() => get_preset("f_survey-return")}>survey-return</button>
        <br></br>
        <br></br>
        5- Ask participants to complete the final survey questions (3 pages)
        <br></br>
        6- Lead participants through group discussion<br></br>
        <br></br>What did you like and dislike about interacting with QT? 
        <br></br>What did you learn as part of the group today?
        <br></br>Do you think that QT understood you? 
        <br></br>What differences did you notice between round 1 and round 2 of the support group?
        <br></br>How did the support group today affect your stress level? 
      </div>

      <div id="controls">
        <h3 style={{backgroundColor:"grey"}}>Interactive Controls:</h3>
        
        <div onChange={(e) => setParticipantSpeaker(e.target.value)}>The current speaker:
          <label> <input type="radio" value="Libby" name="speaker" text="Libby" /> Libby</label>
          <label> <input type="radio" value="Chris" name="speaker" /> Chris</label>
          <label> <input type="radio" value="Lynn" name="speaker" /> Lynn</label>
          <label> <input type="radio" value="Human" name="speaker" /> Default</label>
          <label> <input type="radio" value="" name="speaker" /> None</label>
        </div>
        Said: {latestSpeech}
        <br></br>
        Classified as: {classifications}
        
        <div hidden={condition}><p>Condition: Director</p>
        Recommended Statement:<button style={{backgroundColor:"Chartreuse"}} onClick={() => do_tts(participantSpeaker+". "+treeResponse)}>{participantSpeaker+". "+treeResponse}</button>
        <br></br>
          <button onClick={() => get_preset("d_disclosure")}>Request Disclosure</button>--
          <button onClick={() => get_preset("d_response")}>Request Response</button></div>
        <div hidden={!condition}><p>Condition: Role Model</p>
        Recommended Statement:<button style={{backgroundColor:"Chartreuse"}} onClick={() => do_tts(participantSpeaker+". "+treeResponse)}>{participantSpeaker+". "+treeResponse}</button>
        <br></br>
          <button onClick={() => get_preset("r_disclosure")}>Make Disclosure</button>--
          <button onClick={() => do_tts(participantSpeaker+". "+treeResponse)}>Say Response: {treeResponse}</button></div>
        <br></br>

        {/* <h4>Utility Responses</h4> */}
        Utility Responses: 
          <button style={{backgroundColor:"green"}} onClick={() => do_tts("Yes.")}>Yes</button>---
          <button style={{backgroundColor:"red"}} onClick={() => do_tts("No.")}>No</button>---
          <button style={{backgroundColor:"BlueViolet"}} onClick={() => do_tts("Thank you, "+participantSpeaker)}>Thank You</button>---
          <button style={{backgroundColor:"orange"}} onClick={() => do_tts(participantSpeaker+" can you repeat that? I didn't hear you.")}>Please repeat</button>---
          <button onClick={() => do_tts("I am unsure how to answer that. sorry.")}>Unsure</button>----
          <button onClick={() => do_tts("Is there anything anyone would like to talk about?")}>Prompt</button>----
          <button onClick={() => get_preset("g_QT/emotions/happy")}>happy</button>--
          <button onClick={() => get_preset("g_QT/emotions/shy")}>shy</button>--
        <br></br><br></br>
        Say ChatBot Response: <button style={{backgroundColor:"pink"}} onClick={() => do_tts(botResponse)}>{botResponse}</button>
        <br></br>
        <br></br>
      </div>

      <button id="toggle" onClick={() => setFormToggle(!showForm)}>Show/Hide Form Input:</button>
      <div id="formcontent" hidden={!showForm}>
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
          {/* # 267!,307 - English male, medium
          # 330!,232 - English male, slow
          # 312!,251 - English male, fast
          # 287,254 - English male, fast and deep
          # 303 - English female, slow
          # 306 - English female, medium
          # 308 - English female, slow
          # 295!,270 - American female, slow
          # 317! - American male, slow
          # 230! - American male, fast
          # 345 - south african female, slow
          # 313,233 - ? male, fast */}
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
              <option value="p270">America Female s2!!!</option>
              <option value="p317">America Male s</option>
              <option value="p230">America Male f!!!</option>
              <option value="p313">Male f</option>
            </select>
          </label>
          <br></br><br></br>
          <div hidden={true}>
          <label>OR Enter viseme to show:
            <input 
              type="text" 
              value={viseme}
              onChange={(e) => update_viseme(e.target.value)}
            />
          </label>
          </div>
          <label>Enter the text you would like the robot to say:
            <textarea 
              cols={100}
              rows={4}
              // type="text" 
              value={textToSay}
              onChange={(e) => setTextToSay(e.target.value)}
            />
          </label>
          <br></br><button onClick={() => do_tts(textToSay)}>Say Text From Form: {textToSay}</button><br></br>
      </div>

      <div id="transcription">
        <h2>Transcribed Data:</h2>
        <p>{transcribedData}</p>
      </div>
    </div>
  );
}

export default App;