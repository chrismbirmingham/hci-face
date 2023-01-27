import './App.css';
import React, { useState, useRef, useEffect, useCallback } from "react";
import Mic from './alternate-mic/Microphone';

const App = ({ classes }) => {
  // Variables with state
  const [beginConversation, setBeginConversation] = useState(true);
  const [textToSay, setTextToSay] = useState("This is an example of what I sound like when I am talking.");
  const [botResponse, setBotResponse] = useState("This is the bot response");
  const [facilitatorResponse, setFacilitatorResponse] = useState("This is the Facilitator response");

  const [isRecording, setIsRecording] = useState(false);
  const [showForm, setFormToggle] = useState(false);
  const [showWalkthrough, setWalkthroughToggle] = useState(false);
  const [condition, setCondition] = useState(false);

  const [speakerVoice, setSpeakerVoice] = useState("p270");
  const [participantSpeaker, setParticipantSpeaker] = useState("");
  const [priorSpeaker, setPriorSpeaker] = useState("");
  const [latestSpeech, setLatestSpeech] = useState("");
  const [classifications, setClassifications] = useState("");
  const [transcribedData, setTranscribedData] = useState([""]);
  
  const [viseme, setViseme] = useState("");
  const [expression, setExpression] = useState("");
  const [behavior, setBehavior] = useState("focused");
  
	// The state for our timer
  const Ref = useRef(null);
	const [timer, setTimer] = useState('00:00:00');
	const [minuteGoal, setMinuteGoal] = useState('15');
	const [color, setColor] = useState('Green');
	const [date, setDate] = useState("")


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

  useEffect(() => {const es = new EventSource("http://192.168.1.136:8000/api/text_stream");
    es.addEventListener('open', () => {
      // console.log('SSE opened@!')
    });

    // faceControls should handle vizemes, eyeAU, browAU, mouthAU
    es.addEventListener('human_speech', (e) => {
      let speech = e.data;
      if (speech.length>0){
        setLatestSpeech(speech);
        let full_speech = speech
        console.log("STT process: ", speech, " from ", participantSpeaker)

        if (participantSpeaker === priorSpeaker){
          full_speech = latestSpeech + " " + speech;
        }

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

	const getTimeRemaining = (e) => {
		const total = Date.parse(e) - Date.parse(new Date());
		const seconds = Math.floor((total / 1000) % 60);
		const minutes = Math.floor((total / 1000 / 60) % 60);
		const hours = Math.floor((total / 1000 / 60 / 60) % 24);
		setDate(Date())
		return {
			total, hours, minutes, seconds
		};
	}

	const updateTimerDisplay = (e) => {
		let { total, hours, minutes, seconds } = getTimeRemaining(e);
		if (total >= 0) {
			setTimer(
				(hours > 9 ? hours : '0' + hours) + ':' +
				(minutes > 9 ? minutes : '0' + minutes) + ':'
				+ (seconds > 9 ? seconds : '0' + seconds)
			)
		}
		if (minutes >= 3){setColor("green")}
		if (minutes < 3 && minutes >= 1){
      setWalkthroughToggle(false)
      setColor("orange")
    }
		if (minutes < 1 ){setColor("red")}
	}

	const runTimer = (e) => {
		console.log("run timer")
		if (Ref.current) clearInterval(Ref.current);
		const id = setInterval(() => {
			
			// console.log("Update Timer Display", playTimerVar)
			updateTimerDisplay(e);

		}, 1000)
		Ref.current = id;
	}

	const getDeadTime = (m) => {
		console.log("New goal: ", m)
		let deadline = new Date();
		let newseconds = m*60 + 1
		deadline.setSeconds(deadline.getSeconds() + newseconds);
		return deadline;
	}

  /////// Get and Play Speech ///////
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

  function get_preset(name) {
    const text = name
    if (text==="f_invitation"){
      runTimer(getDeadTime(15))
      setWalkthroughToggle(true)
    }
    if (text) {
      return fetch(`//localhost:8000/api/facilitator_presets?text=${encodeURIComponent(text)}`, { cache: 'no-cache' })
      .then(response => response.text())
      .then(message => {console.log(message); do_tts(message)})
    }
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

  return (
    <div className="App">
      <header className="App-header"></header>
      <Mic/>
      
      <br></br>
      <button id="toggle" onClick={() => setWalkthroughToggle(!showWalkthrough)}>Show/Hide Study Walkthrough: </button>
      <div id="walkthrough" hidden={showWalkthrough}>
      {/* <h2>Study Walkthrough:</h2> */}
      
        1- Start by reviewing consent <br></br> 
        ---<input type="checkbox"/>Being in the study is voluntary <br></br>
        ---<input type="checkbox"/>You will interact with a robot and with the others on this call. <br></br>
        ---<input type="checkbox"/>The primary risk in this study is that you may be uncomfortable answering a question posed in this study. You may decline to answer any question. <br></br>
        ---<input type="checkbox"/>You will not recieve any direct benefit from this study <br></br>
        ---<input type="checkbox"/>This study will be recorded. <br></br>
        ---<input type="checkbox"/>Do you consent to be a part of this study? You may withdraw your consent at any time. <br></br>
        <input type="checkbox"/>Please complete the first four pages of the survey linked in the chat, and return to the zoom session when directed to stop:<br></br>    
        <input type="checkbox"/>https://usc.qualtrics.com/jfe/form/SV_diE2Ow6GQPlSCP4 <br></br>
        <br></br>
          <button onClick={() => do_tts("Testing, 1, 2, 3. Can you all hear me?")}>Speech Test</button>
        <br></br>
        2- <button onClick={() => get_preset("f_qt-intro")}>QT introduction</button>--
          <button onClick={() => get_preset("g_QT/hi")}>wave</button>--
        <button style={{backgroundColor:"grey"}} onClick={() => get_preset("f_survey-prompt")}>survey prompt</button>--
        <button style={{backgroundColor:"grey"}} onClick={() => get_preset("f_survey-return")}>survey return</button>
        <br></br>
        <br></br>
        3- <button onClick={() => get_preset("f_group-intro")}>group introductions</button>--
          <button onClick={() => get_preset("g_QT/emotions/shy")}>shy</button>--(let them respond)--
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
        ---<input type="checkbox"/>What did you like and dislike about interacting with QT? <br></br>
        ---<input type="checkbox"/>What did you learn as part of the group today?<br></br>
        ---<input type="checkbox"/>Do you think that QT understood you? <br></br>
        ---<input type="checkbox"/>What differences did you notice between round 1 and round 2 of the support group?<br></br>
        ---<input type="checkbox"/>How did the support group today affect your stress level? <br></br>
      </div>

      <div id="controls">
        <h3 style={{backgroundColor:color}}>Interactive Controls:</h3>
			  <p style={{backgroundColor:color}}>Time Remaining: {timer}</p>
        
        <div onChange={(e) => update_speaker(e.target.value)}>The current speaker:
          <label> <input type="radio" value="Nathan" name="speaker" /> Nathan</label>
          <label> <input type="radio" value="Lauren" name="speaker" /> Lauren</label>
          <label> <input type="radio" value="Mina" name="speaker" /> Mina</label>
          <label> <input type="radio" value="Participant" name="speaker" /> Default</label>
          <label> <input type="radio" value="" name="speaker" /> None</label>
        </div>
        <br></br>
        {participantSpeaker} Said: {latestSpeech}
        <br></br>
        Classified as: {classifications}
        
        <div hidden={condition}><p>Condition: Director</p>
        Recommended Statement:<button style={{backgroundColor:"Chartreuse"}} onClick={() => do_tts(participantSpeaker+". "+facilitatorResponse)}>{participantSpeaker+". "+facilitatorResponse}</button>
        <br></br>
          <button onClick={() => get_preset("d_disclosure")}>Request Disclosure</button>--
          <button onClick={() => get_preset("d_response")}>Request Response</button></div>
        <div hidden={!condition}><p>Condition: Role Model</p>
        Recommended Statement:<button style={{backgroundColor:"Chartreuse"}} onClick={() => do_tts(participantSpeaker+". "+facilitatorResponse)}>{participantSpeaker+". "+facilitatorResponse}</button>
        <br></br>
          <button onClick={() => get_preset("r_disclosure")}>Make Disclosure</button>--
          <button onClick={() => get_preset("r_response")}>Say Response</button></div>
        <br></br>

        {/* <h4>Utility Responses</h4> */}
        Utility Responses: 
          <button style={{backgroundColor:"green"}} onClick={() => do_tts("Yes.")}>Yes</button>---
          <button style={{backgroundColor:"red"}} onClick={() => do_tts("No.")}>No</button>---
          <button style={{backgroundColor:"BlueViolet"}} onClick={() => do_tts("Thank you, "+participantSpeaker)}>Thank You</button>---
          <button style={{backgroundColor:"orange"}} onClick={() => do_tts(participantSpeaker+" can you repeat that? I didn't hear you.")}>Please repeat</button>---
          <button onClick={() => do_tts("I am unsure how to answer that. sorry.")}>Unsure</button>----
          <button onClick={() => do_tts("Is there anything anyone would like to talk about? Are there any challenges or struggles you are working through?")}>Prompt</button>----
        <br></br><br></br>
          Gestures:
          <button onClick={() => get_preset("g_QT/emotions/happy")}>happy</button>--
          <button onClick={() => get_preset("g_QT/emotions/shy")}>shy</button>--
          <button onClick={() => get_preset("g_QT/emotions/sad")}>sad</button>--
          <button onClick={() => get_preset("g_QT/hi")}>wave</button>--
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
              <option value="none">none</option>
              <option value="focused">focused</option>
              <option value="random">random</option>
              <option value="bored">bored</option>
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
          Set timer for: <input value={minuteGoal} type="text" size={3} onChange={(e) => setMinuteGoal(e.target.value)}/>
			  <button onClick={() => runTimer(getDeadTime(minuteGoal))}>Run Timer</button> --- Current Date: {date}
        <br></br><label>Enter the text you would like the robot to say:
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