import React from "react"

export default function FacilitatorControls ({do_tts, participantSpeaker, condition, botResponse, facilitatorResponse}) {
  /////// Fetch Speech ///////
  function get_preset(mode, query) {
    return fetch(`//localhost:8000/api/facilitator_presets?mode=${encodeURIComponent(mode)}&query=${encodeURIComponent(query)}`, { cache: 'no-cache' })
    .then(response => response.text())
    .then(message => {console.log(message); do_tts(message)})
  }
  function set_gesture(name){
    return fetch(`//localhost:8000/api/qt_gesture?text=${encodeURIComponent(name)}`, { cache: 'no-cache' })
      .then(response => response.text())
      .then(message => {console.log(message)})
  }
  return(
    <div id="controls">
  
      <div hidden={condition}><p>Condition: Director</p>
        Recommended Statement:<button style={{backgroundColor:"Chartreuse"}} onClick={() => do_tts(participantSpeaker+". "+facilitatorResponse)}>{participantSpeaker+". "+facilitatorResponse}</button>
        <br></br> Ignore Recommendations: 
        <button onClick={() => get_preset("director","disclosure")}>Request Disclosure</button>--
        <button onClick={() => get_preset("director","response")}>Request Response</button>
      </div>
      <div hidden={!condition}><p>Condition: Role Model</p>
        Recommended Statement:<button style={{backgroundColor:"Chartreuse"}} onClick={() => do_tts(participantSpeaker+". "+facilitatorResponse)}>{participantSpeaker+". "+facilitatorResponse}</button>
        <br></br> Ignore Recommendations: 
        <button onClick={() => get_preset("role_model","disclosure")}>Make Disclosure</button>--
        <button onClick={() => get_preset("role_model","response")}>Say Response</button>
      </div>
      <br></br>

      {/* <h4>Utility Responses</h4> */}
      <div style={{backgroundColor:"grey"}}>
        Utility Responses: 
        <button style={{backgroundColor:"green"}} onClick={() => do_tts("Yes.")}>Yes</button>---
        <button style={{backgroundColor:"red"}} onClick={() => do_tts("No.")}>No</button>---
        <button style={{backgroundColor:"BlueViolet"}} onClick={() => do_tts("Thank you, "+participantSpeaker)}>Thank You</button>---
        <button style={{backgroundColor:"orange"}} onClick={() => do_tts(participantSpeaker+" can you repeat that? I didn't hear you.")}>Please repeat</button>---
        <button onClick={() => do_tts("I am unsure how to answer that. sorry.")}>Unsure</button>----
        <button onClick={() => do_tts("Is there anything anyone would like to talk about? Are there any challenges or struggles you are working through?")}>Prompt</button>----
        <br></br><br></br>
        Gestures:
        <button onClick={() => set_gesture("QT/emotions/happy")}>happy</button>--
        <button onClick={() => set_gesture("QT/emotions/shy")}>shy</button>--
        <button onClick={() => set_gesture("QT/emotions/sad")}>sad</button>--
        <button onClick={() => set_gesture("QT/hi")}>wave</button>--
      </div>
      <br></br>
      Say ChatBot Response: <button style={{backgroundColor:"pink"}} onClick={() => do_tts(botResponse)}>{botResponse}</button>
      <br></br>
      <br></br>
    </div>
  )
}