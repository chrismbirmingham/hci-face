import React from "react"

export default function FacilitatorControls ({update_speaker, do_tts, get_preset, participantSpeaker, latestSpeech, classifications, condition, botResponse, facilitatorResponse}) {

  return(
    <div id="controls">
    
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
        <button onClick={() => get_preset("d_response")}>Request Response</button>
      </div>
      <div hidden={!condition}><p>Condition: Role Model</p>
        Recommended Statement:<button style={{backgroundColor:"Chartreuse"}} onClick={() => do_tts(participantSpeaker+". "+facilitatorResponse)}>{participantSpeaker+". "+facilitatorResponse}</button>
        <br></br>
        <button onClick={() => get_preset("r_disclosure")}>Make Disclosure</button>--
        <button onClick={() => get_preset("r_response")}>Say Response</button>
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
        <button onClick={() => get_preset("g_QT/emotions/happy")}>happy</button>--
        <button onClick={() => get_preset("g_QT/emotions/shy")}>shy</button>--
        <button onClick={() => get_preset("g_QT/emotions/sad")}>sad</button>--
        <button onClick={() => get_preset("g_QT/hi")}>wave</button>--
      </div>
      <br></br>
      Say ChatBot Response: <button style={{backgroundColor:"pink"}} onClick={() => do_tts(botResponse)}>{botResponse}</button>
      <br></br>
      <br></br>
    </div>
  )
}