import React from "react"

export default function SpeakerMonitor ({update_speaker, participantSpeaker, latestSpeech, classifications}) {

  return(
    <div id="speech">
      <div onChange={(e) => update_speaker(e.target.value)}>The current speaker:
      <label> <input type="radio" value="Nathan." name="speaker" /> Nathan</label>
      <label> <input type="radio" value="Lauren." name="speaker" /> Lauren</label>
      <label> <input type="radio" value="Mina." name="speaker" /> Mina</label>
      <label> <input type="radio" value="Participant" name="speaker" /> Default</label>
      <label> <input type="radio" value="" name="speaker" /> None</label>
    </div>
    <br></br>
    {participantSpeaker} Said: {latestSpeech}
    <br></br>
    Classified as: {classifications}
    </div>
  )
}