import React, { useState, useEffect } from "react";
import Microphone from '@components/Microphone';
import { getTranscribedSpeech } from "@helpers/apiEventSources";


const Transcriber = ({ classes }) => {
  // The transcriber uses the Microphone component, which can
  // be either a set of buttons or an automatic voice detector.
  // The transcriber takes no arguments, updating the transcript
  // through the use of a transcribed_speech Event Source
  // TODO set text to component for UX management
  const [transcribedData, setTranscribedData] = useState([]);
  const [latestSpeech, setLatestSpeech] = useState("");

  function updateTranscription () {
    setTranscribedData(oldData => [oldData+latestSpeech]);
  }

  useEffect(() => {getTranscribedSpeech(setLatestSpeech)}, []);
  useEffect(updateTranscription,[latestSpeech])


  return (
    <div className="Transcriber">
      <header className="Transcriber-header"></header>
      <Microphone/>
      <label>Editable transcript of what has been said:<br></br>
            <textarea 
            cols={100}
            rows={40}
            value={transcribedData}
            onChange={(e) => setTranscribedData(e.target.value)}
            />
        </label>
    </div>
  );
}

export default Transcriber;