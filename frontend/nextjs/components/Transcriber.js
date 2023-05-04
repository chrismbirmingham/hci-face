import React, { useState, useEffect } from "react";
import { getTranscribedSpeech } from "@helpers/apiEventSources";
import GetInput from "./conversation/GetInput";

const Transcriber = ({ classes }) => {
  // The transcriber uses the Microphone component, which can
  // be either a set of buttons or an automatic voice detector.
  // The transcriber takes no arguments, updating the transcript
  // through the use of a transcribed_speech Event Source
  // TODO set text to component for UX management
  const [transcribedData, setTranscribedData] = useState([""]);
  const [latestSpeech, setLatestSpeech] = useState("");
  const [numLines, setNumLines] = useState(0);

  function updateTranscription () {
    setTranscribedData(oldData => [oldData+latestSpeech]);
  }

  useEffect(() => {getTranscribedSpeech(setLatestSpeech)}, []);
  useEffect(updateTranscription,[latestSpeech])
  useEffect(() => {
    let line_counter = 0
    let lines = transcribedData[0].split("\n")
    for (let i=0; i<lines.length; i++) {
      line_counter += Math.floor(lines[i].length/100)
    }
    line_counter += lines.length
    setNumLines(line_counter)
    console.log("numLines: ", line_counter)
  },[transcribedData])

  


  return (
    <div className="Transcriber">
      <header className="Transcriber-header"></header>
      <GetInput setLatestSpeech={setLatestSpeech} auto={true}/>
      <label>Editable transcript of what has been said:<br></br>
      <button onClick={() => {navigator.clipboard.writeText(transcribedData)}}>Copy to clipboard</button><br></br>
          <textarea
          rows={numLines }
          cols={100}
          value={transcribedData}
          onChange={(e) => setTranscribedData([e.target.value])}
          />
        </label>
    </div>
  );
}

export default Transcriber;