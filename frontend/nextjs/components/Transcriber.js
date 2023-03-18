import React, { useState, useEffect } from "react";
import Mic from '@components/mic/Microphone';
import AutoMic from '@components/mic/AutoMic';
import { getTextStream } from "@helpers/apiEventSources";


const Transcriber = ({ classes }) => {
  const [transcribedData, setTranscribedData] = useState([]);
  const [audioPlaying, setAudioPlaying] = useState(false);
  const [latestSpeech, setLatestSpeech] = useState("");
  const [botResponse, setBotResponse] = useState("This is the bot response");

  function updateTranscription () {
    setTranscribedData(oldData => [oldData+latestSpeech]);
  }

  useEffect(() => {getTextStream(setLatestSpeech, setBotResponse)}, []);
  useEffect(updateTranscription,[latestSpeech])


  return (
    <div className="WoZ">
      <header className="WoZ-header"></header>
      <AutoMic audioPlaying={audioPlaying}/>
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