import React, { useState, useEffect } from "react";
import Mic from '../components/mic/Microphone';
import Link from 'next/link';

const WoZ = ({ classes }) => {
  const [transcribedData, setTranscribedData] = useState([""]);
  const [isRecording, setIsRecording] = useState(true);

  // API calling functions
  function getTextStream() {
    const es = new EventSource("http://localhost:8000/api/text_stream");
    es.addEventListener('open', () => {
      // console.log('SSE opened@!')
    });

    es.addEventListener('human_speech', (e) => {
      let speech = e.data;
      if (speech.length>0){
        console.log("STT process: ", speech)
        setTranscribedData(oldData => [oldData +"\n"+ speech]);
      };
    });
  
    es.addEventListener('error', (e) => {
      console.error('Error: ',  e);
    });

    return () => {
      es.close();
    };
  }
  useEffect(getTextStream, []);

  return (
    <div className="WoZ">
      <header className="WoZ-header"></header>
      <Mic isRecording={isRecording}/>
      <label>Editable transcript of what has been said:<br></br>
            <textarea 
            cols={100}
            rows={40}
            value={transcribedData}
            onChange={(e) => setTranscribedData(e.target.value)}
            />
        </label>
      <br></br><Link href="/">Back to home</Link>
    </div>
  );
}

export default WoZ;