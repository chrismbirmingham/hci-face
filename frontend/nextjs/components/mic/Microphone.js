import React, {useState, useEffect} from 'react';
// import AudioVisualizer from './AudioVisualizer';
import useRecorder from "./useRecorder";

export default function Mic () {
  const [audio, setAudio] = useState(null)
  const [recording, setRecording] = useState(0)
  const { recorderState, ...handlers } = useRecorder(); 
  // eslint-disable-next-line
  const { startRecording, cancelRecording, saveRecording } = handlers;


  // Set up and stop audio device
  async function getMicrophone() {
    console.log("Starting up mic")
    const audioDevice = await navigator.mediaDevices.getUserMedia({
      audio: true,
      video: false
    });
    setAudio(audioDevice);
  }
  function stopMicrophone() {
    console.log("Closing down mic")
    audio.getTracks().forEach(track => track.stop());
    setAudio(null);
  }

  useEffect( ()=> {
    if (!audio) {
      getMicrophone()
    }
  }, [audio])



  useEffect(() => {
    console.log("There was a change in recording:", recording)
    if (recording===1) {
      startRecording()
    }
    if (recording===0) {
      saveRecording()
    }// eslint-disable-next-line
  },[recording])


  return (
    <div className="App">
        <div className="controls">
          <button onMouseDown={() => setRecording(recording+1)} onMouseUp={() => setRecording(0)}>Push to talk</button>
        
          <button onClick={() => setRecording(recording+1)}>Start Recording</button>
          <button onClick={() => setRecording(0)}>End Recording</button>
        
        </div>
      </div>
    );
  
}


