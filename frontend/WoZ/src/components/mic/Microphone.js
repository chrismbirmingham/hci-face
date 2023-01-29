import React, {useState, useCallback, useEffect} from 'react';
import AudioAnalyser from './AudioAnalyser';
import useRecorder from "./useRecorder";

export default function Mic ({isRecording}) {
  const [listening, setListen] = useState(false);
  const [audio, setAudio] = useState(null) // eslint-disable-next-line
  const [rmsThresh, setRMSThresh] = useState(.005)
  const [bufferSize, setBufferSize] = useState(20)
  const [hearing, setHearing] = useState(false)
  const { recorderState, ...handlers } = useRecorder(); // eslint-disable-next-line
  const { startRecording, cancelRecording, saveRecording } = handlers;


  async function getMicrophone() {
    setListen(true)
    const audioDevice = await navigator.mediaDevices.getUserMedia({
      audio: true,
      video: false
    });
    setAudio(audioDevice);
  }

  function stopMicrophone() {
    setListen(false)
    audio.getTracks().forEach(track => track.stop());
    setAudio(null );
  }

  function toggleMicrophone() {
    if (audio) {
      stopMicrophone();
    } else {
      getMicrophone();
    }
  }

  useEffect(() => {
    console.log("responding to change in isRecording", isRecording, "listening is ", listening)
    if (listening && !isRecording) {
      console.log("toggle off")
      stopMicrophone()
    }
    if (!listening && isRecording) {
      console.log("toggle on")
      getMicrophone()
    }
  },[isRecording])

  useEffect(() => {
    if (hearing) {
      startRecording()
    }
    if (!hearing) {
      saveRecording()
    }
  },[hearing])


  const wrappingSetHearing = useCallback(val => {
    setHearing(val);
  }, [setHearing])

  function updateRMS(e) {
    stopMicrophone()
    setRMSThresh(e)
    getMicrophone()
  }
  function updateBuffer(e) {
    stopMicrophone()
    setBufferSize(e)
    getMicrophone()
  }
  return (
    <div className="App">
        <div className="controls">
        <button onClick={toggleMicrophone}>
            {audio ? 'Stop Listening' : 'Start Listening'}
          </button>
          {audio ? 
          <div id='Visualize Audio Capture'>
            <AudioAnalyser 
              audio={audio} 
              setHearing={wrappingSetHearing}
              rmsThresh={rmsThresh}
              bufferSize={bufferSize}
            /> 
            <label>RMS Threshold:
              <input 
                type="range" 
                value={rmsThresh}
                min="0.000"
                max="0.1"
                step="0.005"
                onChange={(e) => updateRMS(e.target.value)}
              />{rmsThresh}
            </label>
            <br></br>
            <label>Buffer Tail:
              <input 
                type="range" 
                value={bufferSize}
                min="5"
                max="50"
                step="5"
                onChange={(e) => updateBuffer(e.target.value)}
              />{bufferSize}
            </label>
          </div>: ''}
          <br></br>
        </div>
        <button onClick={() => setHearing(true)}>Manual Start Recording</button>
        <button onClick={() => setHearing(false)}>Manual Save Recording</button>
      </div>
    );
  
}

