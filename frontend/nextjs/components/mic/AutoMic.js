import React, {useState, useEffect} from 'react';
// import AudioVisualizer from './AudioVisualizer';
import useRecorder from "./useRecorder";

export default function AutoMic () {
  const [listening, setListen] = useState(true);
  const [audio, setAudio] = useState(null)
  const [rmsThresh, setRMSThresh] = useState(.003)
  const [bufferSize, setBufferSize] = useState(10)
  const [recording, setRecording] = useState(0)
  const { recorderState, ...handlers } = useRecorder(); 
  // eslint-disable-next-line
  const { startRecording, cancelRecording, saveRecording } = handlers;
  const button_color = {1:"green",0:"pink"}


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
    // return stopMicrophone
  }, [])



  useEffect(() => {
    console.log("There was a change in recording:", recording)
    if (recording===1 && listening) {
      startRecording()
    }
    if (recording===0) {
      saveRecording()
    }// eslint-disable-next-line
  },[recording])


  function calculateRMS(analyser, source, bufferLength, dataArray, rmshistory) {
      source.connect(analyser)
      analyser.getByteTimeDomainData(dataArray)
      const dataCopy = []
      for(let i = 0; i < bufferLength; i++) {
        dataCopy.push((dataArray[i]-128)/256.0)
      }
      let Squares = dataCopy.map((val) => (val*val));
      let Sum = Squares.reduce((acum, val) => (acum + val));
      let Mean = Sum/dataArray.length;
      let rms = Math.sqrt(Mean)
      // console.log("rms: "+rms)
      rmshistory.push(rms)
      if (rmshistory.length>bufferSize) {rmshistory.shift()}

      let rmsmax = Math.max(...rmshistory)

      if (rmsmax>rmsThresh) {
        console.log("recording!"+rmsmax+recording)
        setRecording(recording+1)
      }
      else {setRecording(0)}
    }

  useEffect(() => {
    console.log("Effect triggered" + audio + listening)
    let listenInterval = null;
    if (audio && listening) {
      console.log("Actively listening")
      const audioContext = new window.AudioContext()
      const analyser = new AnalyserNode(audioContext);
      const source = audioContext.createMediaStreamSource(audio);
      const bufferLength = analyser.fftSize;
      console.log(bufferLength)
      const dataArray = new Uint8Array(bufferLength)
      const rmshistory = []

      listenInterval = setInterval( 
        calculateRMS, 50, analyser, source, bufferLength, dataArray, rmshistory)
      }
    if (!listening) {
      console.log("Stopping interval")
      clearInterval(listenInterval)
    }
    return () => {console.log("returning from hook");clearInterval(listenInterval)}
  }, [audio, listening]);

  return (
    <div className="App">
        <div className="controls">
          <button style={{backgroundColor:button_color[recording]}} onClick={() => {setListen(!listening)}}>
            {listening ? 'Stop Listening' : 'Activate Listening'}
          </button> 
          <label>Volume Thresh:
            <input type="text" value={rmsThresh} onChange={(e) => setRMSThresh(e.target.value)}/>
        </label>
          <br></br>
        {/* {audio ? <p>audio</p>:<p>no audio</p>}
        <p>Listening: {listening? 1: 0}</p>
        <p>Recording: {recording}</p> */}
        
        </div>
      </div>
    );
  
}


