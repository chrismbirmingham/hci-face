import React, { useRef, useEffect, useState } from 'react';
import {server_ip} from '@constants/serverip'

function rms(data) {
  // Calculate the root mean squared of a data set
  let squares = data.map((val) => (val*val)); // Square each value in the data set
  let sum = squares.reduce((acum, val) => (acum + val)); // Sum the squared values
  let mean = sum/data.length; // Find the mean of the data set
  let rms = Math.sqrt(mean); // Calculate the root mean squared
  let rmsint = Math.round(rms*100); // Convert to an integer
  rmsint = Math.min(rmsint, 255); // Restrict to 0-255
  if (isNaN(rmsint)) {
    console.log("ERROR: rms is not a number");
    return 0;
  } else {
    return rmsint; // Return the rms value
  }
}





function VoiceRecorder() {
  const animationIdRef = useRef(null);
  const streamRef = useRef(null);
  const recorderRef = useRef(null);
  const [threshold, setThreshold] = useState(100)
  const [rmswindow, setRmswindow] = useState(15)


  useEffect(() => {

    navigator.mediaDevices.getUserMedia({ audio: true })
      .then((stream) => {
        streamRef.current = stream;
        const audioContext = new AudioContext();
        const source = audioContext.createMediaStreamSource(stream);
        const analyser = audioContext.createAnalyser();
        analyser.fftSize = 2048*16;
        const bufferLength = analyser.fftSize;
        const longDataArray = new Uint8Array(bufferLength);

        source.connect(analyser);

        recorderRef.current = new MediaRecorder(stream);
        var rec = false;
        let chunks = [];

        if (recorderRef.current && recorderRef.current.state === "inactive") {
          console.log("recorderRef.current was innactive, starting")
          recorderRef.current.start();
        }
        recorderRef.current.ondataavailable = (e) => {
          console.log("recorderRef.current data available, pushing data on chunks")
          chunks.push(e.data);
        };
  
        recorderRef.current.onstop = () => {
          console.log("Stopping Recording", chunks)
          const audioBlob = new Blob(chunks, { type: "audio/ogg; codecs=opus" });
          const audioFile = new File([audioBlob], "audiofile.wav", {
            type: "audio/mpeg",
          })
          console.log("Uploading recording")
          const formData = new FormData();
          formData.append('uploaded_file', audioFile);
          if (audioBlob.size >1000){
            console.log("Succeeded! uploading, blob size"+audioBlob.size)
            fetch(server_ip+'/api/audio', {
              headers: { Accept: "application/json",
              },
              method: "POST", body: formData
              });
          }
          chunks = [];
        };
        
        const draw = () => {
          animationIdRef.current = requestAnimationFrame(draw);

          analyser.getByteTimeDomainData(longDataArray);

          const rmsarray = []
          for (let i=(16-rmswindow); i<16; i++) {
            let rmsint = rms(longDataArray.slice( 2048*i, 2048*(i+1)))
            rmsarray.push(rmsint)
          }
          let maxrms = Math.max(...rmsarray)
          if ((maxrms > threshold) && !rec) {
            rec = true
          }

          if ((maxrms < threshold) && rec) {
            console.log("stop Recording", maxrms)
            if (rec) {
              rec = false
              console.log("stopRecording")
              recorderRef.current.stop();
              console.log("startListening", maxrms)
              if (recorderRef.current && recorderRef.current.state === "inactive") {
                try {
                  recorderRef.current.start();
                } catch (error) {
                  console.log(error)
                }
              }
          }
        }
        if (maxrms < threshold) {
          console.log("discard listening")
          chunks = [];
        }
        };

        draw();
      })
      .catch((error) => {
        console.error(error);
      });

    return () => {
      console.log("I should stop listening!")
      if (recorderRef.current) {
      recorderRef.current.stop()
      }
      if (streamRef.current) {streamRef.current.getAudioTracks().forEach((track) => track.stop())};
      cancelAnimationFrame(animationIdRef.current);
    };
  }, [threshold, rmswindow]);

  return (
    <div>
      <p style={{"color":"green"}}>Actively Detecting/Recording/Transcribing Speech</p>
    </div>
  )
}



const VoiceMic = ({audioPlaying}) => {
  const [localAudio, setLocalAudio] = useState(true)

  return (
    <div>
      Mute-<input type="checkbox" checked={!localAudio} onChange={() => setLocalAudio(!localAudio)}/>
      {localAudio ?
        <div>
          {audioPlaying ? <p>Audio is playing</p> : <VoiceRecorder/>}
        </div>
        :
        <div>
          <p>Audio is muted</p>
        </div>}
    </div>
  )
}

export default VoiceMic;