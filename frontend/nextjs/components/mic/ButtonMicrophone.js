import React, {useState, useEffect} from 'react';
import useRecorder from "./useRecorder";


export default function ButtonMic() {
  const [recording, setRecording] = useState(0);
  const { recorderState, ...handlers } = useRecorder();
  const { startRecording, cancelRecording, saveRecording } = handlers;


  useEffect(() => {
    console.log("recording: ", recording)
    if (recording === 1) {
      startRecording();
    }
    if (recording === 0) {
      saveRecording();
    }
  }, [recording]);

  return (
    <div className="App">
      <div className="controls space-x-1.5">
        Manual Mic Controls:
        <button
          onMouseDown={() => setRecording(recording + 1)}
          onMouseUp={() => setRecording(0)}
        >
          Push to talk
        </button>

        <button onClick={() => setRecording(recording + 1)}>
          Start Recording
        </button>
        <button onClick={() => setRecording(0)}>End Recording</button>
      </div>
    </div>
  );
}


