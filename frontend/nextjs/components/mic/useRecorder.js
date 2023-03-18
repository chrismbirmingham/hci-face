import { useState, useEffect } from "react";
// import { startRecording, saveRecording } from "./recorder-controls";

const initialState = {
  recordingMinutes: 0,
  recordingSeconds: 0,
  initRecording: false,
  mediaStream: null,
  mediaRecorder: null,
  audio: null,
};

async function startRecording(setRecorderState) {
  try {
    const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
    // console.log("setting the stream")
    setRecorderState((prevState) => {
      return {
        ...prevState,
        initRecording: true,
        mediaStream: stream,
      };
    });
    // console.log("stream set")
  } catch (err) {
    // console.log(err);
  }
}

export function saveRecording(recorder) {
  // console.log("saving the recording")
  // console.log(recorder)
  if (recorder && recorder.state !== "inactive") {
    recorder.stop();
    // console.log("Should be stopped")
  }
  // recorder.stop();
}

export default function useRecorder() {
  const [recorderState, setRecorderState] = useState(initialState);

  useEffect(() => { //Update recording state while recording
    const MAX_RECORDER_TIME = 50;
    let recordingInterval = null;

    if (recorderState.initRecording)
      recordingInterval = setInterval(() => {
        setRecorderState((prevState) => {
          if (
            prevState.recordingMinutes === MAX_RECORDER_TIME &&
            prevState.recordingSeconds === 0
          ) {
            clearInterval(recordingInterval);
            return prevState;
          }

          if (prevState.recordingSeconds >= 0 && prevState.recordingSeconds < 59)
            return {
              ...prevState,
              recordingSeconds: prevState.recordingSeconds + 1,
            };

          if (prevState.recordingSeconds === 59)
            return {
              ...prevState,
              recordingMinutes: prevState.recordingMinutes + 1,
              recordingSeconds: 0,
            };
        });
      }, 1000);
    else clearInterval(recordingInterval);

    return () => clearInterval(recordingInterval);
  });

  useEffect(() => { //Update recorder state if mediastream changes
    if (recorderState.mediaStream)
      setRecorderState((prevState) => {
        return {
          ...prevState,
          mediaRecorder: new MediaRecorder(prevState.mediaStream),
        };
      });
  }, [recorderState.mediaStream]);

  useEffect(() => { //if recorderState.mediaRecorder changes, set recorder to it
    // console.log("mediaRecorder state changed")
    const recorder = recorderState.mediaRecorder;
    // console.log("new state: ", recorder)
    let chunks = [];

    if (recorder && recorder.state === "inactive") {
      // console.log("recorder was innactive, starting")
      recorder.start();

      recorder.ondataavailable = (e) => {
        // console.log("recorder data available, pushing data on chunks")
        chunks.push(e.data);
      };

      recorder.onstop = () => {
        // console.log("Stopping Recording", chunks)
        const audioBlob = new Blob(chunks, { type: "audio/ogg; codecs=opus" });
        const audioFile = new File([audioBlob], "audiofile.wav", {
          type: "audio/mpeg",
        })
        console.log("Uploading recording")
        const formData = new FormData();
        formData.append('uploaded_file', audioFile);
        if (audioBlob.size >1000){
          fetch('//localhost:8000/api/audio', {
            headers: { Accept: "application/json",
          },
            method: "POST", body: formData
          });
        }
        else {console.log("not uploading, blob size"+audioBlob.size)}

        chunks = [];
        // console.log("set up initial state")
        setRecorderState((prevState) => {
          if (prevState.mediaRecorder)
            return {
              ...initialState,
              audio: window.URL.createObjectURL(audioBlob),
            };
          else return initialState;
        });
      };
    }

    return () => {
      if (recorder) recorder.stream.getAudioTracks().forEach((track) => track.stop());
    };
  }, [recorderState.mediaRecorder]);

  return {
    recorderState,
    startRecording: () => {
      // console.log("Hitting start recording"); 
      startRecording(setRecorderState)},
    cancelRecording: () => {
      // console.log("Hitting cancel recording");
      setRecorderState(initialState)},
    saveRecording: () => {
      // console.log("Hitting save recording");
      saveRecording(recorderState.mediaRecorder)},
  };
}
