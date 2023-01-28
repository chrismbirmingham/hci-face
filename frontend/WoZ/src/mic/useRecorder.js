import { useState, useEffect } from "react";
import { startRecording, saveRecording } from "./recorder-controls";

const initialState = {
  recordingMinutes: 0,
  recordingSeconds: 0,
  initRecording: false,
  mediaStream: null,
  mediaRecorder: null,
  audio: null,
};

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
        // console.log("Uploading recording")
        const formData = new FormData();
        formData.append('uploadedFile', audioFile);
        if (audioBlob.size >0){
          fetch('//localhost:8000/api/audio', {
            headers: { Accept: "application/json" },
            method: "POST", body: formData
          });
        }

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
