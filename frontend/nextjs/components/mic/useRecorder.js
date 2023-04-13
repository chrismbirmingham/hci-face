import { useState, useEffect } from "react";
import {server_ip, localhostip} from '@constants/serverip'

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
    setRecorderState((prevState) => {
      return {
        ...prevState,
        initRecording: true,
        mediaStream: stream,
      };
    });
  } catch (err) {
  }
}

export function saveRecording(recorder) {
  if (recorder && recorder.state !== "inactive") {
    recorder.stop();
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
    const recorder = recorderState.mediaRecorder;
    let chunks = [];

    if (recorder && recorder.state === "inactive") {
      recorder.start();

      recorder.ondataavailable = (e) => {
        chunks.push(e.data);
      };

      recorder.onstop = () => {
        console.log("Stopping Recording", chunks)
        const audioBlob = new Blob(chunks, { type: "audio/ogg; codecs=opus" });
        const audioFile = new File([audioBlob], "audiofile.wav", {
          type: "audio/mpeg",
        })
        console.log("Uploading recording")
        const formData = new FormData();
        formData.append('uploaded_file', audioFile);
        if (audioBlob.size >1000){
          fetch(server_ip+'/api/audio', {
            headers: { Accept: "application/json",
            },
            method: "POST", body: formData
            });
        }
        else {console.log("not uploading, blob size"+audioBlob.size)}

        chunks = [];
        setRecorderState((prevState) => {
          if (prevState.mediaRecorder) {
            return {
              ...initialState,
              audio: window.URL.createObjectURL(audioBlob),
            };
          } else {
            return initialState;
          }
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
      startRecording(setRecorderState)},
    cancelRecording: () => {
      setRecorderState(initialState)},
    saveRecording: () => {
      saveRecording(recorderState.mediaRecorder)},
  };
}
