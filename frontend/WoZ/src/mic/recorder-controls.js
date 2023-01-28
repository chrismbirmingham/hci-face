export async function startRecording(setRecorderState) {
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
