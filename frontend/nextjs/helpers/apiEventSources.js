import getAUs from '@helpers/AUtransformers/Visemes';
import getExpresionAUs from '@helpers/AUtransformers/Expressions';
import {server_ip} from '@constants/serverip'

function sourceVisemes (mouthUpdater) {
    const es = new EventSource(server_ip+"/api/viseme_stream");
    es.addEventListener('open', () => {});

    // faceControls should handle vizemes, eyeAU, browAU, mouthAU
    es.addEventListener('viseme', (e) => {
      var auToUpdate = getAUs(e.data);
      mouthUpdater(auToUpdate);
    });

    es.addEventListener('error', (e) => {
      console.error('Error: ',  e);
    });

    return () => {
      es.close();
    };
  }

function sourceFaceCommands (mouthUpdater, browUpdater, eyeUpdaterWrapper, setBehavior) {
  let full_ip = server_ip+"/api/face_stream"
    const es = new EventSource(full_ip);
    es.addEventListener('open', () => {});

    // faceControls should handle emotion, eyeAU, browAU, mouthAU
    es.addEventListener('expression', (e) => {
      var [MouthAU, EyeAU, BrowAU] = getExpresionAUs(e.data)
      mouthUpdater(MouthAU);
      browUpdater(BrowAU);
      eyeUpdaterWrapper(EyeAU);
    });

    es.addEventListener('behavior', (e) => {
      setBehavior(e.data)
    });

    es.addEventListener('error', (e) => {console.error('Error: ',  e)});

    return () => {es.close();};
}

function getTranscribedSpeech(setLatestSpeech) {
  const es = new EventSource(server_ip+"/api/transcribed_speech");
  es.addEventListener('open', () => {
  });  
  es.addEventListener('message', (e) => {
    let speech = e.data;
    if (speech.length>0){
      setLatestSpeech(speech);
    };
  });
  es.addEventListener('error', (e) => {
    if (e.readyState === EventSource.CLOSED) {
      // Connection was closed.
      console.log("Connection closed");
    }
    else if (e.readyState === EventSource.CONNECTING) {
      // Connection is attempting to reconnect.
      console.log("Connection is attempting to reconnect");
    }
    else if (e.readyState === EventSource.OPEN) {
      // Connection is open and ready to communicate.
      console.log("Connection is open");
    }
  });

  return () => {
    es.close();
  };
}

function getBotResponse(setBotResponse) {
  const es = new EventSource(server_ip+"/api/bot_response");
  es.addEventListener('open', () => {
  });  
  es.addEventListener("message", (e) => {
    let bot_says = e.data
    console.log("STT process: ", bot_says, " from bot")
    setBotResponse(bot_says);
  });
  es.addEventListener('error', (e) => {
    if (e.readyState === EventSource.CLOSED) {
      // Connection was closed.
      console.log("Connection closed");
    }
    else if (e.readyState === EventSource.CONNECTING) {
      // Connection is attempting to reconnect.
      console.log("Connection is attempting to reconnect");
    }
    else if (e.readyState === EventSource.OPEN) {
      // Connection is open and ready to communicate.
      console.log("Connection is open");
    }
  });

  return () => {
    es.close();
  };
}

export {sourceVisemes, sourceFaceCommands, getTranscribedSpeech, getBotResponse}