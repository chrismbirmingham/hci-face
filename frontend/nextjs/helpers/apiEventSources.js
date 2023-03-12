import React from "react"
import getAUs from '@helpers/AUtransformers/Visemes';
import getExpresionAUs from '@helpers/AUtransformers/Expressions';


const server_ip = "0.0.0.0"
function sourceVisemes (mouthUpdater) {
    const es = new EventSource("http://"+server_ip+":8000/api/viseme_stream");
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
    const es = new EventSource("http://"+server_ip+":8000/api/face_stream");
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

function getTextStream(setLatestSpeech, setBotResponse) {
  const es = new EventSource("http://"+server_ip+":8000/api/text_stream");
  es.addEventListener('open', () => {
  });

  es.addEventListener('human_speech', (e) => {
    let speech = e.data;
    if (speech.length>0){
      let full_speech = speech
      // console.log("STT process: ", speech, " from ")
      // if (participantSpeaker === priorSpeaker){
      //   full_speech = latestSpeech + " " + speech;
      // }
      setLatestSpeech(full_speech);
      // setTranscribedData(oldData => [participantSpeaker+":"+speech,  <br></br>, ...oldData ]);
      // setPriorSpeaker(participantSpeaker);
      // setClassifications("");
      // setBeginConversation(false);

      // if (full_speech.length > 2) {
      //   console.log(full_speech, full_speech.length)
      //   requestBotResponseCB(full_speech);
      // }
    };
  });

  es.addEventListener("bot_response", (e) => {
    let bot_says = e.data
      console.log("STT process: ", bot_says, " from bot")
      setBotResponse(bot_says);
    // setTranscribedData(oldData => ["bot:"+bot_says,  <br></br>, ...oldData ])
    // requestSpeech(bot_says)
  });

  es.addEventListener('error', (e) => {
    console.error('Error: ',  e);
  });

  return () => {
    es.close();
  };
}

export {sourceVisemes, sourceFaceCommands, getTextStream}