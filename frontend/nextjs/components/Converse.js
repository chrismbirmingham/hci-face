import React, { useState, useEffect } from "react";
import Mic from '@components/mic/Microphone';
import { getTextStream } from "@lib/apiEventSources";
import {requestBotResponse, requestSpeech} from "@lib/apiRequests"

const Converse = ({ classes }) => {
  // Variables with state
  const [transcribedData, setTranscribedData] = useState([""]);
  const [isRecording, setIsRecording] = useState(true);
  const [latestSpeech, setLatestSpeech] = useState("");
  const [botResponse, setBotResponse] = useState("");
  
  
  const [beginConversation, setBeginConversation] = useState(true);
  const [speakerVoice, setSpeakerVoice] = useState("p270");
  const [participantSpeaker, setParticipantSpeaker] = useState("Chris");



  /////// Play Speech ///////
  function ttsWrapper(text) {
    console.log("Will say"+text)
    setTranscribedData(oldData => ["bot: "+text, <br></br>, ...oldData ])
    requestSpeech(text, setIsRecording, speakerVoice)
    return false
  }

  function respondToHumanSpeech () {
    console.log("STT process: ", latestSpeech, " from ", participantSpeaker)
    setTranscribedData(oldData => [participantSpeaker+":"+latestSpeech,  <br></br>, ...oldData ]);
    requestBotResponse(latestSpeech, beginConversation, participantSpeaker)
    setBeginConversation(false)
  }
  function respondToBotSpeech () {
    console.log("Bot response: " + botResponse)
    ttsWrapper(botResponse)
  }

  useEffect(() => {getTextStream(setLatestSpeech, setBotResponse)}, []);
  useEffect(respondToHumanSpeech,[latestSpeech])
  useEffect(respondToBotSpeech,[botResponse])


  return (
    <div className="Converse">
      <header className="Converse-header"></header>
      <Mic isRecording={isRecording}/>
      <div id="transcription">
        <h2>Transcribed Data:</h2>
        <p>{transcribedData}</p>
      </div>
    </div>
  );
}

export default Converse;