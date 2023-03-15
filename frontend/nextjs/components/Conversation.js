import React, { useState, useEffect } from "react";
import Mic from '@components/mic/Microphone';
import { getTextStream } from "@helpers/apiEventSources";
import {requestConversationResponse, requestSpeech} from "@helpers/apiRequests"

const Conversation = ({ classes }) => {
  // Variables with state
  const [conversationHistory, setConversationHistory] = useState([""]);
  const [isRecording, setIsRecording] = useState(true);
  const [latestSpeech, setLatestSpeech] = useState("");
  const [prompt, setPrompt] = useState("The following is a conversation with an AI assistant. The assistant is helpful, creative, clever, and very friendly.\n");
  const [botResponse, setBotResponse] = useState("This is a test");
  
  
  const [beginConversation, setBeginConversation] = useState(true);
  const [speakerVoice, setSpeakerVoice] = useState("p270");
  const [speaker, setSpeaker] = useState("Chris");



  /////// Play Speech ///////
  function ttsWrapper(text) {
    console.log("Will say"+text)
    setConversationHistory(oldData => ["bot: "+text, ...oldData ])
    requestSpeech(text, setIsRecording, speakerVoice)
    return false
  }

  function respondToHumanSpeech () {
    console.log("STT process: ", latestSpeech, " from ", speaker)
    setConversationHistory(oldData => [speaker+":"+latestSpeech, ...oldData ]);
    requestConversationResponse(speaker, latestSpeech, prompt, conversationHistory, setBotResponse)
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
    <div className="Conversation">
      <header className="Conversation-header"></header>
      <Mic isRecording={isRecording}/>
      <label>Enter the starting prompt: <br></br>
            <textarea 
            cols={100}
            rows={4}
            value={prompt}
            onChange={(e) => setPrompt(e.target.value)}
            />
        </label><br></br>
        <label>Enter the current speakers name: <br></br>
            <textarea
                value={speaker}
                onChange={(e) => setSpeaker(e.target.value)}
            />
        </label>
      <div id="transcription">
        <h2>Transcribed Data:</h2>
        {conversationHistory.map((h) => <p>{h}</p>)}
      </div>
    </div>
  );
}

export default Conversation;