import React, { useState, useEffect } from "react";
import Mic from '@components/mic/Microphone';
import AutoMic from '@components/mic/AutoMic';
import { getTextStream } from "@helpers/apiEventSources";
import {requestConversationResponse, requestSpeech} from "@helpers/apiRequests"

const Conversation = ({ classes }) => {
  // Variables with state
  const [conversationHistory, setConversationHistory] = useState([""]);
  const [audioPlaying, setAudioPlaying] = useState(false);
  const [latestSpeech, setLatestSpeech] = useState("");
  const [prompt, setPrompt] = useState("The following is a conversation with an AI assistant. The assistant is helpful, creative, clever, and very friendly.\n");
  const [botResponse, setBotResponse] = useState("");
  
  
  const [beginConversation, setBeginConversation] = useState(true);
  const [speakerVoice, setSpeakerVoice] = useState("p270");
  const [speaker, setSpeaker] = useState("Chris");



  function respondToHumanSpeech () {
    if (latestSpeech !== "") {
      console.log("STT process: ", latestSpeech, " from ", speaker)
      setConversationHistory(oldData => [speaker+": "+latestSpeech, ...oldData ]);
      requestConversationResponse(speaker, latestSpeech, prompt, conversationHistory, setBotResponse)
      setBeginConversation(false)
    }
  }
  
  function respondToBotSpeech () {
    if (botResponse !== "") {
      console.log("Bot response: " + botResponse)
      setConversationHistory(oldData => ["bot: "+botResponse, ...oldData ])
      requestSpeech(botResponse, setAudioPlaying, speakerVoice)
    }
  }

  useEffect(() => {getTextStream(setLatestSpeech, setBotResponse)}, []);

  useEffect(respondToHumanSpeech,[latestSpeech])
  useEffect(respondToBotSpeech,[botResponse])


  return (
    <div className="Conversation">
      <header className="Conversation-header"></header>
      {audioPlaying? <p>Audio is playing</p>: <AutoMic/> }
      <Mic/>
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