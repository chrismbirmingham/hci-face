import React, { useState, useEffect } from "react";
import { getBotResponse } from "@helpers/apiEventSources";
import {requestConversationResponse} from "@helpers/apiRequests"
import Prompt from "./conversation/Prompt";
import GetInput from "./conversation/GetInput";
import Transcription from "./conversation/TranscriptionRecord";

const Conversation = ({ classes }) => {
  const [conversationHistory, setConversationHistory] = useState([""]);
  const [latestSpeech, setLatestSpeech] = useState("");
  const [prompt, setPrompt] = useState("The following is a conversation with an AI assistant. The assistant is helpful, creative, clever, and very friendly.\n");
  const [botResponse, setBotResponse] = useState("");
  const [speaker, setSpeaker] = useState("Chris");
  
  // const [audioPlaying, setAudioPlaying] = useState(false);
  // const [beginConversation, setBeginConversation] = useState(true);
  // const [speakerVoice, setSpeakerVoice] = useState("Emma");


  // Respond to human speech by displaying conversation history and the most recent speech
  function respondToHumanSpeech () {
    console.log("Latest speech: ", latestSpeech)
    if (latestSpeech) {console.log("latest speech evaluated to true")}
    if (latestSpeech !== "") {
      console.log("STT process: ", latestSpeech, " from ", speaker)
      setConversationHistory(oldData => [speaker+": "+latestSpeech, ...oldData ]);
      requestConversationResponse(speaker, latestSpeech, prompt, conversationHistory, setBotResponse)
      // setBeginConversation(false)
    }
  }
  
  function respondToBotSpeech () {
    console.log("Bot response: ", botResponse)
    if (botResponse) {console.log("bot response evaluated to true")}
    if (botResponse !== "") {
      // Update the conversation history
      setConversationHistory(oldData => ["bot: "+botResponse, ...oldData ])
      // Make the speaker say it
      // requestSpeech(botResponse, setAudioPlaying, speakerVoice)
    }
  }

  useEffect(() => {getBotResponse(setBotResponse)}, []);
  useEffect(respondToHumanSpeech,[latestSpeech])
  useEffect(respondToBotSpeech,[botResponse])


  return (
    <div id="divid" className="Conversation">
      <header className="Conversation-header"></header>
      <Prompt prompt={prompt} setPrompt={setPrompt}/>
      <GetInput setLatestSpeech={setLatestSpeech}/>
      <Transcription history={conversationHistory} setHistory={setConversationHistory}/>
    </div>
  );
}

export default Conversation;