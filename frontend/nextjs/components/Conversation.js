import React, { useState, useEffect } from "react";
import { getBotResponse } from "@helpers/apiEventSources";
import {requestConversationResponse} from "@helpers/apiRequests"

const Conversation = ({ classes }) => {
  const [conversationHistory, setConversationHistory] = useState([""]);
  // const [audioPlaying, setAudioPlaying] = useState(false);
  const [latestSpeech, setLatestSpeech] = useState("");
  const [prompt, setPrompt] = useState("The following is a conversation with an AI assistant. The assistant is helpful, creative, clever, and very friendly.\n");
  const [botResponse, setBotResponse] = useState("");
  const [inputText, setInputText] = useState("");
  
  // const [beginConversation, setBeginConversation] = useState(true);
  // const [speakerVoice, setSpeakerVoice] = useState("Emma");
  const [speaker, setSpeaker] = useState("Chris");


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
      setInputText("")
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
      <label>Enter the starting prompt: <br></br>
            <textarea 
                cols={100}
                rows={3}
                value={prompt}
                onChange={(e) => setPrompt(e.target.value)}
            />
      </label><br></br>
      <label>Enter the current speakers name: <br></br>
            <textarea
                value={speaker}
                onChange={(e) => setSpeaker(e.target.value)}
            />
      </label><br></br>
      <label>Enter the text you would like to say (if not using the mic): <br></br>
            <textarea 
                cols={100}
                rows={2}
                value={inputText}
                onChange={(e) => setInputText(e.target.value)}
            /><br></br>
      <input type="button" value="Say it" onClick={() => setLatestSpeech(inputText)}/>
      </label><br></br>
      <div id="transcription">
          <h2>Transcribed Data:</h2>
          {conversationHistory.map((h,ind) => <p key={ind}>{h}</p>)}
      </div>
    </div>
  );
}

export default Conversation;