import React from "react";
import Transcriber from "@components/Transcriber";

export default function Transcription() {
  return (
    <div>
      <h1 className="text-4xl">Transcribe Your Ideas</h1>
      <p>Use the microphone controls below to control your input. 
        You may edit the text in the box below.</p>
      <Transcriber />
    </div>
  );
}