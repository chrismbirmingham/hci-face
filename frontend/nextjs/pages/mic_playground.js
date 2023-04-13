import React from "react";
import Microphone from "@components/Microphone";

const Playground = () => {
  return (
    <div className="text-3xl font-bold underline">
      <h1>Microphone Playground</h1>
      <p>Use the playground to fine tune settings for your microphone.</p>
      <Microphone />
      <a href="/">Back to home</a>
    </div>
  );
};

export default Playground;