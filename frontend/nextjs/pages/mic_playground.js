import React from "react";
import Microphone from "@components/Microphone";

const Playground = () => {
  return (
    <div className="bg-green-600">
      <h1 className="text-4xl">Microphone Playground</h1>
      <br></br>
      <p>Use the playground to fine tune settings for your microphone.</p>
      <br></br>
      <Microphone />
    </div>
  );
};

export default Playground;