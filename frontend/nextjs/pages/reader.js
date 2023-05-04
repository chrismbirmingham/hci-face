import React from "react";
import Reader from "@components/Reader";

export default function Transcription() {
  return (
    <div>
      <h1 className="text-4xl">Listen to Your Text</h1>
      <p>Choose the Voice and Rate and listen to the text being read.</p>
      <Reader />
    </div>
  );
}