import React from "react";
import Transcriber from "@components/Transcriber";
import Link from 'next/link';

export default function about() {
  return (
    <div>
      <h1>Transcribe Your Ideas</h1>
      <p>Use the microphone controls below to control your input. You may edit the text in the box below.</p>
      <Transcriber />
      <Link href="/">Back to home</Link>
    </div>
  );
}