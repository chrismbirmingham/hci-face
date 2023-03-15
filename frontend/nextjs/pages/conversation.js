import React from "react";
import Face from "@components/Face";
import Conversation from "@components/Conversation";
import Link from 'next/link';

export default function about() {
  return (
    <div>
      <Face />
      <Conversation />
      <Link href="/">Back to home</Link>
    </div>
  );
}