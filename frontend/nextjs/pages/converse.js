import React from "react";
import Face from "@components/Face";
import Converse from "@components/Converse";
import Link from 'next/link';

export default function about() {
  return (
    <div>
      <Face />
      <Converse />
      <Link href="/">Back to home</Link>
    </div>
  );
}