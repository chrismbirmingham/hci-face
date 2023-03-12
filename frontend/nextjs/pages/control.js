import React from "react";
import Face from "../components/Face";
import Controller from "@components/Controller";

export default function DisplayFace() {

  return (
    <div>
      <Face />
      <Controller />
      <a href="/">Back to home</a>
    </div>
  );
}