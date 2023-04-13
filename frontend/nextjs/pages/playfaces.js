import React from "react";
import Face from "@components/Face";
import ControlAUs from "@components/ControlAUs";

export default function DisplayControlAUs() {

  return (
    <div>
      <Face />
      <ControlAUs />
      <a href="/">Back to home</a>
    </div>
  );
}