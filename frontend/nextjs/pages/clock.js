import React from "react";
import AlarmClock from "@components/Clock/AlarmClock";
import StopWatch from "@components/Clock/StopWatch";
import CountdownTimer from "@components/Clock/Timer";

export default function DisplayClock() {

  return (
    <div className="space-y-2">
        <AlarmClock />
        <StopWatch />
        <CountdownTimer />
    </div>
  );
}