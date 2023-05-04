import React, { useReducer, useState, useEffect, useCallback } from "react";
import HeadDisplay from './faces/HeadDisplay';
import getExpresionAUs from '@helpers/AUtransformers/Expressions';
import doBehavior from '@helpers/AUtransformers/Behaviors';
import {sourceVisemes, sourceFaceCommands} from "@helpers/apiEventSources";
import {head_settings, initialBrowAU, initialEyeAU, initialMouthAU} from "@constants/initialface"
import { set_dropdown } from "@helpers/controls";
import { behaviors, faces, expressions } from "@constants/choices";

const Face = ({ classes }) => {
  const [expression, setExpression] = useState("neutral");
  const [behavior, setBehavior] = useState("focused");
  const [display, setDisplay] = useState("qt");

  const [mouthAU, updateMouthAU] = useReducer((mouthAU, updates) => ({ ...mouthAU, ...updates }),initialMouthAU);
  const [eyeAU, updateEyeAU] = useReducer((eyeAU, updates) => ({ ...eyeAU, ...updates }),initialEyeAU);
  const [browAU, updateBrowAU] = useReducer((browAU, updates) => ({ ...browAU, ...updates }),initialBrowAU);

  function mouthUpdater (AU) { updateMouthAU({ ...AU })}
  function browUpdater (AU) { updateBrowAU({ ...AU })}
  function eyeUpdater (AU) { updateEyeAU({ ...eyeAU, ...AU })}

  const eyeUpdaterWrapper = useCallback(eyeUpdater,[eyeAU])

  function runBehaviors() {
    var count = 0
    var update_interval_ms = 500
    const interval = setInterval(() => {
      count = doBehavior(count, behavior, 
        browUpdater, mouthUpdater, getExpresionAUs, eyeUpdaterWrapper)}, 
      update_interval_ms);
    return () => clearInterval(interval);
  }

  function runExpression() {
    var [MouthAU, EyeAU, BrowAU] = getExpresionAUs(expression)
    mouthUpdater(MouthAU);
    browUpdater(BrowAU);
    eyeUpdaterWrapper(EyeAU)
  }

  useEffect(() => {sourceVisemes(mouthUpdater)} , []);
  useEffect(() => {sourceFaceCommands(mouthUpdater, browUpdater, eyeUpdaterWrapper, setBehavior)
  }, [eyeUpdaterWrapper]);

  useEffect(runBehaviors, [behavior]);
  useEffect(runExpression, [expression]);

  return (
    <div className="Face">
      <header className="Face-header"></header>
      <div id="robot-container" className="max-w-2xl">
          <HeadDisplay face={display} head_settings={head_settings} eyeAU={eyeAU} browAU={browAU} mouthAU={mouthAU} />
      </div>
    {set_dropdown("Face", display, setDisplay, faces)}
    {set_dropdown("Expression", expression, setExpression, expressions)}
    {set_dropdown("Behavior", behavior, setBehavior, behaviors)}
    </div>
  );
}

export default Face;