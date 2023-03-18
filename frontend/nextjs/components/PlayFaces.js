import React, { useReducer, useState, useEffect, useCallback } from "react";
import Head from './faces/Head';
import getExpresionAUs from '@helpers/AUtransformers/Expressions';
import doBehavior from '@helpers/AUtransformers/Behaviors';
import {sourceVisemes, sourceFaceCommands} from "@helpers/apiEventSources";
import {positions, initialBrowAU, initialEyeAU, initialMouthAU} from "@constants/initialface"
import {EyeForm, BrowForm, MouthForm} from '@components/Forms/Form';

const PlayFaces = ({ classes }) => {
  const [behavior, setBehavior] = useState("focused");
  const [display, setDisplay] = useState("qt");

  // Variables with state
  const [mouthAU, updateMouthAU] = useReducer((mouthAU, updates) => ({ ...mouthAU, ...updates }),initialMouthAU);
  const [eyeAU, updateEyeAU] = useReducer((eyeAU, updates) => ({ ...eyeAU, ...updates }),initialEyeAU);
  const [browAU, updateBrowAU] = useReducer((browAU, updates) => ({ ...browAU, ...updates }),initialBrowAU);

  function mouthUpdater (AU) { updateMouthAU({ ...AU })}
  function browUpdater (AU) { updateBrowAU({ ...AU })}
  function eyeUpdater (AU) { updateEyeAU({ ...eyeAU, ...AU })}

  const eyeUpdaterWrapper = useCallback(eyeUpdater,[eyeAU])

  const faces = ["qt", "default", "cordial", "qt_head"]

  // Unclear if behaviors belong here tbh
  function runBehaviors() {
    var count = 0
    var update_interval_ms = 500
    // console.log("running behaviors", behavior)
    const interval = setInterval(() => {
    count = doBehavior(count, behavior, browUpdater, mouthUpdater, getExpresionAUs, eyeUpdaterWrapper)

    }, update_interval_ms);
    return () => clearInterval(interval);
  }


  useEffect(() => {sourceVisemes(mouthUpdater)} , []);
  useEffect(() => {sourceFaceCommands(mouthUpdater, browUpdater, eyeUpdaterWrapper, setBehavior)
  }, [eyeUpdaterWrapper]);

  useEffect(runBehaviors, [behavior, eyeUpdaterWrapper]);

  return (
    <div className="Face">
      <header className="Face-header"></header>
      
      <div style={{"float":"left","width":"20%"}} id="robot-container">
        <div id="bot" className="neutral">
            {faces.map((f) => <Head face={f} position={positions} eyeAU={eyeAU} browAU={browAU} mouthAU={mouthAU} />)}
        </div>
      </div>
    <label>Face:
    <select value={display} 
        multiple={false}
        onChange={(e) => setDisplay(e.target.value)}>
        <option value="qt">qt</option>
        <option value="default">default</option>
        <option value="cordial">cordial</option>
        <option value="qt_head">qt_head</option>
        </select>
    </label>
    <div style={{"float":"right"}} id="AU control">
        <EyeForm v={eyeAU} f={eyeUpdaterWrapper}/>
        <BrowForm v={browAU} f={browUpdater}/>
        <MouthForm v={mouthAU} f={mouthUpdater}/>
    </div>
    </div>
  );
}

export default PlayFaces;