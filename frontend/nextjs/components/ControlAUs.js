import React, { useReducer, useEffect, useCallback } from "react";
import HeadDisplay from './faces/HeadDisplay';
import {sourceVisemes, sourceFaceCommands} from "@helpers/apiEventSources";
import {EyeForm, BrowForm, MouthForm} from '@components/Forms/Form';
import { faces } from "@constants/choices";
import {head_settings, initialBrowAU, initialEyeAU, initialMouthAU} from "@constants/initialface"

const ControlAUs = ({ classes }) => {
  const [mouthAU, updateMouthAU] = useReducer((mouthAU, updates) => ({ ...mouthAU, ...updates }),initialMouthAU);
  const [eyeAU, updateEyeAU] = useReducer((eyeAU, updates) => ({ ...eyeAU, ...updates }),initialEyeAU);
  const [browAU, updateBrowAU] = useReducer((browAU, updates) => ({ ...browAU, ...updates }),initialBrowAU);

  function mouthUpdater (AU) { updateMouthAU({ ...AU })}
  function browUpdater (AU) { updateBrowAU({ ...AU })}
  function eyeUpdater (AU) { updateEyeAU({ ...eyeAU, ...AU })}

  const eyeUpdaterWrapper = useCallback(eyeUpdater,[eyeAU])

  useEffect(() => {sourceVisemes(mouthUpdater)} , []);
  useEffect(() => {sourceFaceCommands(mouthUpdater, browUpdater, eyeUpdaterWrapper, ()=>{})
  }, [eyeUpdaterWrapper]);


  return (
    <div className="Face">
      <header className="Face-header"></header>
      
      <div style={{"float":"inline-start","width":"20%"}} id="robot-container">
        <div id="bot" className="neutral">
          <h2>Available Faces:</h2>
            {Object.keys(faces).map((f) => 
              <div key={f}>
                <HeadDisplay face={f} head_settings={head_settings} eyeAU={eyeAU} browAU={browAU} mouthAU={mouthAU} />
                <p>{f}</p>
              </div>)}
        </div>
      </div>
    <div style={{"float":"left"}} id="AU control">
        <h2>Action Units</h2>
        <div style={{"float":"left"}} id="Eyes">
          <EyeForm v={eyeAU} f={eyeUpdaterWrapper}/>
          <BrowForm v={browAU} f={browUpdater}/>
        </div>
        <div style={{"float":"right"}} id="Brows">
          <MouthForm v={mouthAU} f={mouthUpdater}/>
        </div>
    </div>
    </div>
  );
}

export default ControlAUs;