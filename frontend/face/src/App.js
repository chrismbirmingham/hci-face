import './App.css';
import React, { useReducer, useState, useEffect, useCallback } from "react";
import {EyeForm, BrowForm, MouthForm} from './components/Form';
import Head from './components/Head';
import getAUs from './helpers/Visemes';
import getExpresionAUs from './helpers/Expressions';
import doBehavior from './helpers/Behaviors';


// Update these as needed
const server_ip = "192.168.1.150" // Set server IP (can be found with ifconfig on host)
const form=false // display form input if manually controlling action units

const App = ({ classes }) => {
  const [behavior, setBehavior] = useState("focused");
  let display="qt"

  // Initial positions
  const positions = {
    right_eye: {x:22, y:-10},
    left_eye: {x:-22, y:-10},
    right_brow: {x:20, y:-14, auScaler: 4, flip:-1},
    left_brow: {x:-20, y:-14, auScaler: 4, flip:1},
    mouth: {x:0, y:16, auScaler: 6},
  }
  const initialMouthAU = {
    au10_raise_upper: 0,
    au12_lip_corners_out: 0.2,
    au13_cheek_puffer: 0,
    au14_dimpler: 0,
    au15_lip_corner_depr: 0,
    au16_lower_lip_depr: 0,
    au17_chin_raiser: 0,
    au18_lip_pucker: 0,
    au20_lip_stretcher: 0,
    au22_lip_funneler: 0,
    au23_lip_tightener: 0,
    au24_lip_pressor: 0,
    au25_lips_part: 0,
    au26_jaw_drop: 0,
    au27_mouth_stretch: 0,
    au28_lip_suck: 0
  };
  const initialBrowAU = {
    au1_inner_brow_raiser: 0,
    au2_outer_brow_raiser: 0,
    au4_brow_lowerer: 0,
  };  
  const initialEyeAU = {
    au5_upper_lid_raiser: 0,
    au6_cheek_raiser: 0,
    au7_lid_tightener: 0,
    au41_lid_droop: 0,
    au42_slit: 0,
    au43_eyes_closed: 0,
    au44_squint: 0.6,
    au45_blink: 0,
    au61_left: 0,
    au62_right: 0,
    au63_up: 0,
    au64_down: 0,
  };  

  // Variables with state
  const [mouthAU, updateMouthAU] = useReducer((mouthAU, updates) => ({ ...mouthAU, ...updates }),initialMouthAU);
  const [eyeAU, updateEyeAU] = useReducer((eyeAU, updates) => ({ ...eyeAU, ...updates }),initialEyeAU);
  const [browAU, updateBrowAU] = useReducer((browAU, updates) => ({ ...browAU, ...updates }),initialBrowAU);


  function mouthUpdater (AU) { updateMouthAU({ ...AU })}
  function browUpdater (AU) { updateBrowAU({ ...AU })}
  function eyeUpdater (AU) { updateEyeAU({ ...eyeAU, ...AU })}
  const eyeUpdaterWrapper = useCallback(eyeUpdater,[eyeAU])

  function sourceVisemes () {
    const es = new EventSource("http://"+server_ip+":8000/api/viseme_stream");
    es.addEventListener('open', () => {});

    // faceControls should handle vizemes, eyeAU, browAU, mouthAU
    es.addEventListener('viseme', (e) => {
      var auToUpdate = getAUs(e.data);
      mouthUpdater(auToUpdate);
    });

    es.addEventListener('error', (e) => {
      console.error('Error: ',  e);
    });

    return () => {
      es.close();
    };
  }

  function sourceFaceCommands () {
    const es = new EventSource("http://"+server_ip+":8000/api/face_stream");
    es.addEventListener('open', () => {});

    // faceControls should handle emotion, eyeAU, browAU, mouthAU
    es.addEventListener('expression', (e) => {
      var [MouthAU, EyeAU, BrowAU] = getExpresionAUs(e.data)
      mouthUpdater(MouthAU);
      browUpdater(BrowAU);
      eyeUpdaterWrapper(EyeAU);
    });

    es.addEventListener('behavior', (e) => {
      setBehavior(e.data)
    });

    es.addEventListener('error', (e) => {console.error('Error: ',  e)});

    return () => {es.close();};
  }

  function runBehaviors() {
    var count = 0
    var update_interval_ms = 500
    console.log("running behaviors", behavior)
    const interval = setInterval(() => {
      if (!form){
        // console.log("No form, so do behavior")
        count = doBehavior(count, behavior, browUpdater, mouthUpdater, getExpresionAUs, eyeUpdaterWrapper)
      }
      else {
        count = doBehavior(count, "", browUpdater, mouthUpdater, getExpresionAUs, eyeUpdaterWrapper)
      }
    }, update_interval_ms);
    return () => clearInterval(interval);
  }


  useEffect(sourceVisemes , []);
  useEffect(sourceFaceCommands, [eyeUpdaterWrapper]);
  useEffect(runBehaviors, [behavior, eyeUpdaterWrapper]);

  return (
    <div className="App">
      <header className="App-header"></header>
      {form ?
      <div>
        <div style={{"float":"left","width":"50%"}} id="robot-container">
          <div id="bot" className="neutral">
            <Head face={display} position={positions} eyeAU={eyeAU} browAU={browAU} mouthAU={mouthAU} />
          </div>
        </div>
        <div style={{"float":"right"}} id="AU control">
          <EyeForm v={eyeAU} f={eyeUpdaterWrapper}/>
          <BrowForm v={browAU} f={browUpdater}/>
          <MouthForm v={mouthAU} f={mouthUpdater}/>
        </div>
      </div>
      : 
      <div id="robot-container">
      <div id="bot" className="neutral">
        <Head face={display} position={positions} eyeAU={eyeAU} browAU={browAU} mouthAU={mouthAU} />
      </div>
      </div>
      }
    </div>
  );
}

export default App;