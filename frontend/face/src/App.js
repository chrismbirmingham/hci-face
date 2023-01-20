// import 'es6-symbol/implement'
import './App.css';
import React, { useReducer, useState, useEffect, useCallback } from "react";
import Head from './rendering/head';
import getAUs from './animation/processVisemes';
import getExpresionAUs from './animation/proccessExpressions';
import {randomFace, boredEyes, focusedEyes} from './animation/proccessBehaviors';
import {EyeForm, BrowForm, MouthForm} from '../../face/src/control/Form';


const App = ({ classes }) => {
  // Initial positions
  const positions = {
    right_eye: {x:22, y:-10},
    left_eye: {x:-22, y:-10},
    right_brow: {x:20, y:-14, auScaler: 4, flip:-1},
    left_brow: {x:-20, y:-14, auScaler: 4, flip:1},
    mouth: {x:0, y:16, auScaler: 6},
  }
  const initialMouthAU = {
    au10: 0,
    au12: 0.2,
    au13: 0,
    au14: 0,
    au15: 0,
    au16: 0,
    au17: 0,
    au18: 0,
    au20: 0,
    au22: 0,
    au23: 0,
    au24: 0,
    au25: 0,
    au26: 0,
    au27: 0,
    au28: 0
  };
  const initialBrowAU = {
    au1: 0,
    au2: 0,
    au4: 0,
  };  
  const initialEyeAU = {
    au5: 0,
    au6: 0,
    au7: 0,
    au41: 0,
    au42: 0,
    au43: 0,
    au44: 0.6,
    au45: 0,
    au61: 0,
    au62: 0,
    au63: 0,
    au64: 0,
  };  

  // Variables with state
  const [mouthAU, updateMouthAU] = useReducer((mouthAU, updates) => ({ ...mouthAU, ...updates }),initialMouthAU);
  const [eyeAU, updateEyeAU] = useReducer((eyeAU, updates) => ({ ...eyeAU, ...updates }),initialEyeAU);
  const [browAU, updateBrowAU] = useReducer((browAU, updates) => ({ ...browAU, ...updates }),initialBrowAU);
  function mouthUpdater (AU) { updateMouthAU({ ...AU })}
  function browUpdater (AU) { updateBrowAU({ ...AU })}
  const eyeUpdater = useCallback(function eyeUpdaterInner (AU) { updateEyeAU({ ...eyeAU, ...AU })},[eyeAU])
  const [behavior, setBehavior] = useState("bored");

  // Visemes are separated from the rest of the face control
  // so speaking and expression (aside from the lips) can happen together
  useEffect(() => {const es = new EventSource("http://192.168.1.136:8000/api/visemes");
    es.addEventListener('open', () => {
      // console.log('SSE opened@!')
    });

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
  }, []);
  useEffect(() => {const es = new EventSource("http://192.168.1.136:8000/api/faceControl");
    es.addEventListener('open', () => {
      // console.log('SSE opened@!')
    });

    // faceControls should handle emotion, eyeAU, browAU, mouthAU
    es.addEventListener('expression', (e) => {
      // console.log(e.data)
      var [MouthAU, EyeAU, BrowAU] = getExpresionAUs(e.data)
      mouthUpdater(MouthAU);
      browUpdater(BrowAU);
      eyeUpdater(EyeAU);
    });

    es.addEventListener('behavior', (e) => {
      // console.log(e.data)
      setBehavior(e.data)
    });

    es.addEventListener('error', (e) => {
      console.error('Error: ',  e);
    });

    return () => {
      es.close();
    };
  });
  useEffect(() => {
    var count = 0
    var update_interval_ms = 500
    const interval = setInterval(() => {
        switch (behavior)
        {
            case "bored":
                count = boredEyes(count, 10, eyeUpdater, mouthUpdater)
                break;
            case "focused":
                count = focusedEyes(count, 20, eyeUpdater, mouthUpdater)
                break;
            case "random":
                count = randomFace(count, 6, eyeUpdater, browUpdater, mouthUpdater, getExpresionAUs)
                break;
            default:
                break;
        }
    }, update_interval_ms);
    return () => clearInterval(interval);
  }, [behavior, eyeUpdater]);


  return (
    <div className="App">
      <header className="App-header"></header>
      <div id="robot-container">
        <div id="bot" className="neutral">
          <Head face="qt" position={positions} eyeAU={eyeAU} browAU={browAU} mouthAU={mouthAU} />
        </div>
      </div>
      <div id="AU control" hidden={true}>
        <EyeForm v={eyeAU} f={eyeUpdater}/>
        <BrowForm v={browAU} f={browUpdater}/>
        <MouthForm v={mouthAU} f={mouthUpdater}/>
      </div>
    </div>
  );
}

export default App;