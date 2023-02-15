import React, { useReducer, useState, useCallback } from "react";
// import {EyeForm, BrowForm, MouthForm} from '../components/Form';
import Link from 'next/link';
import RobotControls from '../components/robotControls';
import getDeadTime from '../components/timer/utils';



const Face = ({ classes }) => {

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
  // const [mouthAU, updateMouthAU] = useReducer((mouthAU, updates) => ({ ...mouthAU, ...updates }),initialMouthAU);
  // const [eyeAU, updateEyeAU] = useReducer((eyeAU, updates) => ({ ...eyeAU, ...updates }),initialEyeAU);
  // // const [browAU, updateBrowAU] = useReducer((browAU, updates) => ({ ...browAU, ...updates }),initialBrowAU);
  const [timerDeadline, setTimerDeadline] = useState("")
  const [timerLength, setTimerLength] = useState('25');
  const [speakerVoice, setSpeakerVoice] = useState("p270");
  // const [showForm, setFormToggle] = useState(false);
  const [viseme, setViseme] = useState("");
  const [expression, setExpression] = useState("");
  const [behavior, setBehavior] = useState("focused");
  // const [participantSpeaker, setParticipantSpeaker] = useState("");
  const [textToSay, setTextToSay] = useState("This is an example of what I sound like when I am talking.");

  // function mouthUpdater (AU) { updateMouthAU({ ...AU })}
  // function browUpdater (AU) { updateBrowAU({ ...AU })}
  // function eyeUpdater (AU) { updateEyeAU({ ...eyeAU, ...AU })}
  // const eyeUpdaterWrapper = useCallback(eyeUpdater,[eyeAU])
  function requestFaceUpdate(name, type){
    const name_text = name
    const type_text = type
    return fetch(`//localhost:8000/api/face_presets?text=${encodeURIComponent(name_text)}&update_type=${encodeURIComponent(type_text)}`, { cache: 'no-cache' })
      .then(response => response.text())
      .then(message => {console.log(message)})
  }

  function update_behavior(name){
    setBehavior(name)
    requestFaceUpdate(name, "behavior")
    console.log(name)
  }
  function update_viseme(name){
    setViseme(name)
    requestFaceUpdate(name, "viseme")
    console.log(name)
  }
  function update_expression(name){
    setExpression(name)
    requestFaceUpdate(name, "expression")
    console.log(name)
  }

  function ttsWrapper(text) {
    console.log("Will say"+text)
    requestSpeech(text)
    return false
  }
  function requestSpeech(text){
    const speaker_id = [speakerVoice]
    const style_wav = ""
    fetch(`//localhost:8000/api/speech?text=${encodeURIComponent(text)}&speaker_id=${encodeURIComponent(speaker_id)}&style_wav=${encodeURIComponent(style_wav)}`, { cache: 'no-cache' })
    .then(function (res) {
        if (!res.ok) throw Error(res.statusText)
        return res.blob()
    }).then(function (blob) {
        const audioUrl = URL.createObjectURL(blob)
        const audio = new Audio(audioUrl)
        audio.play();
    }).catch(function (err) {
  })
  }


  return (
    <div className="Face">
      <header className="Face-header"></header>
      <RobotControls
        setTimerDeadline={setTimerDeadline}
        getDeadTime={getDeadTime}
        showForm={true} 
        do_tts={ttsWrapper} 
        textToSay={textToSay} 
        setTextToSay={setTextToSay} 
        behavior={behavior} 
        update_behavior={update_behavior} 
        expression={expression} 
        update_expression={update_expression} 
        speakerVoice={speakerVoice} 
        setSpeakerVoice={setSpeakerVoice} 
        viseme={viseme} 
        update_viseme={update_viseme} 
        timerLength={timerLength} 
        setMinuteGoal={setTimerLength}/>
      {/* <div>
        <div style={{"float":"left"}} id="AU control">
          <EyeForm v={eyeAU} f={eyeUpdaterWrapper}/>
          <BrowForm v={browAU} f={browUpdater}/>
          <MouthForm v={mouthAU} f={mouthUpdater}/>
        </div>
      </div> */}
      <Link href="/">Back to home</Link>

    </div>
  );
}

export default Face;