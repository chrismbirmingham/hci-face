import React from "react"
import { polly_voices, coqui_voices, expressions, behaviors} from "@constants/choices"
import { set_dropdown } from "@helpers/controls"


export default function RobotControls ({
    showForm, 
    do_tts, 
    setTextToSay, 
    textToSay, 
    behavior, 
    update_behavior, 
    expression, 
    update_expression, 
    speakerVoice, 
    setSpeakerVoice, 
    viseme, 
    update_viseme}) {
    return(

    <div id="formcontent" hidden={!showForm}>
        <h4>Robot Control Requires Backend</h4>
        {set_dropdown("Behaviors", behavior, update_behavior, behaviors)}
        {set_dropdown("Expressions", expression, update_expression, expressions)}
        <label>Enter viseme to show:
            <input type="text" value={viseme} onChange={(e) => update_viseme(e.target.value)}/>
        </label>
        <br></br><br></br>
        {set_dropdown("Polly Voices", speakerVoice, setSpeakerVoice, polly_voices)}
        {set_dropdown("Coqui Voices", speakerVoice, setSpeakerVoice, coqui_voices)}
        <br></br>
        <div>
        </div>
        <br></br>
        <label>Enter the text you would like the robot to say:
            <textarea 
            cols={100}
            rows={4}
            value={textToSay}
            onChange={(e) => setTextToSay(e.target.value)}
            />
        </label>
        <br></br><button onClick={() => do_tts(textToSay)}>Say Text From Form: {textToSay}</button><br></br>
    </div>
    )
}