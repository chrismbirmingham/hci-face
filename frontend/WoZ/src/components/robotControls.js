import React, {useState} from "react"

export default function RobotControls ({setTimerDeadline, getDeadTime, showForm, do_tts, setTextToSay, textToSay, behavior, update_behavior, expression, update_expression, speakerVoice, setSpeakerVoice, viseme, update_viseme}) {
    const [customMinutes, setCustomMinutes] = useState("")


    return(

    <div id="formcontent" hidden={!showForm}>
        <br></br>
        <label>Behavior:
        <select value={behavior} 
            multiple={false}
            onChange={(e) => update_behavior(e.target.value)}>
            <option value="none">none</option>
            <option value="focused">focused</option>
            <option value="random">random</option>
            <option value="bored">bored</option>
            </select>
        </label>
        <label>Expression:
        <select value={expression} 
            multiple={false}
            onChange={(e) => update_expression(e.target.value)}>
            <option value="joy">joy</option>
            <option value="sad">sad</option>
            <option value="surprise">surprise</option>
            <option value="neutral">neutral</option>
            </select>
        </label>
        <label> Coqui Voice:
            <select value={speakerVoice} 
            multiple={false}
            onChange={(e) => setSpeakerVoice(e.target.value)}>
            <option value="p267">British Male m</option>
            <option value="p330">British Male s</option>
            <option value="p312">British Male f</option>
            <option value="p287">British Male d</option>
            <option value="p303">British Female s</option>
            <option value="p308">British Female s2</option>
            <option value="p306">British Female m</option>
            <option value="p295">America Female s</option>
            <option value="p270">America Female s2!!!</option>
            <option value="p317">America Male s</option>
            <option value="p230">America Male f!!!</option>
            <option value="p313">Male f</option>
            </select>
        </label>
        <label> Polly Voices:
            <select value={speakerVoice} 
            multiple={false}
            onChange={(e) => setSpeakerVoice(e.target.value)}>
            <option value='Kevin'>en-US - Kevin</option> 
            <option value='Salli'>en-US - Salli</option> 
            <option value='Matthew'>en-US - Matthew</option> 
            <option value='Kimberly'>en-US - Kimberly</option> 
            <option value='Kendra'>en-US - Kendra</option> 
            <option value='Justin'>en-US - Justin</option> 
            <option value='Joey'>en-US - Joey</option> 
            <option value='Joanna'>en-US - Joanna</option> 
            <option value='Ivy'>en-US - Ivy</option>
            <option value='Aria'>en-NZ - Aria</option>
            <option value='Ayanda'>en-ZA - Ayanda</option>
            <option value='Emma'>en-GB - Emma</option> 
            <option value='Brian'>en-GB - Brian</option> 
            <option value='Amy'>en-GB - Amy</option> 
            <option value='Arthur'>en-GB - Arthur</option>
            <option value='Olivia'>en-AU - Olivia</option>
            <option value='Kajal'>en-IN - Kajal</option>
            </select>
        </label>
        <br></br><br></br>
        <div hidden={false}>
        <label>Enter viseme to show:
            <input type="text" value={viseme} onChange={(e) => update_viseme(e.target.value)}/>
        </label>
        </div>
        Set timer for: <input value={customMinutes} type="text" size={3} onChange={(e) => setCustomMinutes(e.target.value)}/>
        <button onClick={() => setTimerDeadline(getDeadTime(customMinutes))}>Run Timer</button> 
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