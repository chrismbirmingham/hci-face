import React from "react"

export default function RobotControls ({setTimerDeadline, getDeadTime, showForm, do_tts, setTextToSay, textToSay, behavior, update_behavior, expression, update_expression, speakerVoice, setSpeakerVoice, viseme, update_viseme, minuteGoal, setMinuteGoal}) {

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
        <label> Voice:
            <div id="voice notes">
                {/* # 267!,307 - English male, medium
                # 330!,232 - English male, slow
                # 312!,251 - English male, fast
                # 287,254 - English male, fast and deep
                # 303 - English female, slow
                # 306 - English female, medium
                # 308 - English female, slow
                # 295!,270 - American female, slow
                # 317! - American male, slow
                # 230! - American male, fast
                # 345 - south african female, slow
                # 313,233 - ? male, fast */}
            </div>
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
        <br></br><br></br>
        <div hidden={true}>
        <label>OR Enter viseme to show:
            <input 
            type="text" 
            value={viseme}
            onChange={(e) => update_viseme(e.target.value)}
            />
        </label>
        </div>
        Set timer for: <input value={minuteGoal} type="text" size={3} onChange={(e) => setMinuteGoal(e.target.value)}/>
        <button onClick={() => setTimerDeadline(getDeadTime(minuteGoal))}>Run Timer</button> 
        <br></br>
        <label>Enter the text you would like the robot to say:
            <textarea 
            cols={100}
            rows={4}
            // type="text" 
            value={textToSay}
            onChange={(e) => setTextToSay(e.target.value)}
            />
        </label>
        <br></br><button onClick={() => do_tts(textToSay)}>Say Text From Form: {textToSay}</button><br></br>
    </div>
    )
}