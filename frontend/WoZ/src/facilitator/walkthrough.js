import React from "react"

export default function Walkthrough ({showWalkthrough, do_tts, get_preset, switch_condition}) {

    return(
        <div id="walkthrough" hidden={showWalkthrough}>
            1- Start by reviewing consent <br></br> 
            ---<input type="checkbox"/>Being in the study is voluntary <br></br>
            ---<input type="checkbox"/>You will interact with a robot and with the others on this call. <br></br>
            ---<input type="checkbox"/>The primary risk in this study is that you may be uncomfortable answering a question posed in this study. You may decline to answer any question. <br></br>
            ---<input type="checkbox"/>You will not recieve any direct benefit from this study <br></br>
            ---<input type="checkbox"/>This study will be recorded. <br></br>
            ---<input type="checkbox"/>Do you consent to be a part of this study? You may withdraw your consent at any time. <br></br>
            <input type="checkbox"/>Please complete the first four pages of the survey linked in the chat, and return to the zoom session when directed to stop:<br></br>    
            <input type="checkbox"/>https://usc.qualtrics.com/jfe/form/SV_diE2Ow6GQPlSCP4 <br></br>
            <br></br>
                <button onClick={() => do_tts("Testing, 1, 2, 3. Can you all hear me?")}>Speech Test</button>
            <br></br>
            2- <button onClick={() => get_preset("f_qt-intro")}>QT introduction</button>--
                <button onClick={() => get_preset("g_QT/hi")}>wave</button>--
            <button style={{backgroundColor:"grey"}} onClick={() => get_preset("f_survey-prompt")}>survey prompt</button>--
            <button style={{backgroundColor:"grey"}} onClick={() => get_preset("f_survey-return")}>survey return</button>
            <br></br>
            <br></br>
            3- <button onClick={() => get_preset("f_group-intro")}>group introductions</button>--
                <button onClick={() => get_preset("g_QT/emotions/shy")}>shy</button>--(let them respond)--
            <button style={{backgroundColor:"green"}} onClick={() => get_preset("f_invitation")}>invitation to start</button>--
            <button style={{backgroundColor:"orange"}} onClick={() => get_preset("f_closing")}>closing</button>--
            <button style={{backgroundColor:"red"}} onClick={() => get_preset("f_transition")}>End section</button>--
            <button style={{backgroundColor:"grey"}} onClick={() => get_preset("f_survey-prompt")}>survey-prompt</button>--
            <button style={{backgroundColor:"grey"}} onClick={() => get_preset("f_survey-return")}>survey-return</button>
            <br></br>
            <br></br>
            4- 
            <button id="condition" onClick={switch_condition}>Switch Conditions:</button>--
            <button style={{backgroundColor:"green"}} onClick={() => get_preset("f_invitation")}>invitation</button>--
            <button style={{backgroundColor:"orange"}} onClick={() => get_preset("f_closing")}>closing</button>--
            <button style={{backgroundColor:"red"}} onClick={() => get_preset("f_transition")}>End section</button>--
            <button style={{backgroundColor:"grey"}} onClick={() => get_preset("f_survey-prompt")}>survey-prompt</button>--
            <button style={{backgroundColor:"grey"}} onClick={() => get_preset("f_survey-return")}>survey-return</button>
            <br></br>
            <br></br>
            5- Ask participants to complete the final survey questions (3 pages)
            <br></br>
            6- Lead participants through group discussion<br></br>
            ---<input type="checkbox"/>What did you like and dislike about interacting with QT? <br></br>
            ---<input type="checkbox"/>What did you learn as part of the group today?<br></br>
            ---<input type="checkbox"/>Do you think that QT understood you? <br></br>
            ---<input type="checkbox"/>What differences did you notice between round 1 and round 2 of the support group?<br></br>
            ---<input type="checkbox"/>How did the support group today affect your stress level? <br></br>
        </div>
    )
}