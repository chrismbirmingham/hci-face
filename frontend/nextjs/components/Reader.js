import React, { useState } from "react";
import { polly_voices, coqui_voices} from "@constants/choices"
import {requestSpeech} from "@helpers/apiRequests"
import {set_dropdown} from "@helpers/controls"

const Reader = ({ }) => {
    
    const [speakerVoice, setSpeakerVoice] = useState("Aria");
    const [textToSay, setTextToSay] = useState("This is an example of what I sound like when I am talking.");
    const [speakerRate, setRate] = useState(1.0);


    function ttsWrapper(text) {
        requestSpeech(text, ()=>{}, speakerVoice, speakerRate)
        return false
    }

    return (
        <div className="Face">
            <header className="Face-header"></header>
            <h4 className="text-3xl font-bold p-2">Robot Control Requires Backend</h4>
            {set_dropdown("Polly Voices", speakerVoice, setSpeakerVoice, polly_voices)}
            {set_dropdown("Coqui Voices", speakerVoice, setSpeakerVoice, coqui_voices)}
        <br></br>
        <label>Enter the text you would like to be read:
            <textarea 
            cols={100}
            rows={4}
            value={textToSay}
            onChange={(e) => setTextToSay(e.target.value)}
            />
        </label>
        <br></br><button onClick={() => ttsWrapper(textToSay)}>Speak Text</button><br></br>
        <label>Enter the rate of speech: 
            <input type="number" value={speakerRate} onChange={(e) => setRate(e.target.value)}/>
        </label>
        </div>
    );

}

export default Reader;