// input allows for the user to input their interaction with the chatbot. This can either be done by text or by voice.
// If using the microphone the user can choose whether to automatically accept the transcription or manually edit it.

import React, { useState, useEffect } from 'react';
import Microphone from '@components/Microphone';
import { getTranscribedSpeech } from "@helpers/apiEventSources";


const GetInput = ({setLatestSpeech, audioPlaying, auto=false}) => {
    const [boxText, setBoxText] = useState("") // Holding place for user input
    const [transcriptionResult, setTranscriptionResult] = useState("") // Holding place for user input
    const [automaticSend, setAutomaticSend] = useState(auto)


    function updateTranscription () {
        setBoxText(transcriptionResult);
        if (automaticSend) {
            setLatestSpeech(transcriptionResult);
        }
      }
    // if latest speech is updated, update the transcription
    useEffect(() => {getTranscribedSpeech(setTranscriptionResult)}, []);
    useEffect(updateTranscription,[transcriptionResult])


    return (
        <div className="max-w-fit p-6 bg-emerald-300 border border-gray-200 rounded-lg shadow dark:bg-gray-800 dark:border-gray-700">
            <Microphone audioPlaying={audioPlaying}/>
            <label>If not using the mic, you can type your text here: <br></br>
                <textarea 
                    cols={100}
                    rows={2}
                    value={boxText}
                    onChange={(e) => setBoxText(e.target.value)}
                    onKeyDown={(e) => {if (e.key == 13) setLatestSpeech(boxText)}}
                /><br></br>
            </label>
            <label>Manually edit text: <input type="checkbox" checked={!automaticSend} onChange={(e) => setAutomaticSend(!e.target.checked)}></input></label>
            <br></br>
            <input type="button" hidden={automaticSend} value="Send it." onClick={() => setLatestSpeech(boxText)}/>
        </div>
    )
}


export default GetInput;