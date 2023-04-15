import React, { useState } from "react";
import ButtonMic from '@components/mic/ButtonMicrophone';
import SoundWaveform from "@components/mic/AudioVisualizer";
import VoiceMic from "@components/mic/VoiceMicrophone";
import { set_dropdown } from "@helpers/controls"

function renderSwitch(param, audioPlaying) {
    // This function takes in two parameters, param and audioPlaying.

    // The param parameter determines what type of component is returned.
    // The audioPlaying parameter determines whether the VoiceMic component
    // is rendered with the audio playing or not.

    // The switch statement below checks the value of param, and returns
    // a different component depending on the value of param.
    switch(param) {
        case "buttons":
            return <ButtonMic/>;
        case "voice":
            return <VoiceMic audioPlaying={audioPlaying}/>;
        case "soundwave visual":
            return <SoundWaveform />;
        case "show all":
            return (
                <div>
                    <ButtonMic/>
                    <VoiceMic audioPlaying={audioPlaying}/>
                    <SoundWaveform />
                </div>
            );
        case "show none":
            return <p>Not recording</p>;
        default:
            return <p>Not recording</p>;
    }
}


const Microphone = ({ audioPlaying=false, defaultMic="soundwave visual" }) => {
    const [recording, setRecording] = useState(defaultMic);

    return (
        <div className="Microphone p-6">
            <header className="Microphone-header"></header>
            {set_dropdown("Listening Method", recording, setRecording, ["buttons", "voice", "soundwave visual", "show all", "show none"])}
            <div id="renderRecording">
                {renderSwitch(recording, audioPlaying)}
            </div>
        </div>
    );
}

export default Microphone;