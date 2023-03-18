import React from "react"

const server_ip = "127.0.0.1"


function requestFaceUpdate(name, type){
    const name_text = name
    const type_text = type
    return fetch(`http://`+server_ip+`:8000/api/face_presets?text=${encodeURIComponent(name_text)}&update_type=${encodeURIComponent(type_text)}`, { cache: 'no-cache' })
        .then(response => response.text())
        .then(message => {console.log(message)})
}

function requestBotResponse(human_input, beginConversation, participantSpeaker) {
    const text = human_input
    const speaker = participantSpeaker
    if (text) {
      return fetch(`http://`+server_ip+`:8000/api/bot_response?text=${encodeURIComponent(text)}&speaker=${encodeURIComponent(speaker)}&reset_conversation=${encodeURIComponent(beginConversation)}&director_condition=${encodeURIComponent(false)}`, { cache: 'no-cache' })
        .then(response => response.text())
        .then(message => {console.log(message); return message})
    }
}
function requestSpeech(text, setAudioPlaying, speakerVoice){
    const speaker_id = [speakerVoice]
    const style_wav = ""
    fetch(`http://`+server_ip+`:8000/api/speech?text=${encodeURIComponent(text)}&speaker_id=${encodeURIComponent(speaker_id)}&style_wav=${encodeURIComponent(style_wav)}`, { cache: 'no-cache' })
    .then(function (res) {
        if (!res.ok) throw Error(res.statusText)
        return res.blob()
    }).then(function (blob) {
        const audioUrl = URL.createObjectURL(blob)
        const audio = new Audio(audioUrl)
        setAudioPlaying(true)
        audio.play();
        audio.addEventListener('ended', () => {setAudioPlaying(false)});
    }).catch(function (err) {
    })
}
function get_preset(mode, query, do_tts) {
    return fetch(`//localhost:8000/api/presets?mode=${encodeURIComponent(mode)}&query=${encodeURIComponent(query)}`, { cache: 'no-cache' })
    .then(response => response.text())
    .then(message => {console.log(message); do_tts(message)})
}
function set_gesture(name){
    return fetch(`//localhost:8000/api/qt_gesture?text=${encodeURIComponent(name)}`, { cache: 'no-cache' })
    .then(response => response.text())
    .then(message => {console.log(message)})
}

function requestConversationResponse(speaker, speech, prompt, history, setBotResponse) {
    const obj = { 
        speaker: speaker,
        speech: speech,
        prompt: prompt,
        history: history,
    };
    console.log(obj)

    fetch("//localhost:8000/api/conversation", {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify(obj)
    })
    .then(response => response.text())
    .then(message => {console.log("Conversation recieved: "+message); setBotResponse(message)})
}

export {requestFaceUpdate, requestBotResponse, requestSpeech, set_gesture, get_preset, requestConversationResponse }