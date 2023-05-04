import {server_ip} from '@constants/serverip'


function requestFaceUpdate(name, type){
    const name_text = name
    const type_text = type
    return fetch(server_ip+`/api/face_presets?text=${encodeURIComponent(name_text)}&update_type=${encodeURIComponent(type_text)}`, { cache: 'no-cache' })
        .then(response => {
            if (!response.ok) {
                throw new Error(response.statusText)
            }
            return response.text()
        })
        .then(message => {console.log(message)})
        .catch(error => console.log(error.message))
}

function requestBotResponse(human_input, beginConversation, participantSpeaker) {
    const text = human_input
    const speaker = participantSpeaker
    if (text) {
      return fetch(server_ip+`/api/bot_response?text=${encodeURIComponent(text)}&speaker=${encodeURIComponent(speaker)}&reset_conversation=${encodeURIComponent(beginConversation)}&director_condition=${encodeURIComponent(false)}`, { cache: 'no-cache' })
        .then(response => {
            if (response.ok) {
                return response.text()
            } else {
                throw new Error('We could not get a response from the server.')
            }
        })
        .then(message => {console.log(message); return message})
        .catch(error => {console.log(error); return error})
    }
}

function requestSpeech(text, setAudioPlaying, speakerVoice, rate=1.0){
    console.log("requestSpeech called")
    const speaker_id = [speakerVoice]
    const style_wav = ""
    fetch(server_ip+`/api/bot_speech?text=${encodeURIComponent(text)}&speaker_id=${encodeURIComponent(speaker_id)}&style_wav=${encodeURIComponent(style_wav)}`, { cache: 'no-cache' })
    .then(function (res) {
        if (!res.ok) throw Error(res.statusText)
        return res.blob()
    }).then(function (blob) {
        const audioUrl = URL.createObjectURL(blob)
        const audio = new Audio(audioUrl)
        setAudioPlaying(true)
        audio.playbackRate = rate
        audio.play();
        audio.addEventListener('ended', () => {
            setTimeout(()=> {setAudioPlaying(false)}, 100)
            
        });
    }).catch(function (err) {
        console.log(err)
        setAudioPlaying(false)
    })
}

function get_preset(mode, query, do_tts) {
    fetch(server_ip+`/api/presets?mode=${encodeURIComponent(mode)}&query=${encodeURIComponent(query)}`, { cache: 'no-cache' })
    .then(response => {
        if (response.ok) {
            response.text()
            .then(message => {console.log(message); do_tts(message)})
        } else {
            console.log('Error: ' + response.status);
        }
    })
    .catch(error => {
        console.log('Error: ' + error);
    })
}

function set_gesture(name){
    fetch(server_ip+`/api/qt_gesture?text=${encodeURIComponent(name)}`, { cache: 'no-cache' })
    .then(response => response.text())
    .then(message => {console.log(message)})
    .catch(error => {
        console.log('Error setting gesture', error)
    })
}

function requestConversationResponse(speaker, speech, prompt, history, setBotResponse) {
    console.log("requestConversationResponse called")
    const obj = { 
        speaker: speaker,
        speech: speech,
        prompt: prompt,
        history: history,
    };
    console.log("Sending over obj:", JSON.stringify(obj))
    fetch(server_ip+'/api/conversation', {
        method: "POST",
        headers: {
            'accept': 'application/json',
            "Content-Type": "application/json"
        },
        body: JSON.stringify(obj)
    })
    .then(response => {
        if (response.status === 200) {
            return response.text()
        } else {
            throw new Error('Something went wrong on api server!');
        }
    })
    .then(response => {
        console.log("Conversation recieved: "+response); 
        setBotResponse(response)
    }).catch(error => {
        console.error(error);
    });
}

export {requestFaceUpdate, requestBotResponse, requestSpeech, set_gesture, get_preset, requestConversationResponse }