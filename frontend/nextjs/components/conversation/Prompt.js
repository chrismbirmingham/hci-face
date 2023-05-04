// The prompt component shows the prompt that the chatbot is responding to. It also allows the user to change the prompt.


import React, { useState, useEffect } from 'react';
import {conversationPrompts} from "@constants/prompts"
// import { set_dropdown } from "@helpers/controls";

const Prompt = ({prompt, setPrompt}) => {
    const [promptChoice, setPromptChoice] = useState("default")

    useEffect(() => {
        setPrompt(conversationPrompts[promptChoice])
    },[promptChoice])

    return (
        <div className="max-w-fit p-6 bg-emerald-50 border border-gray-200 rounded-lg shadow dark:bg-gray-800 dark:border-gray-700">
            <label htmlFor="prompt">Enter the starting prompt or select one of the choices:</label>
            <select id="prompt" name="prompt" onChange={(e) => setPromptChoice(e.target.value)}>
                {Object.keys(conversationPrompts).map((key) => (
                    <option key={key} value={key}>{key}</option>
                ))}
            </select>
            <br></br>
            <textarea 
                cols={100}
                rows={3}
                value={prompt}
                onChange={(e) => setPrompt(e.target.value)}
            />
            <br></br>
            Note: You may need to modify the prompt with your specifics. <br></br>
            For example, if you are using the interviewer prompt, you will need to change the position being interviewed for.
        </div>
    )
}

export default Prompt