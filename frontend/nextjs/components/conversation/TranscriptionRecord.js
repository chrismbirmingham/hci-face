// A conversation is a list of statements, where a statement is a speaker and a string of text.

import React, { useState, useEffect } from "react";


const Transcription = ({ history, setHistory }) => {
    const [conversationHistory, setConversationHistory] = useState(history.join("\n"));


    useEffect(() => {
        setConversationHistory(history.join("\n"));
    }, [history]);

    useEffect(() => {
        setHistory(conversationHistory.split("\n"));
    }, [conversationHistory]);

    return (
        <div className="max-w-fit p-6 bg-emerald-300 border border-gray-200 rounded-lg shadow dark:bg-gray-800 dark:border-gray-700">
        <header className="Conversation-header"></header>
        <div id="transcription">
            <h2>Transcribed Data:</h2>
            <textarea 
                cols={100}
                rows={history.length+1}
                value={conversationHistory}
                onChange={(e) => setConversationHistory(e.target.value)}
                />
        </div>
        <input type="button" value="Clear Data" onClick={() => setConversationHistory("")}/>
        {/* Button to copy the text to the clipboard */}
        <input type="button" value="Copy to Clipboard" onClick={() => navigator.clipboard.writeText(conversationHistory)}/>
        </div>
    );
}

export default Transcription;