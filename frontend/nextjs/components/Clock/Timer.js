import React, { useState, useEffect } from 'react';
import CircularProgressBar from '@components/Clock/ProgressBar';
import AudioPlayer from '@components/AudioPlayer';

function CountdownTimer({initialTime=25, audioFile="/test.m4a"}) {
    const [timer, setTimer] = useState(initialTime)
    const [timeRemaining, setTimeRemaining] = useState(25*60);
    const [isRunning, setIsRunning] = useState(false);


    useEffect(() => {
        let intervalId;

        if (isRunning && timeRemaining > 0) {
            intervalId = setInterval(() => {
                setTimeRemaining((prevTime) => prevTime - 1);
            }, 1000);
        } 
        
        else if (timeRemaining === 0) {
            alert("Time completed");
            setIsRunning(false);
            setTimeRemaining(timer*60)
        }
        return () => clearInterval(intervalId);
    }, [isRunning, timeRemaining]);

    function handleStart() {
        setIsRunning(true);
    }

    function handleStop() {
        setIsRunning(false);
    }

    function handleReset() {
        setTimeRemaining(timer*60);
        setIsRunning(false);
    }

    function handleTimeChange(event) {
        const time = parseInt(event.target.value, 10);
        setTimer(time)
        setTimeRemaining(time*60);
        setIsRunning(false);
    }

    function handleAddTime() {
        setTimer(((timeRemaining + 5*60)/60).toFixed(2));
        setTimeRemaining(timeRemaining + 5*60);
    }

    const seconds = timeRemaining % 60;
    const minutes = Math.floor(timeRemaining / 60);
    const button_class = "bg-gray-300 hover:bg-gray-400 text-gray-800 font-bold py-2 px-2 rounded inline-flex items-center"

    return (
        <div className="max-w-sm p-6 bg-white border border-gray-200 rounded-lg shadow dark:bg-gray-800 dark:border-gray-700">
            <h className="mb-2 text-2xl font-bold tracking-tight text-gray-900 dark:text-white">Work Session Timer</h>
            <div>
                Time Remaining: {minutes.toString().padStart(2, '0')}:{seconds.toString().padStart(2, '0')}
                <CircularProgressBar value={timeRemaining} total={timer*60} display={{units:"%"}}/>
            </div>
            <div>
                <h2 className="mb-2 text-1xl font-bold tracking-tight text-gray-900 dark:text-white">Timer Controls:</h2>
                <label>Set timer for: 
                    <input className="px-4" label="test" type="number" value={timer} onChange={handleTimeChange} style={{width:"5em"}}/>
                    minutes
                </label>
                <br></br>
                <div className="space-x-0.5">
                    <button onClick={handleStart} disabled={isRunning || timeRemaining === 0}  class={button_class}>
                        <svg class="fill-current w-4 h-4 mr-2" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 30 30">
                                <path d="M5.92 24.096q0 1.088 0.928 1.728 0.512 0.288 1.088 0.288 0.448 0 0.896-0.224l16.16-8.064q0.48-0.256 0.8-0.736t0.288-1.088-0.288-1.056-0.8-0.736l-16.16-8.064q-0.448-0.224-0.896-0.224-0.544 0-1.088 0.288-0.928 0.608-0.928 1.728v16.16z"></path> 
                        </svg>
                    </button>

                    <button onClick={handleStop} disabled={!isRunning} class={button_class}>
                        <svg class="fill-current w-4 h-4 mr-2" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 25 25">
                            <path d="M6 3C4.89543 3 4 3.89543 4 5V19C4 20.1046 4.89543 21 6 21H9C10.1046 21 11 20.1046 11 19V5C11 3.89543 10.1046 3 9 3H6Z" fill="#000000"></path>
                            <path d="M15 3C13.8954 3 13 3.89543 13 5V19C13 20.1046 13.8954 21 15 21H18C19.1046 21 20 20.1046 20 19V5C20 3.89543 19.1046 3 18 3H15Z" fill="#000000"></path>
                        </svg>
                    </button>
                    
                    <button onClick={handleReset} disabled={timeRemaining === 0} class={button_class}>
                        <svg class="fill-current w-4 h-4 mr-2" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 25 25">
                            <path d="M16.3788 6.20698C15.1885 5.25459 13.7434 4.5 12 4.5C7.85787 4.5 4.5 7.85786 4.5 12C4.5 16.1421 7.85787 19.5 12 19.5C15.2549 19.5 18.028 17.4254 19.0646 14.5256C19.2505 14.0055 19.775 13.6568 20.3153 13.7713L21.2935 13.9787C21.8338 14.0932 22.1836 14.6262 22.0179 15.1531C20.6787 19.4112 16.7016 22.5 12 22.5C6.20101 22.5 1.5 17.799 1.5 12C1.5 6.20101 6.20101 1.5 12 1.5C14.7835 1.5 16.9516 2.76847 18.5112 4.0746L20.2929 2.29289C20.5789 2.00689 21.009 1.92134 21.3827 2.07612C21.7564 2.2309 22 2.59554 22 3V8.5C22 9.05228 21.5523 9.5 21 9.5H15.5C15.0956 9.5 14.7309 9.25636 14.5761 8.88268C14.4214 8.50901 14.5069 8.07889 14.7929 7.79289L16.3788 6.20698Z" fill="#000000"></path>
                        </svg>
                    </button>

                    <button class={button_class} onClick={handleAddTime}>
                        <svg class="fill-current w-4 h-4 mr-2" xmlns="http://www.w3.org/2000/svg" viewBox="2 2 22 22">
                            <path id="Vector" d="M6 12H12M12 12H18M12 12V18M12 12V6" stroke="#000000" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"></path> 
                        </svg>
                    </button>
                </div>
            </div>
            {isRunning? <AudioPlayer playAudio={true} fileName={audioFile}/>:<></>}
        </div>
    );
}

export default CountdownTimer;
