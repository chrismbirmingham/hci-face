import React, { useState, useRef, useEffect, useCallback } from "react";


export default function Timer ({timerDeadline}) {
	const [date, setDate] = useState("")
    const Ref = useRef(null);
	const [color, setColor] = useState('Green');
	const [timer, setTimer] = useState('00:00:00');

    const getTimeRemaining = (e) => {
        const total = Date.parse(e) - Date.parse(new Date());
        const seconds = Math.floor((total / 1000) % 60);
        const minutes = Math.floor((total / 1000 / 60) % 60);
        const hours = Math.floor((total / 1000 / 60 / 60) % 24);
        setDate(Date())
        return {
            total, hours, minutes, seconds
        };
    }

    const updateTimerDisplay = (e) => {
        let { total, hours, minutes, seconds } = getTimeRemaining(e);
        if (total >= 0) {
            setTimer(
                (hours > 9 ? hours : '0' + hours) + ':' +
                (minutes > 9 ? minutes : '0' + minutes) + ':'
                + (seconds > 9 ? seconds : '0' + seconds)
            )
        }
        if (minutes >= 3){setColor("green")}
        if (minutes < 3 && minutes >= 1){
    //   setWalkthroughToggle(false)
        setColor("orange")
    }
        if (minutes < 1 ){setColor("red")}
    }


    const runTimer = useCallback(e => {
        console.log("run timer")
        if (Ref.current) clearInterval(Ref.current);
        const id = setInterval(() => {
            // console.log("Update Timer Display", playTimerVar)
            updateTimerDisplay(e);
        }, 1000)
        Ref.current = id;
    },[])

    useEffect(() => {
        runTimer(timerDeadline)
    },[timerDeadline, runTimer])


    return(
        <div>
        <h3 style={{backgroundColor:color}}>Interactive Controls:</h3>
        <p style={{backgroundColor:color}}>Time Remaining: {timer}</p>
        </div>
    )
}