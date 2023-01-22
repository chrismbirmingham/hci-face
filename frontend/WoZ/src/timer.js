import React, { useState, useRef, useEffect } from 'react'


function CountdownTimer() {

	// We need ref in this, because we are dealing
	// with JS setInterval to keep track of it and
	// stop it when needed
	const Ref = useRef(null);

	// The state for our timer
	const [timer, setTimer] = useState('00:00:00');
	const [minuteGoal, setMinuteGoal] = useState('0');
	const [color, setColor] = useState('Green');
	const [date, setDate] = useState("")


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


	const startTimer = (e) => {
		let { total, hours, minutes, seconds }
					= getTimeRemaining(e);
		if (total >= 0) {

			// update the timer
			// check if less than 10 then we need to
			// add '0' at the beginning of the variable
			setTimer(
				(hours > 9 ? hours : '0' + hours) + ':' +
				(minutes > 9 ? minutes : '0' + minutes) + ':'
				+ (seconds > 9 ? seconds : '0' + seconds)
			)
		}
        if (minutes >= 5){setColor("green")}
        if (minutes < 5 && minutes >= 1){setColor("orange")}
        if (minutes < 1 ){setColor("red")}
	}


	const clearTimer = (e) => {

		// If you adjust it you should also need to
		// adjust the Endtime formula we are about
		// to code next	
		setTimer('00:00:00');

		// If you try to remove this line the
		// updating of timer Variable will be
		// after 1000ms or 1sec
		if (Ref.current) clearInterval(Ref.current);
		const id = setInterval(() => {
			startTimer(e);
		}, 1000)
		Ref.current = id;
	}

	const getDeadTime = () => {
		let deadline = new Date();
        let newseconds = minuteGoal*60
		deadline.setSeconds(deadline.getSeconds() + newseconds);
		return deadline;
	}

	// We can use useEffect so that when the component
	// mount the timer will start as soon as possible

	// We put empty array to act as componentDid
	// mount only
	useEffect(() => {
		clearTimer(getDeadTime());// eslint-disable-next-line
	}, []);

	// Another way to call the clearTimer() to start
	// the countdown is via action event from the
	// button first we create function to be called
	// by the button
	const onClickReset = () => {
		clearTimer(getDeadTime());
	}

	return (
		<div className="Timer">
			<p style={{backgroundColor:color}}>Section Timer: {timer}</p>
			<button onClick={onClickReset}>Set Timer To:</button>
            <input type="text" size={3} onChange={(e) => setMinuteGoal(e.target.value)} />
            Minutes --- {date}
		</div>
	)
}

export default CountdownTimer;
