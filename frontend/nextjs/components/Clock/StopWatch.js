import React, { useState, useEffect } from 'react';

function Stopwatch() {
  const [time, setTime] = useState(0);
  const [isRunning, setIsRunning] = useState(false);

  useEffect(() => {
    let intervalId;

    if (isRunning) {
      intervalId = setInterval(() => {
        setTime((prevTime) => prevTime + 1);
      }, 1000);
    }

    return () => clearInterval(intervalId);
  }, [isRunning]);

  function handleStart() {
    setIsRunning(true);
  }

  function handleStop() {
    setIsRunning(false);
  }

  function handleReset() {
    setTime(0);
    setIsRunning(false);
  }

  const hours = Math.floor(time / 3600);
  const minutes = Math.floor((time % 3600) / 60);
  const seconds = time % 60;

  return (
    <div className="max-w-sm p-6 bg-white border border-gray-200 rounded-lg shadow dark:bg-gray-800 dark:border-gray-700">
        <h className="mb-2 text-2xl font-bold tracking-tight text-gray-900 dark:text-white">Basic Stopwatch</h>
        <div>
            {hours.toString().padStart(2, '0')}:{minutes.toString().padStart(2, '0')}:{seconds.toString().padStart(2, '0')}
        </div>
        <div className="space-x-0.5">
            <button onClick={handleStart} disabled={isRunning}>
            Start
            </button>
            <button onClick={handleStop} disabled={!isRunning}>
            Stop
            </button>
            <button onClick={handleReset} disabled={time === 0}>
            Reset
            </button>
        </div>
    </div>
  );
}

export default Stopwatch;
