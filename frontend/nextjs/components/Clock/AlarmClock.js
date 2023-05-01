import React, { useState, useEffect } from 'react';

function AlarmClock() {
  const [time, setTime] = useState(new Date());
  const [displayTime, setDisplayTime] = useState("");
  const [alarms, setAlarms] = useState([]);
  const [alarm, setAlarm] = useState('');
  const [alarmName, setAlarmName] = useState("Example");
  const [alarmStr, setAlarmStr] = useState('');

  useEffect(() => {
    const intervalID = setInterval(() => {
      setTime(new Date());
    }, 1000);

    return () => {
      clearInterval(intervalID);
    };
  }, []);

  useEffect(() => {
    let curtime = time.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
    alarms.forEach((item, index) => {
        console.log(curtime, item)
        if (item.time == curtime) {
            alert("Alarm: "+ item.name+ " is ringing.")
            handleAlarmCancel(index)
        }
    })
  }, [time, alarms]);

  useEffect(() => {
    setDisplayTime(time.toLocaleTimeString());
  }, [time]);

  const handleAlarmChange = (event) => {
    let t = event.target.value;
    setAlarm(t);
    let hours = t.slice(0,2)
    console.log(hours)
    let s = " PM";
    if (['10','11','12'].includes(hours) || t[0]==='0'){
        console.log(hours)
      s = " AM";
    }
    setAlarmStr(t+s);
  };

  const handleAlarmSet = () => {
    console.log("alarm set for", alarm);
    setAlarms([...alarms, {time: alarmStr, name: alarmName}]);
  };

  const handleAlarmCancel = (index) => {
    const newAlarms = [...alarms];
    newAlarms.splice(index, 1);
    setAlarms(newAlarms);
  };

  return (
<div className="max-w-sm p-6 bg-white border border-gray-200 rounded-lg shadow dark:bg-gray-800 dark:border-gray-700">
        <h className="mb-2 text-2xl font-bold tracking-tight text-gray-900 dark:text-white">Alarm Clock</h>
      <p>Current Time: {displayTime}</p>
      <div className="space-x-2">
      <label>Set Alarm For:</label>
      <input className="border border-gray-200" type="time" id="alarm" value={alarm} onChange={handleAlarmChange} />
      </div>
      <div className="space-x-2">
        <label>Name Alarm:</label>
        <input className="border border-gray-200 w-24" type="text" id="alarmName" value={alarmName} onChange={(event) => setAlarmName(event.target.value)} />
        <button onClick={handleAlarmSet}>Set Alarm</button>
      </div>
      <ul>
        {alarms.map((alarm, index) => (
          <li key={index}>{alarm.time} - {alarm.name} <button onClick={() => handleAlarmCancel(index)}>Cancel</button></li>
        ))}
      </ul>
    </div>
  );
}

export default AlarmClock;
