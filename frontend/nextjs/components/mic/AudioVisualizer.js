import React, {useEffect, useRef} from 'react';

export default function AudioVisualizer ({audio, rmsThresh, bufferSize}) {
  const audioContext = new window.AudioContext()
  const analyser = new AnalyserNode(audioContext);
  const source = audioContext.createMediaStreamSource(audio);
  const canvasRef = useRef(null)
  const refID = useRef()
  const hearingRef = useRef()
  const bufferLength = analyser.fftSize;
  const dataArray = new Uint8Array(bufferLength)
  const rmshistory = []
  var rmsOverThresh = false
  hearingRef.current = false


  const draw = () => {
    source.connect(analyser)
    analyser.getByteTimeDomainData(dataArray);

    const canvas = canvasRef.current;
    const height = canvas.height-10;
    const width = canvas.width;
    const sliceWidth = (width * 1.0) / bufferLength;
    
    const context = canvas.getContext('2d');
    context.clearRect(0, 0, width, canvas.height);

    context.lineWidth = 2;
    context.strokeStyle = '#000';
    
    context.beginPath();
    context.moveTo(0, height);

    let x = 0;
    const dataCopy = []
    for(let i = 0; i < bufferLength; i++) {
      let v = (dataArray[i]-128)/256.0
      const y = (height) - (v)*height;
      dataCopy.push(v)
      context.lineTo(x, y);
      x += sliceWidth;
    }

    let Squares = dataCopy.map((val) => (val*val));
    let Sum = Squares.reduce((acum, val) => (acum + val));
    let Mean = Sum/dataArray.length;
    let rms = Math.sqrt(Mean)
    rmshistory.push(rms)
    if (rmshistory.length>bufferSize) {rmshistory.shift()}
    let rmsmean = rmshistory.reduce((acum, val) => (acum + val))/rmshistory.length

    // context.moveTo(width, (height) - (rmsThresh) * height*5);
    // context.lineTo(0, (height) - (rmsThresh) * height*5);

    // let thresholdSurpassed = false
    if (rmsmean>rmsThresh) {
      context.strokeStyle = 'green';
      // thresholdSurpassed = true
    }
    if (rms>rmsThresh) {
      context.strokeStyle = 'pink';
      // thresholdSurpassed = true
    }

    context.stroke();  
    refID.current = requestAnimationFrame(draw);

  }

  useEffect(() => {
    refID.current = requestAnimationFrame(draw);
    return () => {
      cancelAnimationFrame(refID.current);
    };// eslint-disable-next-line
  },[] );


  return (
    <div>
      <canvas width="300" height="50" ref={canvasRef} />
    </div>
    )
  }