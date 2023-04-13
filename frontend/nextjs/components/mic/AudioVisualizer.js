import React, { useRef, useEffect, useState } from 'react';

function custom_rms(data) {
  let Squares = data.map((val) => (val*val));
  let Sum = Squares.reduce((acum, val) => (acum + val));
  let Mean = Sum/data.length;
  let rms = Math.sqrt(Mean)
  let rmsint = Math.round(rms*100)
  rmsint = Math.min(rmsint, 255)
  return rmsint
}


export default function SoundWaveform() {
  const canvasRef = useRef(null);
  const animationIdRef = useRef(null);
  const [threshold, setThreshold] = useState(100)
  const [rmswindow, setRmswindow] = useState(15)

  useEffect(() => {
    const canvas = canvasRef.current;
    const canvasContext = canvas.getContext('2d');

    navigator.mediaDevices.getUserMedia({ audio: true })
      .then((stream) => {
        const audioContext = new AudioContext();
        const analyser = audioContext.createAnalyser();


        const source = audioContext.createMediaStreamSource(stream);
        source.connect(analyser);

        analyser.fftSize = 2048*16;
        const bufferLength = analyser.fftSize;
        const dataArray = new Uint8Array(bufferLength);


        const draw = () => {
          animationIdRef.current = requestAnimationFrame(draw);

          analyser.getByteTimeDomainData(dataArray);


          const rmsarray = []
          for (let i=(16-rmswindow); i<16; i++) {
            let rmsint = custom_rms(dataArray.slice( 2048*i, 2048*(i+1)))
            rmsarray.push(rmsint)
          }
          // console.log(rmsarray)
          let maxrms = Math.max(...rmsarray)
          // console.log(maxrms)

          canvasContext.fillStyle = 'rgb(255, 255, 255)';
          canvasContext.fillRect(0, 0, canvas.width, canvas.height);

          canvasContext.lineWidth = 2;
          if (maxrms>threshold) {
            canvasContext.strokeStyle = 'rgb(255, 0, 0)';
          }
          else {
            canvasContext.strokeStyle = 'rgb(0, 0, 0)';
          }
          canvasContext.beginPath();

          const sliceWidth = canvas.width * 1.0 / bufferLength;
          let x = 0;
          for (let i = 0; i < bufferLength; i++) {
            const v = dataArray[i] / 128.0;
            const y = v * canvas.height / 2;

            if (i === 0) {
              canvasContext.moveTo(x, y);
            } else {
              canvasContext.lineTo(x, y);
            }

            x += sliceWidth;
          }

          canvasContext.stroke();
        };

        draw();
      })
      .catch((error) => {
        console.error(error);
      });

    return () => {
      cancelAnimationFrame(animationIdRef.current);
    };
  }, [threshold, rmswindow]);

  return (
    <div>
      <h1>Sound Waveform</h1>
      <p>Color is red when sound is detected</p>
      <canvas ref={canvasRef} />
      <br />Thresh = {threshold}
      <input type="range" min="0" max="255" value={threshold} title='Thresh' onChange={(e) => setThreshold(e.target.value)} />
      <br />RMS Window = {rmswindow}
      <input type="range" min="1" max="16" value={rmswindow} onChange={(e) => setRmswindow(e.target.value)} />
      <br />
      <br />
    </div>
  )
}


