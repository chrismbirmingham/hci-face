import React, {useState, useEffect} from "react";

export default function AudioPlayer( {playAudio, fileName="/test.m4a"}) {
    const audioRef = React.useRef(null);
    const [isPlaying, setIsPlaying] = useState(false)
  
    const handlePlay = () => {
      if (audioRef.current) {
        audioRef.current.play();
        setIsPlaying(true)
      }
    };
  
    const handlePause = () => {
      if (audioRef.current) {
        audioRef.current.pause();
        setIsPlaying(false)
      }
    };

    useEffect(()=>{
        if (playAudio) {
            handlePlay()
        }
        else {
            handlePause()
            audioRef.is
        }
    }, [playAudio])
  
    return (
      <div>
        <audio ref={audioRef} loop>
          <source src={fileName} type="audio/mp4" />
        </audio>
        {!isPlaying && <button onClick={handlePlay}>Play</button>}
        {isPlaying && <button onClick={handlePause}>Pause</button>}
      </div>
    );
  }
