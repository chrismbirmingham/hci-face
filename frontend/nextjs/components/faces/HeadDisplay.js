
import { useEffect } from "react";
import EyeDrawing from "./eyeDrawing"
import BrowDrawing from "./browDrawing"
import MouthDrawing from "./mouthDrawing"
import BackgroundDrawing from "./backgroundDrawing";
import StyleHead from "./styleHead";

function HeadDisplay({face, head_settings, eyeAU, browAU, mouthAU}){
    const updated_settings = StyleHead({face, head_settings})
    const {positions, head_shape, eye_shape, brow_shape, mouth_shape, eye_settings, brow_settings, mouth_settings, background_settings} = updated_settings;

    let viewBox = Object.values(background_settings).join(" ")


    useEffect (() => {
    },[eyeAU, browAU, mouthAU]
    )

    return (
      <svg viewBox={viewBox} xmlns="http://www.w3.org/2000/svg">
        <BackgroundDrawing position={positions} shape={head_shape} settings={background_settings}/>
        <EyeDrawing position={positions.right_eye} AU={eyeAU} shape={eye_shape} settings={eye_settings}/>
        <EyeDrawing position={positions.left_eye} AU={eyeAU} shape={eye_shape} settings={eye_settings}/>
        <BrowDrawing position={positions.right_brow} AU={browAU} shape={brow_shape} settings={brow_settings}/>
        <BrowDrawing position={positions.left_brow} AU={browAU} shape={brow_shape} settings={brow_settings}/>
        <MouthDrawing position={positions.mouth} AU={mouthAU} shape={mouth_shape} settings={mouth_settings}/>
      </svg>
    )
}

export default HeadDisplay