import Mouth from "./default/mouth";
import Brow from "./default/brow";
import Eye from './default/eyes';

import MouthCodial from "./cordial/mouth";
import BrowCordial from "./cordial/brow";
import EyeCordial from './cordial/eyes';

import MouthQT from "./qt/mouth";
import BrowQT from "./qt/brow";
import EyeQT from './qt/eyes';
import { useEffect } from "react";

function HeadDisplay({face, position, eyeAU, browAU, mouthAU}){
    position.faceColor = "#CCC" // "#D7E4F5"
    position.right_eye.faceColor = position.faceColor
    position.left_eye.faceColor = position.faceColor
    let width = 102
    let height = 59
    let x_0 = -50.9
    let y_0 = -30.5
    let viewBox = x_0 + " " + y_0  + " " +  width  + " " + height

    useEffect (() => {
    },[eyeAU, browAU, mouthAU]
    )

    return (
      <div>
          {face==="default" ?
          <svg viewBox={viewBox} xmlns="http://www.w3.org/2000/svg">
            <rect id="face" x="-50" y="-30" width="100" height="59" fill={position.faceColor} stroke="black" strokeWidth="2"/>
            <Eye id="right" position={position.right_eye} eyeAU={eyeAU}/>
            <Eye id="left" position={position.left_eye} eyeAU={eyeAU}/> 
            <Brow id="leftbrow" position={position.left_brow} browAU={browAU}/>
            <Brow id="rightbrow" position={position.right_brow} browAU={browAU}/>
            <Mouth position={position.mouth} mouthAU={mouthAU} />
          </svg>:<></>
          }
          {face==="qt" ?
          <svg viewBox={viewBox} xmlns="http://www.w3.org/2000/svg">
            <rect id="face" x="-50" y="-30" width="100" height="59" fill={position.faceColor} stroke="black" strokeWidth="5"/>
            <EyeQT id="right" position={position.right_eye} eyeAU={eyeAU}/>
            <EyeQT id="left" position={position.left_eye} eyeAU={eyeAU}/> 
            <BrowQT id="leftbrow" position={position.left_brow} browAU={browAU}/>
            <BrowQT id="rightbrow" position={position.right_brow} browAU={browAU}/>
            <MouthQT position={position.mouth} mouthAU={mouthAU} />
          </svg>:<></>
          }
          {face==="cordial" ?
          <svg viewBox={viewBox} xmlns="http://www.w3.org/2000/svg">
            <rect id="face" x="-50" y="-30" width="100" height="59" fill={position.faceColor} stroke="#EEE" strokeWidth="2"/>
            <EyeCordial id="right" position={position.right_eye} eyeAU={eyeAU}/>
            <EyeCordial id="left" position={position.left_eye} eyeAU={eyeAU}/> 
            <BrowCordial id="leftbrow" position={position.left_brow} browAU={browAU}/>
            <BrowCordial id="rightbrow" position={position.right_brow} browAU={browAU}/>
            <MouthCodial position={position.mouth} mouthAU={mouthAU} />
          </svg>:<></>
        }
        {face==="qt_head" ?
          <svg viewBox={x_0+" "+-50+" "+width+" "+110} xmlns="http://www.w3.org/2000/svg">
            <rect id="neck" x="-10" y="39" rx="15" ry="15" width="20" height="20" fill="blue" stroke="grey" strokeWidth=".5"/>
            <rect id="frame" x="-50" y="-49" width={width} height="95" rx="25" ry="15" fill="#FFF" stroke="blue" strokeWidth=".5"/>
            <rect id="face" x="-48" y="-30" width="97" height="59" fill={position.faceColor} stroke="#000" strokeWidth="1.5"/>
            <rect id="camerabar" x="-25" y="-43" rx="5" ry="5" width="50" height="7" fill="#000" stroke="#FFF" strokeWidth=".5"/>
            <rect id="cameral" x="-15" y="-42" rx="5" ry="5" width="5" height="5" fill="grey" stroke="#111" strokeWidth=".5"/>
            <rect id="camerar" x="-5" y="-42" rx="5" ry="5" width="5" height="5" fill="grey" stroke="#111" strokeWidth=".5"/>
            <EyeQT id="right" position={position.right_eye} eyeAU={eyeAU}/>
            <EyeQT id="left" position={position.left_eye} eyeAU={eyeAU}/> 
            <BrowQT id="leftbrow" position={position.left_brow} browAU={browAU}/>
            <BrowQT id="rightbrow" position={position.right_brow} browAU={browAU}/>
            <MouthQT position={position.mouth} mouthAU={mouthAU} />
          </svg>:<></>
          }
      </div> 
    )
}

export default HeadDisplay