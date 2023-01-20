import Mouth from "./default/mouth";
import Brow from "./default/brow";
import Eye from './default/eyes';

import MouthCodial from "./cordial/mouth";
import BrowCordial from "./cordial/brow";
import EyeCordial from './cordial/eyes';


import MouthQT from "./qt/mouth";
import BrowQT from "./qt/brow";
import EyeQT from './qt/eyes';



function Head({face, position, eyeAU, browAU, mouthAU}){
    position.faceColor = "grey" // "#D7E4F5"
    position.right_eye.faceColor = position.faceColor
    position.left_eye.faceColor = position.faceColor
    if (face==="default"){
      return (
        <div id="head">
            <svg
              viewBox="-50 -30 100 59"
              xmlns="http://www.w3.org/2000/svg"
              >
              <defs>
                <radialGradient id="sampleGradient">
                  <stop offset="85%" stopColor="#D7E4F5" />
                  <stop offset="100%" stopColor="#a0c9ff" />
                </radialGradient>{/* fill="url(#sampleGradient)" */}
              </defs>
              <rect id="face" x="-50" y="-30" width="100" height="59" fill={position.faceColor} stroke="black" strokeWidth="2"/>
              <Eye id="right" position={position.right_eye} eyeAU={eyeAU}/>
              <Eye id="left" position={position.left_eye} eyeAU={eyeAU}/> 
              <Brow id="leftbrow" position={position.left_brow} browAU={browAU}/>
              <Brow id="rightbrow" position={position.right_brow} browAU={browAU}/>
              <Mouth position={position.mouth} mouthAU={mouthAU} />
            </svg>
          </div> 
      )
    }
    if (face==="qt"){
      return (
        <div id="head">
            <svg
              viewBox="-50.9 -30.5 101 59"
              xmlns="http://www.w3.org/2000/svg"
              >
              <defs>
                <radialGradient id="sampleGradient">
                  <stop offset="90%" stopColor={position.faceColor} />
                  <stop offset="100%" stopColor="#555" />
                </radialGradient>{/* fill="url(#sampleGradient)" */}
              </defs>
              <rect id="face" x="-50" y="-30" width="100" height="59" fill="url(#sampleGradient)" stroke="black" strokeWidth="5"/>
              <EyeQT id="right" position={position.right_eye} eyeAU={eyeAU}/>
              <EyeQT id="left" position={position.left_eye} eyeAU={eyeAU}/> 
              <BrowQT id="leftbrow" position={position.left_brow} browAU={browAU}/>
              <BrowQT id="rightbrow" position={position.right_brow} browAU={browAU}/>
              <MouthQT position={position.mouth} mouthAU={mouthAU} />
            </svg>
          </div> 
      )
    }
    if (face==="cordial"){
      return (
        <div id="head">
            <svg
              viewBox="-50 -30 100 59"
              xmlns="http://www.w3.org/2000/svg"
              >
              <defs>
                <radialGradient id="sampleGradient">
                  <stop offset="85%" stopColor="#D7E4F5" />
                  <stop offset="100%" stopColor="#a0c9ff" />
                </radialGradient>{/* fill="url(#sampleGradient)" */}
              </defs>
              <rect id="face" x="-50" y="-30" width="100" height="59" fill={position.faceColor} stroke="#EEE" strokeWidth="2"/>
              <EyeCordial id="right" position={position.right_eye} eyeAU={eyeAU}/>
              <EyeCordial id="left" position={position.left_eye} eyeAU={eyeAU}/> 
              <BrowCordial id="leftbrow" position={position.left_brow} browAU={browAU}/>
              <BrowCordial id="rightbrow" position={position.right_brow} browAU={browAU}/>
              <MouthCodial position={position.mouth} mouthAU={mouthAU} />
            </svg>
          </div> 
      )
    }
    // return (
    //     <div id="head">
    //         <svg
    //           viewBox="-50 -30 100 59"
    //           xmlns="http://www.w3.org/2000/svg"
    //           >
    //           <defs>
    //             <radialGradient id="sampleGradient">
    //               <stop offset="85%" stopColor="#D7E4F5" />
    //               <stop offset="100%" stopColor="#a0c9ff" />
    //             </radialGradient>
    //             {/* fill="url(#sampleGradient)" */}
    //           </defs>
    //           {/* <rect id="neck" x="-10" y="39" rx="15" ry="15" width="20" height="20" fill="blue" stroke="grey" strokeWidth=".5"/>
    //           <rect x="-50" y="-50" rx="35" ry="15" width="100" height="95" fill="#FFF" stroke="#FFF" strokeWidth=".5"/>
    //           <rect id="camera" x="-25" y="-43" rx="5" ry="5" width="50" height="7" fill="#000" stroke="#FFF" strokeWidth=".5"/>
    //           <rect id="camera2" x="-15" y="-42" rx="5" ry="5" width="5" height="5" fill="grey" stroke="#111" strokeWidth=".5"/>
    //           <rect id="camera2" x="-5" y="-42" rx="5" ry="5" width="5" height="5" fill="grey" stroke="#111" strokeWidth=".5"/> */}
    //           <rect id="face" x="-50" y="-30" width="100" height="59" fill={faceColor} stroke="#EEE" strokeWidth="2"/>
    //           {/* <rect id="face" x="-50" y="-30" width="100" height="59" fill="#D7E4F5" stroke="#EEE" strokeWidth="2"/> */}
    //           <Eye id="right" position={position.right_eye} eyeAU={eyeAU}/>
    //           <Eye id="left" position={position.left_eye} eyeAU={eyeAU}/> 
    //           <Brow id="leftbrow" position={position.left_brow} browAU={browAU}/>
    //           <Brow id="rightbrow" position={position.right_brow} browAU={browAU}/>
    //           <Mouth position={position.mouth} mouthAU={mouthAU} />
    //         </svg>
    //       </div> 
    // )
}

export default Head