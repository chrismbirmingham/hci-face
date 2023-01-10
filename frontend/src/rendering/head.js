import Mouth from "./mouth";
import Brow from "./brow";
import Eye from './eyes';

function Head({position, eyeAU, browAU, mouthAU}){
    return (
        <div id="head">
            <svg
              viewBox="-50 -50 100 100"
              xmlns="http://www.w3.org/2000/svg"
              >
              <defs>
                <radialGradient id="sampleGradient">
                  <stop offset="85%" stopColor="#D7E4F5" />
                  <stop offset="100%" stopColor="#a0c9ff" />
                </radialGradient>
                {/* fill="url(#sampleGradient)" */}
              </defs>
              <rect id="neck" x="-10" y="39" rx="15" ry="15" width="20" height="20" fill="blue" stroke="grey" strokeWidth=".5"/>
              <rect x="-50" y="-50" rx="35" ry="15" width="100" height="95" fill="#FFF" stroke="#FFF" strokeWidth=".5"/>
              <rect id="face" x="-45" y="-30" width="90" height="60" fill="#D7E4F5" stroke="#EEE" strokeWidth="2"/>
              <rect id="camera" x="-25" y="-43" rx="5" ry="5" width="50" height="7" fill="#000" stroke="#FFF" strokeWidth=".5"/>
              <rect id="camera2" x="-15" y="-42" rx="5" ry="5" width="5" height="5" fill="grey" stroke="#111" strokeWidth=".5"/>
              <rect id="camera2" x="-5" y="-42" rx="5" ry="5" width="5" height="5" fill="grey" stroke="#111" strokeWidth=".5"/>
              <Eye id="right" position={position.right_eye} eyeAU={eyeAU}/>
              <Eye id="left" position={position.left_eye} eyeAU={eyeAU}/> 
              <Brow id="leftbrow" position={position.left_brow} browAU={browAU}/>
              <Brow id="rightbrow" position={position.right_brow} browAU={browAU}/>
              <Mouth position={position.mouth} mouthAU={mouthAU} />
            </svg>
          </div> 
    )
}

export default Head