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
              </defs>
              {/* <rect x="-50" y="-50" rx="25" ry="35" width="99" height="99" fill="url(#sampleGradient)" stroke="#FFF" strokeWidth=".5"/> */}
              <rect x="-50" y="-50" width="99" height="99" fill="#D7E4F5" stroke="#FFF" strokeWidth=".5"/>
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