import React from "react"
export default function doBehavior(count, behavior, browUpdater, mouthUpdater, getExpresionAUs, eyeUpdaterWrapper) {
  switch (behavior)
  {
      case "bored":
          count = boredEyes(count, 10, eyeUpdaterWrapper, mouthUpdater)
          break;
      case "focused":
          count = focusedEyes(count, 20, eyeUpdaterWrapper, mouthUpdater)
          break;
      case "random":
          count = randomFace(count, 6, eyeUpdaterWrapper, browUpdater, mouthUpdater, getExpresionAUs)
          break;
      default:
          break;
  }
  return count
}

function lookRandom(focus) {
    const eyes = { // focus of 1 move 0, focus of zero move 1
      au61_left: (1-focus) * Math.random(),
      au62_right: (1-focus) * Math.random(),
      au63_up: (1-focus) * Math.random(),
      au64_down: (1-focus) * Math.random(),
    }
    return eyes
    
  }

function boredEyes(count, reset_count, eyeUpdater, mouthUpdater){
    count = count + 1
    if (count % 4 === 0){
        let eyes = lookRandom(.5)
        eyeUpdater(eyes)
      }

      // Purse lips
      if (count===reset_count-4){
        mouthUpdater({au24_lip_pressor:.75})
      }
      if (count===reset_count-1){
        mouthUpdater({au24_lip_pressor:0})
      }

      // close eyes
      if (count===reset_count){
        eyeUpdater({au45_blink:.85})
      }
      // open eyes
      if (count>reset_count){
        eyeUpdater({au45_blink:0})
        count = 0
      }
      return count
}

function focusedEyes(count, reset_count, eyeUpdater, mouthUpdater){
    count = count + 1
    if (count % 4 === 0){
        let eyes = lookRandom(.85)
        eyeUpdater(eyes)
      }

      // Purse lips
      if (count===reset_count-4){
        mouthUpdater({au28_lip_suck:.33})
      }
      if (count===reset_count-1){
        mouthUpdater({au28_lip_suck:0})
      }

      // close eyes
      if (count===reset_count){
        eyeUpdater({au45_blink:.85})
      }
      // open eyes
      if (count>reset_count){
        eyeUpdater({au45_blink:0})
        count = 0
      }
      return count
}

function randomFace(count, reset_count, eyeUpdater, browUpdater, mouthUpdater, getExpresionAUs){
    let faces = ["anger", "joy", "sad", "fear", "disgust", "surprise", "neutral"]
    // console.log("random")
    count = count + 1
    var expression
    if (count>reset_count){
        count = 0
        const randomElement1 = faces[Math.floor(Math.random() * faces.length)];
        const randomElement2 = faces[Math.floor(Math.random() * faces.length)];
        expression = randomElement1 + "-" + randomElement2
        var [MouthAU, EyeAU, BrowAU] = getExpresionAUs(expression)
        mouthUpdater(MouthAU)
        browUpdater(BrowAU)
        eyeUpdater(EyeAU)
    }
    return count
}