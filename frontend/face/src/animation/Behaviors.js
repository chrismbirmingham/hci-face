function lookRandom(focus) {
    const eyes = { // focus of 1 move 0, focus of zero move 1
      au61: (1-focus) * Math.random(),
      au62: (1-focus) * Math.random(),
      au63: (1-focus) * Math.random(),
      au64: (1-focus) * Math.random(),
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
        mouthUpdater({au24:.75})
      }
      if (count===reset_count-1){
        mouthUpdater({au24:0})
      }

      // close eyes
      if (count===reset_count){
        eyeUpdater({au45:.65})
      }
      // open eyes
      if (count>reset_count){
        eyeUpdater({au45:0})
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
        mouthUpdater({au28:.33})
      }
      if (count===reset_count-1){
        mouthUpdater({au28:0})
      }

      // close eyes
      if (count===reset_count){
        eyeUpdater({au45:.65})
      }
      // open eyes
      if (count>reset_count){
        eyeUpdater({au45:0})
        count = 0
      }
      return count
}

function randomFace(count, reset_count, eyeUpdater, browUpdater, mouthUpdater, getExpresionAUs){
    let faces = ["anger", "joy", "sad", "fear", "disgust", "surprise", "neutral"]
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

export {randomFace, boredEyes, focusedEyes}