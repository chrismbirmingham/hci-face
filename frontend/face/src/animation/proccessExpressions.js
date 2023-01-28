
// text -> X-SAMPA -> Viseme -> FACS
export default function getExpresionAUs(expression, intensity = .5){

    const MouthAU = {
        au10: 0,
        au12: 0,
        au13: 0,
        au14: 0,
        au15: 0,
        au16: 0,
        au17: 0,
        au18: 0,
        au20: 0,
        au22: 0,
        au23: 0,
        au24: 0,
        au25: 0,
        au26: 0,
        au27: 0,
        au28: 0
    };
    const BrowAU = {
        au1: 0,
        au2: 0,
        au4: 0,
    };  
    const EyeAU = {
        au5: 0,
        au6: 0,
        au7: 0,
        au41: 0,
        au42: 0,
        au43: 0,
        au44: 0,
        au45: 0,
        au61: 0,
        au62: 0,
        au63: 0,
        au64: 0,
    };  
    var d = 1;
    if (expression.includes("-")){
        d = 1.3
    }

    if (expression.includes("neutral")){
        MouthAU.au12 = MouthAU.au12 + (0.2/d) * intensity
        MouthAU.au26 = MouthAU.au26 + (0.2/d) * intensity
        EyeAU.au44 = EyeAU.au44 + ( 0.6/d) * intensity
    };

    if (expression.includes("joy") || expression.includes("happy")){ 
        EyeAU.au6 = EyeAU.au6 + (0.25/d) * intensity
        EyeAU.au44 = EyeAU.au44 + (0.6/d) * intensity
        BrowAU.au1 = BrowAU.au1 + (0.15/d) * intensity
        MouthAU.au10 = MouthAU.au10 + (0.5/d) * intensity
        MouthAU.au12 = MouthAU.au12 + (1/d) * intensity
        MouthAU.au14 = MouthAU.au14 + (1/d) * intensity
        MouthAU.au16 = MouthAU.au16 + (0.3/d) * intensity
        MouthAU.au27 = MouthAU.au28 + (0.27/d) * intensity
        MouthAU.au28 = MouthAU.au28 + (0.27/d) * intensity
    };

    if (expression.includes("sad")){ 
        BrowAU.au1 = BrowAU.au1 + (.5/d) * intensity
        BrowAU.au2 = BrowAU.au2 - (1/d) * intensity
        EyeAU.au41 = EyeAU.au41 + (.5/d) * intensity
        EyeAU.au44 = EyeAU.au44 + (.9/d) * intensity
        EyeAU.au45 = EyeAU.au45 + (.3/d) * intensity
        EyeAU.au64 = EyeAU.au64 + (1/d) * intensity
        MouthAU.au12 = MouthAU.au12 + (.2/d) * intensity
        MouthAU.au14 = MouthAU.au14 + (.24/d) * intensity
        MouthAU.au15 = MouthAU.au15 + (1.5/d) * intensity
        MouthAU.au16 = MouthAU.au16 + (.2/d) * intensity
        MouthAU.au25 = MouthAU.au25 + (.15/d) * intensity
        MouthAU.au26 = MouthAU.au26 + (.27/d) * intensity
        MouthAU.au28 = MouthAU.au28 + (.33/d) * intensity
    };
        
    if (expression.includes("surprise")){ 
        BrowAU.au1 = BrowAU.au1 + (1.3/d) * intensity
        BrowAU.au2 = BrowAU.au2 + (1/d) * intensity
        MouthAU.au10 = MouthAU.au10 + (.54/d) * intensity
        MouthAU.au12 = MouthAU.au12 + (.2/d) * intensity
        MouthAU.au26 = MouthAU.au26 + (1/d) * intensity
    };

    if (expression.includes("fear")){ 
        BrowAU.au1 = BrowAU.au1 + (1/d) * intensity
        BrowAU.au2 = BrowAU.au2 + (.5/d) * intensity
        BrowAU.au4 = BrowAU.au4 + (.5/d) * intensity
        EyeAU.au5 = EyeAU.au5 + (.36/d) * intensity
        EyeAU.au6 = EyeAU.au6 + (1/d) * intensity
        MouthAU.au20 = MouthAU.au20 + (1/d) * intensity
        MouthAU.au22 = MouthAU.au22 + (.21/d) * intensity
        MouthAU.au25 = MouthAU.au25 + (.4/d) * intensity
        MouthAU.au26 = MouthAU.au26 + (1/d) * intensity
    };

    if (expression.includes("anger")){ 
        BrowAU.au2 = BrowAU.au2 + (1/d) * intensity
        BrowAU.au4 = BrowAU.au4 + (1/d) * intensity
        EyeAU.au5 = EyeAU.au5 + (.36/d) * intensity
        EyeAU.au7 = EyeAU.au7 + (.57/d) * intensity
        EyeAU.au42 = EyeAU.au42 + (1/d) * intensity
        MouthAU.au10 = MouthAU.au10 + (.24/d) * intensity
        MouthAU.au12 = MouthAU.au12 + (.2/d) * intensity
        MouthAU.au15 = MouthAU.au15 + (1/d) * intensity
        MouthAU.au18 = MouthAU.au18 + (.2/d) * intensity
        MouthAU.au24 = MouthAU.au24 + (.6/d) * intensity
        MouthAU.au26 = MouthAU.au26 + (1/d) * intensity
    };

    if (expression.includes("disgust")){ 
        EyeAU.au43 = EyeAU.au43 + (1/d) * intensity
        BrowAU.au4 = BrowAU.au4 + (1/d) * intensity
        MouthAU.au12 = MouthAU.au12 + (.2/d) * intensity
        MouthAU.au13 = MouthAU.au13 + (.9/d) * intensity
        MouthAU.au15 = MouthAU.au15 + (1/d) * intensity
        MouthAU.au20 = MouthAU.au20 + (.4/d) * intensity
        MouthAU.au24 = MouthAU.au24 + (1/d) * intensity
    };
    return [MouthAU, EyeAU, BrowAU]
}
