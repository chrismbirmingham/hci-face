
// text -> X-SAMPA -> Viseme -> FACS
export default function getExpresionAUs(expression){

    const MouthAU = {}
    const EyeAU = {}
    const BrowAU = {}
    switch(expression) {
        case "neutral":
            BrowAU.au1 =  0
            BrowAU.au2 =  0
            BrowAU.au4 =  0
            EyeAU.au5 =  0
            EyeAU.au6 =  0
            EyeAU.au7 =  1
            MouthAU.au10 = 0
            MouthAU.au12 = 0
            MouthAU.au13 = 0
            MouthAU.au14 = 0
            MouthAU.au15 = 0
            MouthAU.au16 = 0.1
            MouthAU.au17 = 0
            MouthAU.au18 = 0
            MouthAU.au20 = 0
            MouthAU.au22 = 0
            MouthAU.au23 = 0
            MouthAU.au24 = 0
            MouthAU.au25 = 0
            MouthAU.au26 = 0.2
            MouthAU.au27 = 0
            MouthAU.au28 = 0
            EyeAU.au41 =  0
            EyeAU.au42 =  0
            EyeAU.au43 =  0
            EyeAU.au44 =  0
            EyeAU.au45 =  0
            EyeAU.au61 =  0
            EyeAU.au62 =  0
            EyeAU.au63 =  0
            EyeAU.au64 =  0
        break;

        case "happy": 
        EyeAU.au6 = 1
        MouthAU.au12 = 1
        MouthAU.au26 = 1
        break;

        case "sad": 
        BrowAU.au1 = 1
        BrowAU.au4 = 1
        EyeAU.au41 = .9
        EyeAU.au62 = .2
        EyeAU.au64 = .2
        MouthAU.au15 = 1.4
        MouthAU.au23 = .5
        break;
        
        case "surprised": 
        BrowAU.au1 = 1
        BrowAU.au2 = 1
        MouthAU.au5 = .4
        MouthAU.au15 = 1
        MouthAU.au26 = 2
        break;

        case "fearful": 
        BrowAU.au1 = 1
        BrowAU.au2 = 1
        BrowAU.au4 = 1
        MouthAU.au5 = 1
        MouthAU.au7 = 1
        MouthAU.au15 = 1
        MouthAU.au16 = 1
        MouthAU.au20 = 1
        MouthAU.au26 = 2
        break;

        case "angry": 
        BrowAU.au4 = 1
        MouthAU.au5 = 1
        MouthAU.au7 = 1
        MouthAU.au15 = 1
        MouthAU.au23 = 1
        break;

        case "disgusted": 
        MouthAU.au9 = 1
        MouthAU.au15 = 1
        MouthAU.au17 = 1
        break;

        default:
        break
    }
    return [MouthAU, EyeAU, BrowAU]
}
