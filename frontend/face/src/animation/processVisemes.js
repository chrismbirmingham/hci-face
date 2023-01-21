
// text -> X-SAMPA -> Viseme -> FACS
export default function getAUs(vizeme, intensity = .5){
    const au = {
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
    switch(vizeme) {
        case "IDLE":
        break;

        case "M_B_P": //au 23, 24?, 14?,
        au.au23= .75 *intensity
        au.au14= .25 *intensity

        break;

        case "AA_AH": //au 25, 26, 14
        au.au26= 1 *intensity
        au.au25= .5 *intensity
        au.au14= .5 *intensity

        break;

        case "AO_AW": //au 25, 26, 27
        au.au26= .75 *intensity
        au.au27= 1 *intensity


        break;

        case "EH_AE_AY": //au 25, 26, 14
        au.au14= .75 *intensity
        au.au26= .75 *intensity


        break;

        case "CH_SH_ZH": //au 18, 25, 10
        au.au10= .75 *intensity
        au.au18= 1 *intensity
        break;

        case "N_NG_D_Z": //au 10,
        au.au10=.6 *intensity
        au.au18=.5 *intensity


        break;

        case "R_ER": //au 10
        au.au10=1 *intensity
        au.au18=.7 *intensity


        break;

        case "EY": //au 25, 26, 14
        au.au26=1 *intensity
        break;

        case "L": //au 25
        au.au10=.5 *intensity
        au.au18=.5 *intensity


        break;

        // "you" "too" "moo"
        case "OO": //au 10, 25,
        au.au10=1 *intensity
        au.au25=1 *intensity
        au.au18=1 *intensity


        break;

        // --------------- CONSONANTS ---------------------//

        // M,B,P -> My, Buy, Pie
        case 'BILABIAL':
        au.au23= .75 *intensity
        au.au14= .25 *intensity
        au.au24= .7 *intensity

        break;

        // F,V -> oFFer, Vest
        case "LABIODENTAL":
        au.au10=0.5 *intensity
        au.au20=0.4 *intensity
        au.au25=.8 *intensity

        break;

        // TH, TH - THin, THis
        case "INTERDENTAL":
        au.au10=.6 *intensity
        au.au18=.75 *intensity
        au.au25=.5 *intensity

        break;

        // L,T,D,Z,S,N -> Light, Top, DaD, Zebra, Sad, Nope
        case "DENTAL_ALVEOLAR":
        au.au25=.65 *intensity

        break;

        // R,SH,ZH,CH -> Red, SHould, aSia, CHart
        case "POSTALVEOLAR":
        au.au10= .75 *intensity
        au.au18= 1 *intensity
        au.au25= 1 *intensity

        break;

        // K,G,NG -> Cat, Game, thiNG
        case "VELAR_GLOTTAL":
            au.au10=.6 *intensity
            // au.au18=.5 *intensity
            au.au26=.5 *intensity

            break;

        // ------------------ VOWELS ------------------------//
        // EE, I -> flEEce, bIt
        case "CLOSE_FRONT_VOWEL":
            au.au26=1 *intensity
            au.au20=1 *intensity
            au.au10=.4 *intensity
            break;

        // OO -> bOOt
        case "CLOSE_BACK_VOWEL":
            au.au10=.5 *intensity
            au.au13=.8 *intensity
            au.au16=.6 *intensity
            au.au18=1 *intensity
            au.au23=1 *intensity
            au.au24=1 *intensity
            au.au25=1 *intensity
            au.au26=.4 *intensity


            break;

        // schwa -> ArenA
        case "MID_CENTRAL_VOWEL":
            au.au26= 1 *intensity
            au.au25= .5 *intensity
            au.au23=1 *intensity

            break;

        // AE,AU,A,AY,EH -> trAp, mOUth, fAther, fAce, drEss
        //ɛ (eh)
        case "OPEN_FRONT_VOWEL":
            au.au14= 1 *intensity
            au.au20= 1 *intensity
            au.au25= .7 *intensity
            au.au26= .75 *intensity

        break;

        // AW,OI,O -> thOUght, chOIce, gOAt
        case "OPEN_BACK_VOWEL":
            au.au26= .5 *intensity
            au.au27= 1 *intensity

        break;


        default:
        break
    }
    return au
}
