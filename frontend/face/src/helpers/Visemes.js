
// text -> X-SAMPA -> Viseme -> FACS
export default function getAUs(vizeme, intensity = .5){
    const au = {
        au10_raise_upper: 0,
        au12_lip_corners_out: 0,
        au13_cheek_puffer: 0,
        au14_dimpler: 0,
        au15_lip_corner_depr: 0,
        au16_lower_lip_depr: 0,
        au17_chin_raiser: 0,
        au18_lip_pucker: 0,
        au20_lip_stretcher: 0,
        au22_lip_funneler: 0,
        au23_lip_tightener: 0,
        au24_lip_pressor: 0,
        au25_lips_part: 0,
        au26_jaw_drop: 0,
        au27_mouth_stretch: 0,
        au28_lip_suck: 0
    };
    switch(vizeme) {
        case "IDLE":
        break;

        case "M_B_P": //au 23, 24?, 14?,
        au.au23_lip_tightener= .75 * intensity
        au.au14_dimpler= .25 * intensity
        break;

        case "AA_AH": //au 25, 26, 14
        au.au26_jaw_drop= 1 * intensity
        au.au25_lips_part= .5 * intensity
        au.au14_dimpler= .5 * intensity
        break;

        case "AO_AW": //au 25, 26, 27
        au.au26_jaw_drop= .75 * intensity
        au.au27_mouth_stretch= 1 * intensity
        break;

        case "EH_AE_AY": //au 25, 26, 14
        au.au14_dimpler= .75 * intensity
        au.au26_jaw_drop= .75 * intensity
        break;

        case "CH_SH_ZH": //au 18, 25, 10
        au.au10_raise_upper= .75 * intensity
        au.au18_lip_pucker= 1 * intensity
        break;

        case "N_NG_D_Z": //au 10,
        au.au10_raise_upper=.6 * intensity
        au.au18_lip_pucker=.5 * intensity
        break;

        case "R_ER": //au 10
        au.au10_raise_upper=1 * intensity
        au.au18_lip_pucker=.7 * intensity
        break;

        case "EY": //au 25, 26, 14
        au.au26_jaw_drop=1 * intensity
        break;

        case "L": //au 25
        au.au10_raise_upper=.5 * intensity
        au.au18_lip_pucker=.5 * intensity
        break;

        // "you" "too" "moo"
        case "OO": //au 10, 25,
        au.au10_raise_upper=1 * intensity
        au.au25_lips_part=1 * intensity
        au.au18_lip_pucker=1 * intensity
        break;

        // --------------- CONSONANTS ---------------------//

        // M,B,P -> My, Buy, Pie
        case 'BILABIAL':
        au.au23_lip_tightener= .75 * intensity
        au.au14_dimpler= .25 * intensity
        au.au24_lip_pressor= .7 * intensity
        break;

        // F,V -> oFFer, Vest
        case "LABIODENTAL":
        au.au10_raise_upper=0.5 * intensity
        au.au20_lip_stretcher=0.4 * intensity
        au.au25_lips_part=.8 * intensity
        break;

        // TH, TH - THin, THis
        case "INTERDENTAL":
        au.au10_raise_upper=.6 * intensity
        au.au18_lip_pucker=.75 * intensity
        au.au25_lips_part=.5 * intensity
        break;

        // L,T,D,Z,S,N -> Light, Top, DaD, Zebra, Sad, Nope
        case "DENTAL_ALVEOLAR":
        au.au25_lips_part=.65 * intensity
        break;

        // R,SH,ZH,CH -> Red, SHould, aSia, CHart
        case "POSTALVEOLAR":
        au.au10_raise_upper= .75 * intensity
        au.au18_lip_pucker= 1 * intensity
        au.au25_lips_part= 1 * intensity
        break;

        // K,G,NG -> Cat, Game, thiNG
        case "VELAR_GLOTTAL":
            au.au10_raise_upper=.6 * intensity
            // au.au18_lip_pucker=.5 * intensity
            au.au26_jaw_drop=.5 * intensity
            break;

        // ------------------ VOWELS ------------------------//
        // EE, I -> flEEce, bIt
        case "CLOSE_FRONT_VOWEL":
            au.au26_jaw_drop=1 * intensity
            au.au20_lip_stretcher=1 * intensity
            au.au10_raise_upper=.4 * intensity
            break;

        // OO -> bOOt
        case "CLOSE_BACK_VOWEL":
            au.au10_raise_upper=.5 * intensity
            au.au13_cheek_puffer=.8 * intensity
            au.au16_lower_lip_depr=.6 * intensity
            au.au18_lip_pucker=1 * intensity
            au.au23_lip_tightener=1 * intensity
            au.au24_lip_pressor=1 * intensity
            au.au25_lips_part=1 * intensity
            au.au26_jaw_drop=.4 * intensity
            break;

        // schwa -> ArenA
        case "MID_CENTRAL_VOWEL":
            au.au26_jaw_drop= 1 * intensity
            au.au25_lips_part= .5 * intensity
            au.au23_lip_tightener=1 * intensity
            break;

        // AE,AU,A,AY,EH -> trAp, mOUth, fAther, fAce, drEss
        //ɛ (eh)
        case "OPEN_FRONT_VOWEL":
            au.au14_dimpler= 1 * intensity
            au.au20_lip_stretcher= 1 * intensity
            au.au25_lips_part= .7 * intensity
            au.au26_jaw_drop= .75 * intensity
        break;

        // AW,OI,O -> thOUght, chOIce, gOAt
        case "OPEN_BACK_VOWEL":
            au.au26_jaw_drop= .5 * intensity
            au.au27_mouth_stretch= 1 * intensity
        break;


        default:
        break
    }
    return au
}
