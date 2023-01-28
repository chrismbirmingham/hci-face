
// text -> X-SAMPA -> Viseme -> FACS
export default function getExpresionAUs(expression, intensity = .5){

    const MouthAU = {
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
    const BrowAU = {
        au1_inner_brow_raiser: 0,
        au2_outer_brow_raiser: 0,
        au4_brow_lowerer: 0,
    };  
    const EyeAU = {
        au5_upper_lid_raiser: 0,
        au6_cheek_raiser: 0,
        au7_lid_tightener: 0,
        au41_lid_droop: 0,
        au42_slit: 0,
        au43_eyes_closed: 0,
        au44_squint: 0,
        au45_blink: 0,
        au61_left: 0,
        au62_right: 0,
        au63_up: 0,
        au64_down: 0,
    }; 

    if (expression.includes("-")){
        intensity = intensity * 0.75
    }

    if (expression.includes("neutral")){
        MouthAU.au12_lip_corners_out = MouthAU.au12_lip_corners_out + (0.2 * intensity)
        MouthAU.au26_jaw_drop = MouthAU.au26_jaw_drop + (0.2 * intensity)
        EyeAU.au44_squint = EyeAU.au44_squint + ( 0.6 * intensity)
    };

    if (expression.includes("joy") || expression.includes("happy")){ 
        EyeAU.au6_cheek_raiser = EyeAU.au6_cheek_raiser + (0.25 * intensity)
        EyeAU.au44_squint = EyeAU.au44_squint + (0.6 * intensity)
        BrowAU.au1_inner_brow_raiser = BrowAU.au1_inner_brow_raiser + (0.15 * intensity)
        MouthAU.au10_raise_upper = MouthAU.au10_raise_upper + (0.5 * intensity)
        MouthAU.au12_lip_corners_out = MouthAU.au12_lip_corners_out + (1 * intensity)
        MouthAU.au14_dimpler = MouthAU.au14_dimpler + (1 * intensity)
        MouthAU.au16_lower_lip_depr = MouthAU.au16_lower_lip_depr + (0.3 * intensity)
        MouthAU.au27_mouth_stretch = MouthAU.au28_lip_suck + (0.27 * intensity)
        MouthAU.au28_lip_suck = MouthAU.au28_lip_suck + (0.27 * intensity)
    };

    if (expression.includes("sad")){ 
        BrowAU.au1_inner_brow_raiser = BrowAU.au1_inner_brow_raiser + (.5 * intensity)
        BrowAU.au2_outer_brow_raiser = BrowAU.au2_outer_brow_raiser - (1 * intensity)
        EyeAU.au41_lid_droop = EyeAU.au41_lid_droop + (.5 * intensity)
        EyeAU.au44_squint = EyeAU.au44_squint + (.9 * intensity)
        EyeAU.au45_blink = EyeAU.au45_blink + (.3 * intensity)
        EyeAU.au64_down = EyeAU.au64_down + (1 * intensity)
        MouthAU.au12_lip_corners_out = MouthAU.au12_lip_corners_out + (.2 * intensity)
        MouthAU.au14_dimpler = MouthAU.au14_dimpler + (.24 * intensity)
        MouthAU.au15_lip_corner_depr = MouthAU.au15_lip_corner_depr + (1.5 * intensity)
        MouthAU.au16_lower_lip_depr = MouthAU.au16_lower_lip_depr + (.2 * intensity)
        MouthAU.au25_lips_part = MouthAU.au25_lips_part + (.15 * intensity)
        MouthAU.au26_jaw_drop = MouthAU.au26_jaw_drop + (.27 * intensity)
        MouthAU.au28_lip_suck = MouthAU.au28_lip_suck + (.33 * intensity)
    };
        
    if (expression.includes("surprise")){ 
        BrowAU.au1_inner_brow_raiser = BrowAU.au1_inner_brow_raiser + (1.3 * intensity)
        BrowAU.au2_outer_brow_raiser = BrowAU.au2_outer_brow_raiser + (1 * intensity)
        MouthAU.au10_raise_upper = MouthAU.au10_raise_upper + (.54 * intensity)
        MouthAU.au12_lip_corners_out = MouthAU.au12_lip_corners_out + (.2 * intensity)
        MouthAU.au26_jaw_drop = MouthAU.au26_jaw_drop + (1 * intensity)
    };

    if (expression.includes("fear")){ 
        BrowAU.au1_inner_brow_raiser = BrowAU.au1_inner_brow_raiser + (1 * intensity)
        BrowAU.au2_outer_brow_raiser = BrowAU.au2_outer_brow_raiser + (.5 * intensity)
        BrowAU.au4_brow_lowerer = BrowAU.au4_brow_lowerer + (.5 * intensity)
        EyeAU.au5_upper_lid_raiser = EyeAU.au5_upper_lid_raiser + (.36 * intensity)
        EyeAU.au6_cheek_raiser = EyeAU.au6_cheek_raiser + (1 * intensity)
        MouthAU.au20_lip_stretcher = MouthAU.au20_lip_stretcher + (1 * intensity)
        MouthAU.au22_lip_funneler = MouthAU.au22_lip_funneler + (.21 * intensity)
        MouthAU.au25_lips_part = MouthAU.au25_lips_part + (.4 * intensity)
        MouthAU.au26_jaw_drop = MouthAU.au26_jaw_drop + (1 * intensity)
    };

    if (expression.includes("anger")){ 
        BrowAU.au2_outer_brow_raiser = BrowAU.au2_outer_brow_raiser + (1 * intensity)
        BrowAU.au4_brow_lowerer = BrowAU.au4_brow_lowerer + (1 * intensity)
        EyeAU.au5_upper_lid_raiser = EyeAU.au5_upper_lid_raiser + (.36 * intensity)
        EyeAU.au7_lid_tightener = EyeAU.au7_lid_tightener + (.57 * intensity)
        EyeAU.au42_slit = EyeAU.au42_slit + (1 * intensity)
        MouthAU.au10_raise_upper = MouthAU.au10_raise_upper + (.24 * intensity)
        MouthAU.au12_lip_corners_out = MouthAU.au12_lip_corners_out + (.2 * intensity)
        MouthAU.au15_lip_corner_depr = MouthAU.au15_lip_corner_depr + (1 * intensity)
        MouthAU.au18_lip_pucker = MouthAU.au18_lip_pucker + (.2 * intensity)
        MouthAU.au24_lip_pressor = MouthAU.au24_lip_pressor + (.6 * intensity)
        MouthAU.au26_jaw_drop = MouthAU.au26_jaw_drop + (1 * intensity)
    };

    if (expression.includes("disgust")){ 
        EyeAU.au43_eyes_closed = EyeAU.au43_eyes_closed + (1 * intensity)
        BrowAU.au4_brow_lowerer = BrowAU.au4_brow_lowerer + (1 * intensity)
        MouthAU.au12_lip_corners_out = MouthAU.au12_lip_corners_out + (.2 * intensity)
        MouthAU.au13_cheek_puffer = MouthAU.au13_cheek_puffer + (.9 * intensity)
        MouthAU.au15_lip_corner_depr = MouthAU.au15_lip_corner_depr + (1 * intensity)
        MouthAU.au20_lip_stretcher = MouthAU.au20_lip_stretcher + (.4 * intensity)
        MouthAU.au24_lip_pressor = MouthAU.au24_lip_pressor + (1 * intensity)
    };

    return [MouthAU, EyeAU, BrowAU]
}
