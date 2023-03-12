  // Initial positions
  const positions = {
    right_eye: {x:22, y:-10},
    left_eye: {x:-22, y:-10},
    right_brow: {x:20, y:-14, auScaler: 4, flip:-1},
    left_brow: {x:-20, y:-14, auScaler: 4, flip:1},
    mouth: {x:0, y:16, auScaler: 6},
  }
  const initialMouthAU = {
    au10_raise_upper: 0,
    au12_lip_corners_out: 0.2,
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
  const initialBrowAU = {
    au1_inner_brow_raiser: 0,
    au2_outer_brow_raiser: 0,
    au4_brow_lowerer: 0,
  };  
  const initialEyeAU = {
    au5_upper_lid_raiser: 0,
    au6_cheek_raiser: 0,
    au7_lid_tightener: 0,
    au41_lid_droop: 0,
    au42_slit: 0,
    au43_eyes_closed: 0,
    au44_squint: 0.6,
    au45_blink: 0,
    au61_left: 0,
    au62_right: 0,
    au63_up: 0,
    au64_down: 0,
  };  

export {positions, initialBrowAU, initialEyeAU, initialMouthAU}