  // Initial positions

const head_settings = {
  eye_settings: {
    eye: "green",
    ball: "white", //position.faceColor, //"white",
    lid: "purple",
    sparkle: "white",
    skin: "gray",
    line_width: 0,
    iris_width: 3, // width of color ring around pupil
  },
  eye_shape: {
    boundary_rad: 10,
    pupil_rad: 5,
    sparkle_rad: 1,
    xy_skew: 1, // higher is flatter, lower is narrower
  },
  brow_settings: {
    stroke: "gray",
    fill: "gray",
    fillOpacity: ".8",
    strokeWidth: ".1",
    strokeLinecap: "round"
  },
  brow_shape: {
    ix:6, 
    iy:4, 
    mx:-3, 
    my:7, 
    ox:-15, 
    oy:4,
    thickness: 2,
  },
  mouth_settings: {
    lip_color: "gray",
    mouth_color: "#DDD",
    chin_shading: ".1",
  },
  mouth_shape: {
    height: 2,
    width: 15,
    thickness: 5,
    upper_convexity: .7, // lower for more heart shapped, higher for more triangle shapped. 1 is round
    lower_convexity: 1, // lower for more heart shapped, higher for more triangle shapped. 1 is round
    chin_y: -10,
    nose_y: 12,
    nose_width: 3,
    nose_height: 3
  },
  head_shape: {
    head_height: 110,
    head_width: 130,
    y_offset: 15,
    concavity:.96,
    shape_name:"mask"
  },
  background_settings: {
    x_0: -75,
    y_0: -130,
    width: 150,
    height: 220,
  },
  positions: {
    right_eye: {x:22, y:-10},
    left_eye: {x:-22, y:-10},
    right_brow: {x:20, y:-14, auScaler: 4, flip:-1},
    left_brow: {x:-20, y:-14, auScaler: 4, flip:1},
    mouth: {x:0, y:20, auScaler: 6},
}
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


export {head_settings, initialBrowAU, initialEyeAU, initialMouthAU}