import { animated, useSpring } from "react-spring";


let center_x, center_y
let right_x, right_y, left_x, left_y, top_x, top_y, bottom_x, bottom_y
let lbottom_x, lbottom_y, rbottom_x, rbottom_y, ltop_x, ltop_y, rtop_x, rtop_y;
let chin_y, nose_y;

function AdjustMouth (position, mouthAU) 
{
    var scale = position.auScaler;
    var au10_raise_upper = mouthAU.au10*scale;
    var au12_lip_corners_out = mouthAU.au12*scale;
    var au13_cheek_puffer = mouthAU.au13*scale;
    var au14_dimpler = mouthAU.au14*scale;
    var au15_lip_corner_depr = mouthAU.au15*scale;
    var au16_lower_lip_depr = mouthAU.au16*scale;
    var au17_chin_raiser = mouthAU.au17*scale;
    var au18_lip_pucker = mouthAU.au18*scale;
    var au20_lip_stretcher = mouthAU.au20*scale;
    var au22_lip_funneler = mouthAU.au22*scale;
    var au23_lip_tightener = mouthAU.au23*scale;
    var au24_lip_pressor = mouthAU.au24*scale;
    var au25_lips_part = mouthAU.au25*scale;
    var au26_jaw_drop = mouthAU.au26*scale;
    var au27_mouth_stretch = mouthAU.au27*scale;
    var au28_lip_suck = mouthAU.au28*scale;

    // SET BASELINES
    [center_x, center_y] = [position.x, position.y]; // location of the mouth - all coordinates relative to center
    const width_factor=.6;

    [right_x, right_y] = [25*width_factor, 1]; // lip right corner x and y
    [left_x, left_y] = [-25*width_factor, 1]; // lip left corner x and y

    [top_x, top_y] = [0, 1]; // lip top x and y
    [rtop_x, rtop_y] = [12.5*width_factor, 2.5]; // right side top line control point
    [ltop_x, ltop_y] = [-12.5*width_factor, 2.5]; // left side top line control point

    [bottom_x, bottom_y] = [0, 0.5];  // lip bottom x and y
    [rbottom_x, rbottom_y] = [12.5*width_factor, 1.5]; // right side bottom line control point
    [lbottom_x, lbottom_y] = [-12.5*width_factor, 1.5]; // left side bottom line control point
    chin_y = -20
    nose_y = 12


    
    // Adjust baselines
    top_y = top_y + au10_raise_upper
    rtop_y = rtop_y + au10_raise_upper
    ltop_y = ltop_y + au10_raise_upper
    
    right_x = right_x + au12_lip_corners_out
    right_y = right_y + au12_lip_corners_out*1.2
    left_x = left_x - au12_lip_corners_out
    left_y = left_y + au12_lip_corners_out*1.2
    
    center_y = center_y - au13_cheek_puffer*.7
    right_x = right_x + au13_cheek_puffer*.5
    right_y = right_y + au13_cheek_puffer*.6
    right_x = right_x - au13_cheek_puffer*.5
    left_y = left_y + au13_cheek_puffer*.6
    
    right_x = right_x + au14_dimpler
    left_x = left_x - au14_dimpler
    
    right_y = right_y - au15_lip_corner_depr
    left_y = left_y - au15_lip_corner_depr
    
    bottom_y = bottom_y - au16_lower_lip_depr
    rbottom_y = rbottom_y - au16_lower_lip_depr
    lbottom_y = lbottom_y - au16_lower_lip_depr
    
    bottom_y = bottom_y + au17_chin_raiser
    rbottom_y = rbottom_y + au17_chin_raiser
    lbottom_y = lbottom_y + au17_chin_raiser
    chin_y = chin_y + au17_chin_raiser
    
    left_x = left_x + au18_lip_pucker*1.5
    right_x = right_x - au18_lip_pucker*1.5
    bottom_y = bottom_y - au18_lip_pucker*.333
    
    right_x = right_x + au20_lip_stretcher
    right_y = right_y - au20_lip_stretcher
    left_x = left_x - au20_lip_stretcher
    left_y = left_y - au20_lip_stretcher
    
    left_x = left_x + au22_lip_funneler
    right_x = right_x - au22_lip_funneler
    bottom_y = bottom_y - au22_lip_funneler*.333
    top_y = top_y + au22_lip_funneler
    
    left_x = left_x + au23_lip_tightener*2
    right_x = right_x - au23_lip_tightener*2
    
    top_y = top_y - au24_lip_pressor*.5
    bottom_y = bottom_y + au24_lip_pressor*.25
    
    top_y = top_y + au25_lips_part*.5
    bottom_y = bottom_y - au25_lips_part*.25
    
    bottom_y = bottom_y - au26_jaw_drop
    rbottom_y = rbottom_y - au26_jaw_drop
    lbottom_y = lbottom_y - au26_jaw_drop
    chin_y = chin_y - au26_jaw_drop
    
    bottom_y = bottom_y - au27_mouth_stretch*2.5
    rbottom_y = rbottom_y - au27_mouth_stretch*2.2
    lbottom_y = lbottom_y - au27_mouth_stretch*2.2
    top_y = top_y + au27_mouth_stretch*1.2
    rtop_y = rtop_y + au27_mouth_stretch
    ltop_y = ltop_y + au27_mouth_stretch
    
    top_y = top_y - au28_lip_suck*.333
    bottom_y = bottom_y + au28_lip_suck*.333
    rtop_y = rtop_y - au28_lip_suck*.333
    rbottom_y = rbottom_y + au28_lip_suck*.333
    ltop_y = ltop_y - au28_lip_suck*.333
    lbottom_y = lbottom_y + au28_lip_suck*.333

    let chinPoints = [center_x, center_y, right_x, chin_y]
    let nosePoints = [center_x, center_y, right_x, nose_y]

    let mouthPoints = [center_x, center_y, right_x, right_y, left_x, left_y, top_x, top_y, bottom_x, bottom_y, lbottom_x, lbottom_y, rbottom_x, rbottom_y, ltop_x, ltop_y, rtop_x, rtop_y]

    return [mouthPoints, chinPoints, nosePoints]
}

function LinifyMouth(mouthPoints)
{
    [center_x, center_y, right_x, right_y, left_x, left_y, top_x, top_y, bottom_x, bottom_y, lbottom_x, lbottom_y, rbottom_x, rbottom_y, ltop_x, ltop_y, rtop_x, rtop_y] = mouthPoints

    var udis = [
    //left to top
    "M", center_x+left_x, center_y-left_y,
    "Q", 
    center_x+ltop_x, center_y-ltop_y-5,
    center_x+top_x, center_y-top_y-4,
    //top to right
    "Q", 
    center_x+rtop_x, center_y-rtop_y-5,
    center_x+right_x, center_y-right_y,
    //right to top
    "Q", 
    center_x+rtop_x, center_y-rtop_y,
    center_x+top_x, center_y-top_y,
    //top to left
    "Q", 
    center_x+ltop_x, center_y-ltop_y,
    center_x+left_x, center_y-left_y,
    ];

    var ldis = [
    //right to bottom
    "M", center_x+right_x, center_y-right_y,
    "Q", 
    center_x+rbottom_x, center_y-rbottom_y,
    center_x+bottom_x, center_y-bottom_y,

    //bottom to left
    "Q", 
    center_x+lbottom_x, center_y-lbottom_y,
    center_x+left_x, center_y-left_y,

    //left to bottom
    "Q", 
    center_x+lbottom_x, center_y-lbottom_y+5,
    center_x+bottom_x, center_y-bottom_y+4,
    
    //bottom to right
    "Q", 
    center_x+rbottom_x, center_y-rbottom_y+5,
    center_x+right_x, center_y-right_y,
    "Z"
    ]

    var mdis = [
    //right to top
    "M", center_x+right_x, center_y-right_y,
    "Q", 
    center_x+rtop_x, center_y-rtop_y,
    center_x+top_x, center_y-top_y,
    //top to left
    "Q", 
    center_x+ltop_x, center_y-ltop_y,
    center_x+left_x, center_y-left_y,
    //left to bottom
    "Q", 
    center_x+lbottom_x, center_y-lbottom_y,
    center_x+bottom_x, center_y-bottom_y,
    
    //bottom to right
    "Q", 
    center_x+rbottom_x, center_y-rbottom_y,
    center_x+right_x, center_y-right_y,
    ]


    const ud = udis.join(' ');
    const ld = ldis.join(' ');
    const md = mdis.join(' ');
    // console.log(d)
    return [ud, ld, md]
}

function LinifyChin(chinPoints)
{
    [center_x, center_y, right_x, chin_y] = chinPoints

    var dis = [
    //chin
    "M", center_x-right_x*.3, center_y-chin_y,
    "C", 
    center_x-right_x*.3, center_y-(chin_y-chin_y*.3),
    center_x+right_x*.3, center_y-(chin_y-chin_y*.3),
    center_x+right_x*.3, center_y-chin_y,
    ];


    const d = dis.join(' ');
    // console.log(d)
    return d
}

function LinifyNose(nosePoints)
{
    [center_x, center_y, right_x, nose_y] = nosePoints

    let i = 3

    var dis = [
    //nose
    "M", center_x-i*1.5, center_y-(nose_y+i*1.8),
    "C", 
    center_x-i*2, center_y-(nose_y-i*.25),
    center_x-i*1.8, center_y-(nose_y-i*.1),
    center_x-i, center_y-nose_y,

    "M", center_x-i, center_y-nose_y,
    "C", 
    center_x+i*.3, center_y-(nose_y-nose_y*.1),
    center_x-i*.3, center_y-(nose_y-nose_y*.1),
    center_x+i, center_y-nose_y,

    "M", center_x+i*1.5, center_y-(nose_y+i*1.8),
    "C", 
    center_x+i*2, center_y-(nose_y-i*.25),
    center_x+i*1.8, center_y-(nose_y-i*.1),
    center_x+i, center_y-nose_y,
    ];


    const d = dis.join(' ');
    // console.log(d)
    return d
}


function Mouth({position, mouthAU}) {
   
    const [mouth_positions, chinPoints, nosePoints] = AdjustMouth(position, mouthAU)
    const [ud, ld, md] = LinifyMouth(mouth_positions)
    // const md = ud + " " + ld
    const cd = LinifyChin(chinPoints)
    const nd = LinifyNose(nosePoints)
    const animationProps = useSpring({
        mouth: md,
        upperlip: ud,
        lowerlip: ld,
        chin: cd,
        nose: nd,
    })
  
    return (
        <g>
          <animated.path d={animationProps.mouth} stroke="#000" fill="#000" fillOpacity=".8" strokeWidth="0" strokeLinecap="round"/>
          <animated.path d={animationProps.upperlip} stroke="#000" fill="#000" fillOpacity="0.4" strokeWidth=".5" strokeLinecap="round"/>
          <animated.path d={animationProps.lowerlip} stroke="#000" fill="#000" fillOpacity="0.4" strokeWidth=".5" strokeLinecap="round"/>
          {/* <animated.path d={animationProps.chin} stroke="#000" fill="#002" fillOpacity=".1" strokeWidth="0" strokeLinecap="round"/> */}
          <animated.path d={animationProps.nose} stroke="#000" fill="#00F" fillOpacity=".0" strokeWidth=".5" strokeLinecap="round"/>
        </g>
    );
}

export default Mouth;