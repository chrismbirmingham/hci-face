import { animated, useSpring } from "react-spring";


let cx, cy
let rx, ry, lx, ly, tx, ty, bx, by
let lbx, lby, rbx, rby, ltx, lty, rtx, rty;
let chiny, nosey;

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
    [cx, cy] = [position.x, position.y]; // location of the mouth - all coordinates relative to center
    const w=.6;

    [rx, ry] = [25*w, 1]; // lip right corner x and y
    [lx, ly] = [-25*w, 1]; // lip left corner x and y

    [tx, ty] = [0, 1]; // lip top x and y
    [rtx, rty] = [12.5*w, 2.5]; // right side top line control point
    [ltx, lty] = [-12.5*w, 2.5]; // left side top line control point

    [bx, by] = [0, 0.5];  // lip bottom x and y
    [rbx, rby] = [12.5*w, 1.5]; // right side bottom line control point
    [lbx, lby] = [-12.5*w, 1.5]; // left side bottom line control point
    chiny = -20
    nosey = 12


    
    // Adjust baselines
    ty = ty + au10_raise_upper
    rty = rty + au10_raise_upper
    lty = lty + au10_raise_upper
    
    rx = rx + au12_lip_corners_out
    ry = ry + au12_lip_corners_out*1.2
    lx = lx - au12_lip_corners_out
    ly = ly + au12_lip_corners_out*1.2
    
    cy = cy - au13_cheek_puffer*.7
    rx = rx + au13_cheek_puffer*.5
    ry = ry + au13_cheek_puffer*.6
    rx = rx - au13_cheek_puffer*.5
    ly = ly + au13_cheek_puffer*.6
    
    rx = rx + au14_dimpler
    lx = lx - au14_dimpler
    
    ry = ry - au15_lip_corner_depr
    ly = ly - au15_lip_corner_depr
    
    by = by - au16_lower_lip_depr
    rby = rby - au16_lower_lip_depr
    lby = lby - au16_lower_lip_depr
    
    by = by + au17_chin_raiser
    rby = rby + au17_chin_raiser
    lby = lby + au17_chin_raiser
    chiny = chiny + au17_chin_raiser
    
    lx = lx + au18_lip_pucker*1.5
    rx = rx - au18_lip_pucker*1.5
    by = by - au18_lip_pucker*.333
    
    rx = rx + au20_lip_stretcher
    ry = ry - au20_lip_stretcher
    lx = lx - au20_lip_stretcher
    ly = ly - au20_lip_stretcher
    
    lx = lx + au22_lip_funneler
    rx = rx - au22_lip_funneler
    by = by - au22_lip_funneler*.333
    ty = ty + au22_lip_funneler
    
    lx = lx + au23_lip_tightener*2
    rx = rx - au23_lip_tightener*2
    
    ty = ty - au24_lip_pressor*.5
    by = by + au24_lip_pressor*.25
    
    ty = ty + au25_lips_part*.5
    by = by - au25_lips_part*.25
    
    by = by - au26_jaw_drop
    rby = rby - au26_jaw_drop
    lby = lby - au26_jaw_drop
    chiny = chiny - au26_jaw_drop
    
    by = by - au27_mouth_stretch*2.5
    rby = rby - au27_mouth_stretch*2.2
    lby = lby - au27_mouth_stretch*2.2
    ty = ty + au27_mouth_stretch*1.2
    rty = rty + au27_mouth_stretch
    lty = lty + au27_mouth_stretch
    
    ty = ty - au28_lip_suck*.333
    by = by + au28_lip_suck*.333
    rty = rty - au28_lip_suck*.333
    rby = rby + au28_lip_suck*.333
    lty = lty - au28_lip_suck*.333
    lby = lby + au28_lip_suck*.333

    let chinPoints = [cx, cy, rx, chiny]
    let nosePoints = [cx, cy, rx, nosey]

    let mouthPoints = [cx, cy, rx, ry, lx, ly, tx, ty, bx, by, lbx, lby, rbx, rby, ltx, lty, rtx, rty]

    return [mouthPoints, chinPoints, nosePoints]
}

function LinifyMouth(mouthPoints)
{
    [cx, cy, rx, ry, lx, ly, tx, ty, bx, by, lbx, lby, rbx, rby, ltx, lty, rtx, rty] = mouthPoints

    var udis = [
    //left to top
    "M", cx+lx, cy-ly,
    "Q", 
    cx+ltx, cy-lty-5,
    cx+tx, cy-ty-4,
    //top to right
    "Q", 
    cx+rtx, cy-rty-5,
    cx+rx, cy-ry,
    //right to top
    "Q", 
    cx+rtx, cy-rty,
    cx+tx, cy-ty,
    //top to left
    "Q", 
    cx+ltx, cy-lty,
    cx+lx, cy-ly,
    ];

    var ldis = [
    //right to bottom
    "M", cx+rx, cy-ry,
    "Q", 
    cx+rbx, cy-rby,
    cx+bx, cy-by,

    //bottom to left
    "Q", 
    cx+lbx, cy-lby,
    cx+lx, cy-ly,

    //left to bottom
    "Q", 
    cx+lbx, cy-lby+5,
    cx+bx, cy-by+4,
    
    //bottom to right
    "Q", 
    cx+rbx, cy-rby+5,
    cx+rx, cy-ry,
    "Z"
    ]

    var mdis = [
    //right to top
    "M", cx+rx, cy-ry,
    "Q", 
    cx+rtx, cy-rty,
    cx+tx, cy-ty,
    //top to left
    "Q", 
    cx+ltx, cy-lty,
    cx+lx, cy-ly,
    //left to bottom
    "Q", 
    cx+lbx, cy-lby,
    cx+bx, cy-by,
    
    //bottom to right
    "Q", 
    cx+rbx, cy-rby,
    cx+rx, cy-ry,
    ]


    const ud = udis.join(' ');
    const ld = ldis.join(' ');
    const md = mdis.join(' ');
    // console.log(d)
    return [ud, ld, md]
}

function LinifyChin(chinPoints)
{
    [cx, cy, rx, chiny] = chinPoints

    var dis = [
    //chin
    "M", cx-rx*.3, cy-chiny,
    "C", 
    cx-rx*.3, cy-(chiny-chiny*.3),
    cx+rx*.3, cy-(chiny-chiny*.3),
    cx+rx*.3, cy-chiny,
    ];


    const d = dis.join(' ');
    // console.log(d)
    return d
}

function LinifyNose(nosePoints)
{
    [cx, cy, rx, nosey] = nosePoints

    let i = 3

    var dis = [
    //nose
    "M", cx-i*1.5, cy-(nosey+i*1.8),
    "C", 
    cx-i*2, cy-(nosey-i*.25),
    cx-i*1.8, cy-(nosey-i*.1),
    cx-i, cy-nosey,

    "M", cx-i, cy-nosey,
    "C", 
    cx+i*.3, cy-(nosey-nosey*.1),
    cx-i*.3, cy-(nosey-nosey*.1),
    cx+i, cy-nosey,

    "M", cx+i*1.5, cy-(nosey+i*1.8),
    "C", 
    cx+i*2, cy-(nosey-i*.25),
    cx+i*1.8, cy-(nosey-i*.1),
    cx+i, cy-nosey,
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