
import { animated, useSpring } from "react-spring";

function LinifyMouth(mouthPoints, lipParams)
{
    var [center_x, center_y, right_x, right_y, left_x, left_y, top_x, top_y, bottom_x, bottom_y, lbottom_x, lbottom_y, rbottom_x, rbottom_y, ltop_x, ltop_y, rtop_x, rtop_y] = mouthPoints
    var [thickness, lower_convexity, upper_convexity] = lipParams


    if (top_y<bottom_y){
        let m = (top_y+bottom_y)/2
        top_y = m
        bottom_y = m
        thickness = thickness/2
    }


    var udis = [
    //left to top
    "M", center_x+left_x, center_y-left_y,
    "Q", 
    center_x+ltop_x, center_y-ltop_y-thickness,
    center_x+top_x, center_y-top_y-thickness*upper_convexity,
    //top to right
    "Q", 
    center_x+rtop_x, center_y-rtop_y-thickness,
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
    center_x+lbottom_x, center_y-lbottom_y+thickness,
    center_x+bottom_x, center_y-bottom_y+thickness*lower_convexity,
    
    //bottom to right
    "Q", 
    center_x+rbottom_x, center_y-rbottom_y+thickness,
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
    var [center_x, center_y, width, chin_y] = chinPoints
    var dis = [
    //chin
    "M", center_x-width, center_y-chin_y,
    "C", 
    center_x-width*.7, center_y*1.4,
    center_x+width*.7, center_y*1.4,
    center_x+width, center_y-chin_y,

    "M", center_x-width, center_y-chin_y/2,
    "C", 
    center_x-width*.7, center_y*1.4,
    center_x+width*.7, center_y*1.4,
    center_x+width, center_y-chin_y/2,
    ];


    const d = dis.join(' ');
    // console.log(d)
    return d
}

function LinifyNose(nosePoints, noseParams)
{
    var [center_x, center_y, right_x, nose_y] = nosePoints
    const [nose_width, nose_height] = noseParams


    var dis = [
    //nose
    "M", center_x-nose_width*1.5, center_y-(nose_y+nose_height*1.8),
    "C", 
    center_x-nose_width*2, center_y-(nose_y-nose_height*.25),
    center_x-nose_width*1.8, center_y-(nose_y-nose_height*.1),
    center_x-nose_width, center_y-nose_y,

    "M", center_x-nose_width, center_y-nose_y,
    "C", 
    center_x+nose_width*.3, center_y-(nose_y*.9),
    center_x-nose_width*.3, center_y-(nose_y*.9),
    center_x+nose_width, center_y-nose_y,

    "M", center_x+nose_width*1.5, center_y-(nose_y+nose_height*1.8),
    "C", 
    center_x+nose_width*2, center_y-(nose_y-nose_height*.25),
    center_x+nose_width*1.8, center_y-(nose_y-nose_height*.1),
    center_x+nose_width, center_y-nose_y,
    ];


    const d = dis.join(' ');
    return d
}

function animateMouth (position, mouthAU, mouth_shape){
    var [center_x, center_y] = [position.x, position.y]; // location of the mouth - all coordinates relative to center
    var {height, width, thickness, upper_convexity, lower_convexity, 
        chin_y, nose_y, nose_width, nose_height} = mouth_shape; // mouth shape parameters

    var scale = position.auScaler;

    var right_x = width
    var right_y = height/4
    var left_x = -1*width
    var left_y = height/4
    var top_x = 0
    var top_y = height
    var bottom_x = 0
    var bottom_y = height/2
    var lbottom_x = -1*width/2
    var lbottom_y = height/2
    var rbottom_x = width/2
    var rbottom_y = height/2
    var ltop_x = -1*width/2
    var ltop_y = height
    var rtop_x = width/2
    var rtop_y = height


    var au10_raise_upper = mouthAU.au10_raise_upper*scale;
    var au12_lip_corners_out = mouthAU.au12_lip_corners_out*scale;
    var au13_cheek_puffer = mouthAU.au13_cheek_puffer*scale;
    var au14_dimpler = mouthAU.au14_dimpler*scale;
    var au15_lip_corner_depr = mouthAU.au15_lip_corner_depr*scale;
    var au16_lower_lip_depr = mouthAU.au16_lower_lip_depr*scale;
    var au17_chin_raiser = mouthAU.au17_chin_raiser*scale;
    var au18_lip_pucker = mouthAU.au18_lip_pucker*scale;
    var au20_lip_stretcher = mouthAU.au20_lip_stretcher*scale;
    var au22_lip_funneler = mouthAU.au22_lip_funneler*scale;
    var au23_lip_tightener = mouthAU.au23_lip_tightener*scale;
    var au24_lip_pressor = mouthAU.au24_lip_pressor*scale;
    var au25_lips_part = mouthAU.au25_lips_part*scale;
    var au26_jaw_drop = mouthAU.au26_jaw_drop*scale;
    var au27_mouth_stretch = mouthAU.au27_mouth_stretch*scale;
    var au28_lip_suck = mouthAU.au28_lip_suck*scale;

    
    // Adjust baselines
    top_y += au10_raise_upper
    rtop_y += au10_raise_upper
    ltop_y += au10_raise_upper
    
    right_x += au12_lip_corners_out
    right_y += au12_lip_corners_out*1.2
    left_x -= au12_lip_corners_out
    left_y += au12_lip_corners_out*1.2
    
    center_y -= au13_cheek_puffer*.7
    right_x += au13_cheek_puffer*.5
    right_y += au13_cheek_puffer*.6
    right_x -= au13_cheek_puffer*.5
    left_y += au13_cheek_puffer*.6
    
    right_x += au14_dimpler
    left_x -= au14_dimpler
    
    right_y -= au15_lip_corner_depr
    left_y -= au15_lip_corner_depr
    
    bottom_y -= au16_lower_lip_depr
    rbottom_y -= au16_lower_lip_depr
    lbottom_y -= au16_lower_lip_depr
    
    bottom_y += au17_chin_raiser
    rbottom_y += au17_chin_raiser
    lbottom_y += au17_chin_raiser
    chin_y += au17_chin_raiser
    
    left_x += au18_lip_pucker*1.5
    right_x -= au18_lip_pucker*1.5
    bottom_y -= au18_lip_pucker*.333
    
    right_x += au20_lip_stretcher
    right_y -= au20_lip_stretcher
    left_x -= au20_lip_stretcher
    left_y -= au20_lip_stretcher
    
    left_x += au22_lip_funneler
    right_x -= au22_lip_funneler
    bottom_y -= au22_lip_funneler*.333
    top_y += au22_lip_funneler
    
    left_x += au23_lip_tightener*2
    right_x -= au23_lip_tightener*2
    
    top_y -= au24_lip_pressor*.5
    bottom_y += au24_lip_pressor*.25
    
    top_y += au25_lips_part*.5
    bottom_y -= au25_lips_part*.25
    
    bottom_y -= au26_jaw_drop
    rbottom_y -= au26_jaw_drop
    lbottom_y -= au26_jaw_drop
    chin_y -= au26_jaw_drop
    
    bottom_y -= au27_mouth_stretch*2.5
    rbottom_y -= au27_mouth_stretch*2.2
    lbottom_y -= au27_mouth_stretch*2.2
    top_y += au27_mouth_stretch*1.2
    rtop_y += au27_mouth_stretch
    ltop_y += au27_mouth_stretch
    
    top_y -= au28_lip_suck*.333
    bottom_y += au28_lip_suck*.333
    rtop_y -= au28_lip_suck*.333
    rbottom_y += au28_lip_suck*.333
    ltop_y -= au28_lip_suck*.333
    lbottom_y += au28_lip_suck*.333

    let chinPoints = [center_x, center_y, rtop_x, chin_y]
    let nosePoints = [center_x, center_y, right_x, nose_y]
    let mouthPoints = [center_x, center_y, right_x, right_y, left_x, left_y, top_x, top_y, bottom_x, bottom_y, lbottom_x, lbottom_y, rbottom_x, rbottom_y, ltop_x, ltop_y, rtop_x, rtop_y]
    let lipParams = [thickness, lower_convexity, upper_convexity]
    let noseParams = [nose_width, nose_height]

    const [ud, ld, md] = LinifyMouth(mouthPoints, lipParams)
    const cd = LinifyChin(chinPoints)
    const nd = LinifyNose(nosePoints, noseParams)
    const animationProps = useSpring({
        mouth: md,
        upperlip: ud,
        lowerlip: ld,
        chin: cd,
        nose: nd,
    })
  
    return animationProps
}


export default function MouthDrawing(props) {
    const {position, AU, shape, settings} = props;
    var {lip_color, mouth_color, chin_shading} = settings;

    const Animation = animateMouth(position, AU, shape)

    return (
    <g>
        <animated.path d={Animation.mouth} stroke="black" fill={mouth_color} fillOpacity=".8" strokeWidth=".1" strokeLinecap="round"/>
        <animated.path d={Animation.upperlip} stroke={lip_color} fill="#000" fillOpacity="0.4" strokeWidth=".1" strokeLinecap="round"/>
        <animated.path d={Animation.lowerlip} stroke={lip_color} fill="#000" fillOpacity="0.4" strokeWidth=".1" strokeLinecap="round"/>
        <animated.path d={Animation.nose} stroke="black" fill="#00F" fillOpacity=".0" strokeWidth=".2" strokeLinecap="round"/>
        <animated.path d={Animation.chin} stroke="gray" fill="#002" lineopacity=".5" fillOpacity={chin_shading} strokeWidth=".1" strokeLinecap="round"/>
    </g>
    );
}

