import { animated, useSpring } from "react-spring";

var ey, ex, eright_x, eright_y;
var px, py, pr, sr;
var ulc, llc;
var lr, ud;




function moveEye(new_px, new_py, r, eright_x, eright_y){
    if (new_px < 0){
        px = Math.min(Math.abs(new_px), eright_x-r) * -1
    }
    else {px = Math.min(new_px, eright_x-r)}

    if (new_py < 0){
        py = Math.min(Math.abs(new_py), eright_y-r) * -1
    }
    else {py = Math.min(new_py, eright_y-r)}

    return [px, py]


}

function circlePath(center_x, center_y, r){
    var dis = [
        "M", center_x, center_y+r,
        "A", r,r, "0","1","0",center_x, center_y-r,
        "A", r,r, "0","0","0",center_x, center_y+r,
    ];
    const d = dis.join(' ');
    // console.log(d)
    return d
}

function bezelCirclePath(center_x, center_y, r){
    var dis = [
        "M", center_x-r, center_y,
        "C", center_x-r, center_y+r, center_x+r, center_y+r, center_x+r, center_y,// bottom half
        "C", center_x+r, center_y-r, center_x-r, center_y-r, center_x-r, center_y,
    ];
    const d = dis.join(' ');
    // console.log(d)
    return d
}

function lowerLidPath(center_x, center_y, eright_x, eright_y, c){
    // v: 1 is bottom half, with higher c = more closed. 0 is upper half with lower c more closed
    var dis = [
        "M", center_x-eright_x*1.1, center_y,
        "C", center_x-eright_x*1.1, center_y+eright_y, center_x+eright_x*1.1, center_y+eright_y, center_x+eright_x*1.1, center_y,
        "Q", center_x, center_y+eright_y*(1-c)*2, center_x-eright_x*1.1, center_y,
    ];
    const d = dis.join(' ');
    // console.log(d)
    return d
}
function upperLidPath(center_x, center_y, eright_x, eright_y, c){
    // desired behavior is 0 is open, 1 is closed and 2 is all the way down
    var dis = [
        "M", center_x-eright_x*1.1, center_y,
        "C", center_x-eright_x*1.1, center_y-eright_y, center_x+eright_x*1.1, center_y-eright_y, center_x+eright_x*1.1, center_y,
        "Q", center_x, center_y-eright_y*(1-c)*2, center_x-eright_x*1.1, center_y,
    ];
    const d = dis.join(' ');
    // console.log(d)
    return d
}


function Eye({position, eyeAU}) {
    [ey, ex] = [position.y, position.x];
    // Eye Params
    [eright_x, eright_y, pr, sr] = [10, 10, 6, 1];
    [llc, ulc] = [0,0];
    [lr, ud] = [0,0];

    var au5_upper_lid_raiser = eyeAU.au5_upper_lid_raiser;
    var au6_cheek_raiser = eyeAU.au6_cheek_raiser;
    var au7_lid_tightener = eyeAU.au7_lid_tightener;
    var au41_lid_droop = eyeAU.au41_lid_droop;
    var au42_slit = eyeAU.au42_slit;
    var au43_eyes_closed = eyeAU.au43_eyes_closed;
    var au44_squint = eyeAU.au44_squint;
    var au45_blink = eyeAU.au45_blink;
    var au61_left = eyeAU.au61_left;
    var au62_right = eyeAU.au62_right;
    var au63_up = eyeAU.au63_up;
    var au64_down = eyeAU.au64_down;


    lr = lr + au62_right - au61_left;
    ud = ud + au63_up - au64_down;

    ulc = ulc - au5_upper_lid_raiser*.5;
    llc = llc + au6_cheek_raiser*.5;

    ulc=ulc + au7_lid_tightener*.3;
    llc=llc + au7_lid_tightener*.3;

    ulc = ulc + au41_lid_droop*.7;

    ulc = ulc + au42_slit*.6;
    llc = llc + au42_slit*.6;

    ulc = ulc + au43_eyes_closed;
    llc = llc + au43_eyes_closed;

    ulc = ulc + au44_squint*.7;
    llc = llc + au44_squint*.6;

    ulc = ulc + au45_blink*2;

    [px, py] = moveEye(lr*eright_x, ud* -1 * eright_y, pr, eright_x, eright_y)

    let ed = bezelCirclePath(ex,ey, eright_x)
    let bd = bezelCirclePath(ex,ey, eright_x+3)

    let pd = circlePath(ex+px, ey+py, pr)// Pupil
    let sd = circlePath(ex+px-pr/2, ey+py-pr/2, sr) // Pupil shine


    let lld = lowerLidPath(ex, ey, eright_x, eright_y, llc)
    let uld = upperLidPath(ex, ey, eright_x, eright_y, ulc)

    const animationProps = useSpring({
        border: bd,
        eyeball: ed,
        pupil: pd,
        sparkle: sd,
        lowerLid: lld,
        upperLid: uld
    }
        )

    return (
        <g>
            <animated.path className="eyeball" d={animationProps.eyeball} stroke="#000" fill="#FFF" fillOpacity="0.95" strokeWidth="0" strokeLinecap="round"/>
            <animated.path className="pupil" d={animationProps.pupil} stroke="purple" fill="#000" fillOpacity="0.95" strokeWidth="3" strokeLinecap="round"/>
            <animated.path className="sparkle" d={animationProps.sparkle} stroke="#000" fill="#FFF" fillOpacity="1" strokeWidth="0" strokeLinecap="round"/>
            <animated.path className="lowerLid" d={animationProps.lowerLid} stroke="#000" fill={position.faceColor} fillOpacity=".995" strokeWidth="0" strokeLinecap="round"/>
            <animated.path className="upperLid" d={animationProps.upperLid} stroke="#000" fill={position.faceColor} fillOpacity=".995" strokeWidth="0" strokeLinecap="round"/>
            <animated.path className="border" d={animationProps.border} stroke={position.faceColor} fill="#FFF" fillOpacity="0" strokeWidth="5" strokeLinecap="round"/>
        </g>
    )
}

export default Eye;