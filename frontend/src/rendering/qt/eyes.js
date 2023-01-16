import { animated, useSpring } from "react-spring";

var ey, ex, erx, ery;
var px, py, pr, sr;
var ulc, llc;
var lr, ud;




function moveEye(new_px, new_py, r, erx, ery){
    if (new_px < 0){
        px = Math.min(Math.abs(new_px), erx-r) * -1
    }
    else {px = Math.min(new_px, erx-r)}

    if (new_py < 0){
        py = Math.min(Math.abs(new_py), ery-r) * -1
    }
    else {py = Math.min(new_py, ery-r)}

    return [px, py]


}

function circlePath(cx, cy, r){
    var dis = [
        "M", cx-2, cy+r*.8,
        "A", r*.7,r, "0","1","0",cx-2, cy-r*.8,
        "A", r*.7,r, "0","0","0",cx-2, cy+r*.8,
    ];
    const d = dis.join(' ');
    // console.log(d)
    return d
}

function bezelCirclePath(cx, cy, r){
    var dis = [
        "M", cx-r, cy,
        "C", cx-r, cy+r, cx+r, cy+r, cx+r, cy,// bottom half
        "C", cx+r, cy-r, cx-r, cy-r, cx-r, cy,
    ];
    const d = dis.join(' ');
    // console.log(d)
    return d
}

function lowerLidPath(cx, cy, erx, ery, c){
    // v: 1 is bottom half, with higher c = more closed. 0 is upper half with lower c more closed
    var dis = [
        "M", cx-erx*1.1, cy,
        "C", cx-erx*1.1, cy+ery, cx+erx*1.1, cy+ery, cx+erx*1.1, cy,
        "Q", cx, cy+ery*(1-c)*2, cx-erx*1.1, cy,
    ];
    const d = dis.join(' ');
    // console.log(d)
    return d
}
function upperLidPath(cx, cy, erx, ery, c){
    // desired behavior is 0 is open, 1 is closed and 2 is all the way down
    var dis = [
        "M", cx-erx*1.1, cy,
        "C", cx-erx*1.1, cy-ery, cx+erx*1.1, cy-ery, cx+erx*1.1, cy,
        "Q", cx, cy-ery*(1-c)*2, cx-erx*1.1, cy,
    ];
    const d = dis.join(' ');
    // console.log(d)
    return d
}


function Eye({position, eyeAU}) {
    [ey, ex] = [position.y+3, position.x];
    // Eye Params
    [erx, ery, pr, sr] = [10, 10, 6, 1];
    [llc, ulc] = [0,0];
    [lr, ud] = [0,0];

    var au5_upper_lid_raiser = eyeAU.au5;
    var au6_cheek_raiser = eyeAU.au6;
    var au7_lid_tightener = eyeAU.au7;
    var au41_lid_droop = eyeAU.au41;
    var au42_slit = eyeAU.au42;
    var au43_eyes_closed = eyeAU.au43;
    var au44_squint = eyeAU.au44;
    var au45_blink = eyeAU.au45;
    var au61_left = eyeAU.au61;
    var au62_right = eyeAU.au62;
    var au63_up = eyeAU.au63;
    var au64_down = eyeAU.au64;


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

    ulc = ulc + au44_squint*.3;
    llc = llc + au44_squint*.2;

    ulc = ulc + au45_blink*2;

    [px, py] = moveEye(lr * erx, ud * -1 * ery, pr, erx, ery)

    // let ed = bezelCirclePath(ex,ey, erx)
    // let bd = bezelCirclePath(ex,ey, erx+3)

    let pd = circlePath(ex+px, ey+py, pr)// Pupil
    let sd = circlePath(ex+px-pr*.1, ey+py-pr*.3, sr*1.2) // Pupil shine


    let lld = lowerLidPath(ex+px, ey+py, erx, ery, llc)
    let uld = upperLidPath(ex+px, ey+py, erx, ery, ulc)

    const animationProps = useSpring({
        // border: bd,
        // eyeball: ed,
        pupil: pd,
        sparkle: sd,
        lowerLid: lld,
        upperLid: uld
    }
        )

    return (
        <g>
            {/* <animated.path className="eyeball" d={animationProps.eyeball} stroke="#000" fill="#FFF" fillOpacity="0.95" strokeWidth=".15" strokeLinecap="round"/> */}
            <animated.path className="pupil" d={animationProps.pupil} stroke="#777" fill="#111" fillOpacity="0.95" strokeWidth="1" strokeLinecap="round"/>
            <animated.path className="sparkle" d={animationProps.sparkle} stroke="#000" fill="#CCC" fillOpacity="1" strokeWidth="0" strokeLinecap="round"/>
            <animated.path className="lowerLid" d={animationProps.lowerLid} stroke="#000" fill="#D7E4F5" fillOpacity=".995" strokeWidth="0" strokeLinecap="round"/>
            <animated.path className="upperLid" d={animationProps.upperLid} stroke="#000" fill="#D7E4F5" fillOpacity=".995" strokeWidth="0" strokeLinecap="round"/>
            {/* <animated.path className="border" d={animationProps.border} stroke="#D7E4F5" fill="#FFF" fillOpacity="0" strokeWidth="5" strokeLinecap="round"/> */}
        </g>
    )
}

export default Eye;