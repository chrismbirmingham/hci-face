import { animated } from "react-spring";
import { useSpring } from "react-spring";
import { circlePath, bezelCirclePath, doubleArcPath } from "./drawingPrimitives";

function constrainEyeMovement(goal_px, goal_py, pupil_rad, eright_x, eright_y){
    var pupil_x, pupil_y;
    if (goal_px < 0){
        pupil_x = Math.min(Math.abs(goal_px), eright_x-pupil_rad) * -1
    }
    else {pupil_x = Math.min(goal_px, eright_x-pupil_rad)}

    if (goal_py < 0){
        pupil_y = Math.min(Math.abs(goal_py), eright_y-pupil_rad) * -1
    }
    else {pupil_y = Math.min(goal_py, eright_y-pupil_rad)}

    return [pupil_x, pupil_y]
}


function animateEye(position, eyeAU, eye_shape) {
    const {boundary_rad, pupil_rad, sparkle_rad, xy_skew} = eye_shape;
    var upper_lid_close= 0
    var lower_lid_close = 0;
    var leftright = 0 
    var updown = 0
    // Eye Params

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


    leftright += au62_right - au61_left;
    updown += au63_up - au64_down;

    upper_lid_close -= au5_upper_lid_raiser*.5;
    lower_lid_close += au6_cheek_raiser*.5;

    upper_lid_close += au7_lid_tightener*.3;
    lower_lid_close += au7_lid_tightener*.3;

    upper_lid_close += au41_lid_droop*.7;

    upper_lid_close += au42_slit*.6;
    lower_lid_close += au42_slit*.6;

    upper_lid_close += au43_eyes_closed;
    lower_lid_close += au43_eyes_closed;

    upper_lid_close += au44_squint*.7;
    lower_lid_close += au44_squint*.6;

    upper_lid_close += au45_blink*2;

    var pupil_x, pupil_y
    [pupil_x, pupil_y] = constrainEyeMovement(leftright*boundary_rad, -1 *updown*boundary_rad, pupil_rad+3, boundary_rad, boundary_rad)

    const animationProps = useSpring({
        border: bezelCirclePath(position.x,position.y, boundary_rad+1),
        eyeball: bezelCirclePath(position.x,position.y, boundary_rad),
        pupil: circlePath(position.x+pupil_x, position.y+pupil_y, pupil_rad, xy_skew),
        sparkle: circlePath(position.x+pupil_x-pupil_rad/2, position.y+pupil_y-pupil_rad/2, sparkle_rad, 1),
        lowerLid: doubleArcPath(position.x, position.y, boundary_rad, boundary_rad, lower_lid_close, 1),
        upperLid: doubleArcPath(position.x, position.y, boundary_rad, -1*boundary_rad, upper_lid_close, -1)
    })

    return animationProps
}
export default function EyeDrawing(props) {
    const {position, AU, shape, settings} = props;
    const Animation = animateEye(position, AU, shape)

    const {eye, ball, lid, sparkle, skin, line_width, iris_width} = settings;

    return (
        <g>
            <animated.path className="eyeball" d={Animation.eyeball} stroke="#000" fill={ball} fillOpacity="0.95" strokeWidth={line_width} strokeLinecap="round"/>
            <animated.path className="pupil" d={Animation.pupil} stroke={eye} fill="#000" fillOpacity="0.95" strokeWidth={iris_width} strokeLinecap="round"/>
            <animated.path className="sparkle" d={Animation.sparkle} stroke="#000" fill={sparkle} fillOpacity="1" strokeWidth="0" strokeLinecap="round"/>
            <animated.path className="lowerLid" d={Animation.lowerLid} stroke="#000" fill={lid} fillOpacity=".995" strokeWidth={line_width} strokeLinecap="round"/>
            <animated.path className="upperLid" d={Animation.upperLid} stroke="#000" fill={lid} fillOpacity=".995" strokeWidth={line_width} strokeLinecap="round"/>
            <animated.path className="border" d={Animation.border} stroke={skin} fill="#FFF" fillOpacity="0" strokeWidth="2.5" strokeLinecap="round"/>
        </g>
    )
}
