import { animated} from "react-spring";


function maskFace( width, height){
    var dis = [
        "M", -1*width, -1*height,
        "C", -.75*width, -.9*height, -.25*width, -.9*height, 0, -.9*height,
        "L", 0, height,
        "Q", -1*width, height,  -1*width, -1*height,
    ];
    const d = dis.join(' ');
    return d
}

function yodaFace( width, height){
    var dis = [
        "M", -.75*width, -.75*height,
        "C", -.5*width, -1*height, -.25*width, -1*height, 0, -1*height,
        "L", 0, height,
        "Q", -.5*width, height,  -.75*width, .3*height,
        "C", -1.2*width, .1*height, -.8*width, -.2*height, -2.3*width, -.3*height,
        "C", -2.1*width, -.2*height, -.8*width, -.8*height, -.75*width, -.75*height,
    ];
    const d = dis.join(' ');
    return d
}

function doubleArcPath(center_x, center_y, eright_x, eright_y, close){
    // v: 1 is bottom half, with higher close = more closed. 0 is upper half with lower close more closed
    var dis = [
        "M", center_x-eright_x*1.1, center_y,
        "C", center_x-eright_x*1.1, center_y+eright_y, center_x+eright_x*1.1, center_y+eright_y, center_x+eright_x*1.1, center_y,
        "Q", center_x, center_y+eright_y*(1-close)*2, center_x-eright_x*1.1, center_y,
    ];
    const d = dis.join(' ');
    return d
}


export default function BackgroundDrawing(props) {
    const {position, shape, settings} = props;
    var {head_height, head_width, y_offset, concavity, image, shape_name} = shape;
    var {x_0, y_0, width, height} = settings;
    console.log({x_0, y_0, width, height})

    if (shape_name==="mask"){
        let head_left = maskFace(head_width/3, head_height/2.5)
        let head_right = maskFace(head_width/-3, head_height/2.5)
        return (
            <g>
                <animated.path d={head_left} stroke={position.faceColor} fill={position.faceColor} fillOpacity=".8" strokeWidth=".2" strokeLinecap="round"/>
                <animated.path d={head_right} stroke={position.faceColor} fill={position.faceColor} fillOpacity="1" strokeWidth=".2" strokeLinecap="round"/>
                <image href={image} height={height} width={width} x={x_0} y={y_0}/>
            </g>
        )
    }
    if (shape_name === "yoda") {
    let head_left  = yodaFace(head_width/3, head_height/2.5)
    let head_right  = yodaFace(head_width/-3, head_height/2.5)
    return (
            <g>
                <animated.path d={head_left} stroke={position.faceColor} fill={position.faceColor} fillOpacity="1" strokeWidth=".2" strokeLinecap="round"/>
                <animated.path d={head_right} stroke={position.faceColor} fill={position.faceColor} fillOpacity="1" strokeWidth=".2" strokeLinecap="round"/>
                <image href={image} height={height} width={width} x={x_0} y={y_0}/>
            </g>
        )
    }
    if (shape_name === "default") {
        let head_d  = doubleArcPath(0, y_0+y_offset, head_width, head_height, concavity)
        return (
                <g>
                    <animated.path d={head_d} stroke={position.faceColor} fill={position.faceColor} fillOpacity="1" strokeWidth=".2" strokeLinecap="round"/>
                    <image href={image} height={height} width={width} x={x_0} y={y_0}/>
                </g>
            )
        }
}
