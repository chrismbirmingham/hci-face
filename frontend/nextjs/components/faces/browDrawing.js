import { animated, useSpring } from "react-spring";



function animateBrow(position, browAU, brow_shape) {
    
    var scale = position.auScaler;
    var flip = position.flip;
    var au1_inner_brow_raiser = browAU.au1_inner_brow_raiser*scale;
    var au2_outer_brow_raiser = browAU.au2_outer_brow_raiser*scale;
    var au4_brow_lowerer = browAU.au4_brow_lowerer*scale;

    let center_x, center_y
    [center_x, center_y] = [position.x, position.y];
    var {ix, iy, mx, my, ox, oy, thickness} = brow_shape;
    ix *= flip
    mx *= flip
    ox *= flip
    // console.log([center_x, center_y, ox, oy, mx, my, ix, iy])
    my += au1_inner_brow_raiser*.25
    iy += au1_inner_brow_raiser*.5

    my += au2_outer_brow_raiser*.5
    mx -= au2_outer_brow_raiser*.5*flip
    oy += au2_outer_brow_raiser

    mx += au4_brow_lowerer*flip
    my -= au4_brow_lowerer
    ix += au4_brow_lowerer*flip
    iy -= au4_brow_lowerer

    var dis = [
        "M", center_x+ox, center_y-oy,
        "Q", 
        center_x+mx, center_y-my,
        center_x+ix, center_y-iy,

        // Below here is the return if shape is not a line
        "L",center_x+ix*(1+thickness/5), center_y-iy-thickness,
        // "M", center_x+ox, center_y-oy,
        "Q", 
        center_x+mx, center_y-my-thickness*2,
        center_x+ox, center_y-oy-thickness,
        "L", center_x+ox, center_y-oy,//"Z"
    ];


    const d = dis.join(' ');
    
        const animationProps = useSpring({
        brow: d}
        )
    // console.log(d)
    return animationProps
}

function Brow(props) {
    const {position, AU, shape, settings} = props;
    const Animation = animateBrow(position, AU, shape)
    var {stroke, fill, fillOpacity, strokeWidth, strokeLinecap} = settings;
    return (
        <animated.path d={Animation.brow} stroke={stroke} fill={fill} fillOpacity={fillOpacity} strokeWidth={strokeWidth} strokeLinecap={strokeLinecap}/>
    );
    }

export default Brow;